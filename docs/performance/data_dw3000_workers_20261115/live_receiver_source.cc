// Native UHD -> production fixed-tone source -> streaming autocorrelation
// acquisition -> bounded packet jobs -> existing C++ demod -> console reporter.
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_fixed_tone_cancel_source.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <ctime>

using namespace gr::uwb;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void on_signal(int) { interrupted = 1; }
constexpr double fs = 737.28e6;
constexpr size_t pre = 36864, window = 450000, block = 65536;
constexpr size_t history_size = 1 << 21;

static double elapsed_us(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::micro>(end-start).count();
}
static double thread_cpu_us() {
    timespec t{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    return t.tv_sec*1e6 + t.tv_nsec/1e3;
}
struct PacketCost {
    double queue_us=0, resample_us=0, demod_us=0, retry_us=0, compute_us=0, cpu_us=0;
    size_t worker=0;
};

struct Options {
    std::string args = "addr=192.168.10.2,recv_frame_size=8000,num_recv_frames=32768";
    std::string channel = "3", antenna = "RX1", subdev, replay;
    std::string cir = "auto";
    double freq = 6489.6e6, gain = 60, seconds = 0;
    size_t workers = 2, capacity = 24, raw_blocks = 128, summary = 50;
};

template<class T> std::vector<T> load(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot open " + path);
    const auto bytes = f.tellg();
    if (bytes <= 0 || size_t(bytes) % sizeof(T)) throw std::runtime_error("invalid " + path);
    std::vector<T> v(size_t(bytes) / sizeof(T));
    f.seekg(0); if (!f.read(reinterpret_cast<char*>(v.data()), bytes))
        throw std::runtime_error("cannot read " + path);
    return v;
}
static bool crc_ok(const std::vector<uint8_t>& b) {
    if (b.size() < 2) return false;
    uint16_t crc = 0;
    for (size_t i = 0; i + 2 < b.size(); ++i) {
        crc ^= b[i];
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
    return crc == uint16_t(b[b.size()-2] | uint16_t(b.back()) << 8);
}
static const char* status_name(demod::DemodStatus s) {
    static const char* names[] = {"OK", "INVALID_INPUT", "TIMING_FAILED", "CFO_FAILED",
        "SFD_FAILED", "PHR_FAILED", "PAYLOAD_FAILED", "FCS_FAILED", "QUEUE_FULL",
        "INTERNAL_ERROR", "CIR_FAILED"};
    const int i = int(s); return i >= 0 && i <= 10 ? names[i] : "UNKNOWN";
}

// Every completed attempt (including queue/gap drops) contributes to a batch.
// Printing is isolated and bounded: a slow terminal cannot block the RX thread.
struct Reporter {
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<std::string> lines;
    uint64_t count = 0, pass = 0, batch_pass = 0, omitted = 0;
    uint64_t full_retries = 0, batch_retries = 0;
    uint64_t received = 0, source_errors = 0, raw_drops = 0, gaps = 0, models = 0;
    size_t queued = 0, queue_peak = 0, interval;
    std::map<std::string, size_t> batch_fail;
    bool done = false;
    std::thread thread;
    explicit Reporter(size_t n) : interval(n) { thread = std::thread([this]{ loop(); }); }
    ~Reporter() { finish(); }
    void line_locked(std::string s) {
        if (lines.size() < 4096) lines.push_back(std::move(s)); else ++omitted;
        cv.notify_one();
    }
    void info(std::string s) { std::lock_guard<std::mutex> l(mutex); line_locked(std::move(s)); }
    void batch_locked(const std::string& prefix, size_t n) {
        std::ostringstream s;
        s << prefix << " completed=" << count << " batch=" << n
          << " batch_fcs=" << batch_pass << '/' << n
          << " batch_pct=" << std::fixed << std::setprecision(1) << 100.0*batch_pass/n
          << " total_fcs=" << pass << '/' << count
          << " total_pct=" << 100.0*pass/count
          << " full_retry=" << batch_retries
          << " raw_drop_samples=" << raw_drops << " source_errors=" << source_errors
          << " gaps=" << gaps << " models=" << models
          << " queued=" << queued << " queue_peak=" << queue_peak
          << " output_omitted=" << omitted;
        for (const auto& v : batch_fail) s << ' ' << v.first << '=' << v.second;
        line_locked(s.str()); batch_pass = 0; batch_retries = 0; batch_fail.clear();
    }
    void result(uint64_t id, int64_t seed, const std::string& state,
                const demod::DemodResult* r = nullptr, double latency_ms = 0,
                bool full_retry = false, const PacketCost* cost = nullptr) {
        const bool good = r && r->status == demod::DemodStatus::Success
            && r->payload.fcs_pass && crc_ok(r->payload.bytes);
        std::lock_guard<std::mutex> l(mutex);
        ++count; pass += good; batch_pass += good;
        full_retries += full_retry; batch_retries += full_retry;
        if (!good) ++batch_fail[state == "OK" ? "CRC_CHECK_FAILED" : state];
        std::ostringstream s;
        s << std::fixed << std::setprecision(3) << "PACKET id=" << id
          << " time_ms=" << seed / (fs / 1000) << " status="
          << (state == "OK" && !good ? "CRC_CHECK_FAILED" : state);
        if (r) s << " sync=" << r->timing.detected_peaks
                 << " sfd=" << r->sfd.ok << " phr=" << r->phr.ok
                 << " len=" << r->phr.psdu_length << " cfo_khz=" << r->cfo.cfo_hz / 1000
                 << " fit=" << r->cfo.peaks_used << " fcs=" << r->payload.fcs_pass
                 << " crc=" << crc_ok(r->payload.bytes) << " latency_ms=" << latency_ms;
        if (r) s << " full_retry=" << full_retry;
        if (cost) s << " worker=" << cost->worker << " queue_us=" << cost->queue_us
                    << " resample_us=" << cost->resample_us << " demod_us=" << cost->demod_us
                    << " retry_us=" << cost->retry_us << " compute_us=" << cost->compute_us
                    << " cpu_us=" << cost->cpu_us;
        line_locked(s.str());
        if (count % interval == 0) batch_locked("SUMMARY", interval);
    }
    void snapshot(const pipeline::FixedToneCancelStats& st, uint64_t gap_count, size_t q) {
        std::lock_guard<std::mutex> l(mutex);
        received = st.source_samples; source_errors = st.source_error_reads;
        raw_drops = st.dropped_samples; models = st.model_updates; gaps = gap_count;
        queued = q; queue_peak = std::max(queue_peak, q);
    }
    void finish() {
        {
            std::lock_guard<std::mutex> l(mutex);
            if (done) return;
            if (count % interval) batch_locked("SUMMARY_PARTIAL", count % interval);
            std::ostringstream s;
            s << "FINAL completed=" << count << " fcs=" << pass << '/' << count
              << " raw_samples=" << received << " raw_drop_samples=" << raw_drops
              << " source_errors=" << source_errors << " gaps=" << gaps
              << " models=" << models << " output_omitted=" << omitted;
            s << " full_retry=" << full_retries;
            // Workers and RX are stopped here. Reserve one final line even if
            // normal packet output filled the bounded console queue.
            lines.push_back(s.str()); done = true;
        }
        cv.notify_all(); if (thread.joinable()) thread.join();
    }
    void loop() {
        auto next = Clock::now() + std::chrono::seconds(1);
        std::unique_lock<std::mutex> l(mutex);
        for (;;) {
            cv.wait_until(l, next, [&]{ return done || !lines.empty(); });
            while (!lines.empty()) {
                auto s = std::move(lines.front()); lines.pop_front();
                l.unlock(); std::cout << s << std::endl; l.lock();
            }
            if (done) return;
            if (Clock::now() >= next) {
                std::ostringstream s;
                s << "HEARTBEAT completed=" << count << " fcs=" << pass
                  << " raw_samples=" << received << " queued=" << queued
                  << " raw_drop_samples=" << raw_drops << " source_errors=" << source_errors
                  << " gaps=" << gaps << " output_omitted=" << omitted;
                l.unlock(); std::cout << s.str() << std::endl; l.lock();
                next = Clock::now() + std::chrono::seconds(1);
            }
        }
    }
};

struct Job {
    std::vector<std::complex<int16_t>> iq;
    uint64_t id = 0; int64_t seed = 0;
    Clock::time_point ready;
    Job() : iq(window) {}
};
struct Workers {
    Reporter& report;
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<Job> jobs;
    std::deque<size_t> free, pending;
    std::vector<std::thread> threads;
    size_t ready = 0;
    bool stopping = false;
    std::string error;
    Workers(Reporter& r, const Options& o, const std::vector<float>& taps,
            const std::vector<std::complex<float>>& tmpl) : report(r), jobs(o.capacity) {
        for (size_t i = 0; i < jobs.size(); ++i) free.push_back(i);
        try {
            for (size_t worker = 0; worker < o.workers; ++worker)
                threads.emplace_back([&, worker] {
                    try {
                        core::RationalResampler65_48Core rs(taps);
                        rs.set_fixed_phase_schedule(true);
                        std::vector<std::complex<float>> y(rs.expected_output_length(window));
                        demod::core::DemodScratch scratch; scratch.reserve(y.size());
                        demod::Qm35825Profile p;
                        p.code_index = 10; p.preamble_repetitions = 256; p.cir_repetitions = 246;
                        p.sfd_mode = "4z2";
                        p.cir_soft_chip_mode = o.cir == "full" ? demod::CirSoftChipMode::Full
                                                               : demod::CirSoftChipMode::Bypass;
                        p.rx_fast_sfd = true; p.rx_fast_cfo_rotation = true;
                        p.rx_seed_refine_tail = true; p.rx_softchip_on_demand = true;
                        // Blind sparse autocorrelation seeds can precede or
                        // follow the true first SYNC by several symbols.
                        p.timing_search_margin = 131072;
                        p.timing_max_backtrack_symbols = 128;
                        { std::lock_guard<std::mutex> l(mutex); ++ready; cv.notify_all(); }
                        for (;;) {
                            size_t slot;
                            { std::unique_lock<std::mutex> l(mutex);
                              cv.wait(l, [&]{return stopping || !pending.empty();});
                              if (pending.empty()) return;
                              slot = pending.front(); pending.pop_front(); }
                            auto& job = jobs[slot];
                            try {
                                const auto compute_start = Clock::now();
                                const double cpu_start = thread_cpu_us();
                                PacketCost cost;
                                cost.worker = worker;
                                cost.queue_us = elapsed_us(job.ready, compute_start);
                                rs.reset();
                                size_t made = rs.process_sc16(job.iq.data(), window, 1.f/32768,
                                                              y.data(), y.size()).produced;
                                while (made < y.size()) {
                                    const auto n = rs.flush(y.data()+made, y.size()-made);
                                    if (!n) break;
                                    made += n;
                                }
                                const auto resample_end = Clock::now();
                                cost.resample_us = elapsed_us(compute_start, resample_end);
                                auto r = demod::core::demodulate_one(y.data(), made, p,
                                    job.id, std::llround(pre*65.0/48), 0, tmpl, scratch);
                                const bool retry = o.cir == "auto" && r.timing.ok && r.sfd.ok &&
                                    (r.status != demod::DemodStatus::Success ||
                                     !r.payload.fcs_pass || !crc_ok(r.payload.bytes));
                                const auto demod_end = Clock::now();
                                cost.demod_us = elapsed_us(resample_end, demod_end);
                                if (retry) {
                                    auto full = p;
                                    full.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
                                    r = demod::core::demodulate_one(y.data(), made, full,
                                        job.id, std::llround(pre*65.0/48), 0, tmpl, scratch);
                                }
                                const auto compute_end = Clock::now();
                                cost.retry_us = retry ? elapsed_us(demod_end, compute_end) : 0;
                                cost.compute_us = elapsed_us(compute_start, compute_end);
                                cost.cpu_us = thread_cpu_us()-cpu_start;
                                report.result(job.id, job.seed, status_name(r.status), &r,
                                    elapsed_us(job.ready, compute_end)/1000, retry, &cost);
                            } catch (const std::exception& e) {
                                report.result(job.id, job.seed, "INTERNAL_ERROR");
                                report.info(std::string("WORKER_ERROR ") + e.what());
                            }
                            { std::lock_guard<std::mutex> l(mutex); free.push_back(slot); }
                        }
                    } catch (const std::exception& e) {
                        std::lock_guard<std::mutex> l(mutex);
                        error = e.what(); stopping = true; cv.notify_all();
                    }
                });
            std::unique_lock<std::mutex> l(mutex);
            cv.wait(l, [&]{return ready == o.workers || !error.empty();});
            if (!error.empty()) throw std::runtime_error(error);
        } catch (...) { stop(); throw; }
    }
    ~Workers() { stop(); }
    void stop() {
        { std::lock_guard<std::mutex> l(mutex); stopping = true; }
        cv.notify_all(); for (auto& t : threads) if (t.joinable()) t.join();
    }
    size_t backlog() { std::lock_guard<std::mutex> l(mutex); return jobs.size()-free.size(); }
    std::string failure() { std::lock_guard<std::mutex> l(mutex); return error; }
    void submit(uint64_t id, int64_t seed, const std::vector<std::complex<int16_t>>& ring) {
        size_t slot;
        { std::lock_guard<std::mutex> l(mutex);
          if (free.empty()) { report.result(id, seed, "DROPPED_QUEUE"); return; }
          slot = free.front(); free.pop_front(); }
        auto& job = jobs[slot]; job.id = id; job.seed = seed; job.ready = Clock::now();
        const size_t offset = size_t(seed-pre) & (history_size-1);
        const size_t first = std::min(window, history_size-offset);
        std::copy_n(ring.data()+offset, first, job.iq.data());
        std::copy_n(ring.data(), window-first, job.iq.data()+first);
        { std::lock_guard<std::mutex> l(mutex); pending.push_back(slot); }
        cv.notify_one();
    }
};

// Sparse streaming delayed autocorrelation, evaluated every 100 samples.
// A 60-pair rolling statistic spans 6000 samples; it is intentionally NOT
// advertised as numerically identical to the dense offline 6000-pair scan.
struct Acquisition {
    static constexpr int64_t step = 100, lag = 750, span = 6000;
    struct Pair { double re=0, im=0, e1=0, e2=0; };
    Pair pairs[60], sum;
    size_t index=0, count=0;
    int64_t probe=0, first=-1, last=-1, holdoff=0;
    double peak=0;
    void reset(int64_t start) { *this = Acquisition{}; probe=start; }
    int64_t scan(const std::vector<std::complex<int16_t>>& ring) {
        const auto a = ring[size_t(probe)&(history_size-1)];
        const auto b = ring[size_t(probe+lag)&(history_size-1)];
        const double ar=a.real(), ai=a.imag(), br=b.real(), bi=b.imag();
        Pair v{ar*br+ai*bi, ai*br-ar*bi, ar*ar+ai*ai, br*br+bi*bi};
        auto& old = pairs[index];
        sum.re += v.re-old.re; sum.im += v.im-old.im;
        sum.e1 += v.e1-old.e1; sum.e2 += v.e2-old.e2;
        old=v; index=(index+1)%60; count=std::min<size_t>(60,count+1);
        const int64_t at = probe-(span-step); probe += step;
        if (count < 60 || at < holdoff) return -1;
        const double metric = (sum.re*sum.re+sum.im*sum.im) / std::max(1.0,sum.e1*sum.e2);
        if (metric >= .15) {
            if (first < 0 || at-last > 30000) { first=at; peak=0; }
            last=at; peak=std::max(peak,metric);
            if (at-first >= 36864 && peak > .5) {
                const auto seed=first;
                holdoff=seed+589824; first=-1; peak=0; return seed;
            }
        } else if (first >= 0 && at-last > 30000) { first=-1; peak=0; }
        return -1;
    }
};

int main(int argc, char** argv)
try {
    Options o;
    for (int i=1; i<argc; ++i) {
        const std::string key=argv[i];
        auto value=[&] { if (++i>=argc) throw std::runtime_error("missing value for "+key); return std::string(argv[i]); };
        if (key=="--help") {
            std::cout << "Native DW3000 live receiver. Defaults: X410 ch3/RX1, code10/256/4z2/6M8, STS off.\n"
                "--args DEVICE_ARGS --channel 3 --antenna RX1 --subdev SPEC --freq HZ --gain DB\n"
                "--seconds 0 (until Ctrl+C) --workers 2 --queue 24 --raw-blocks 128\n"
                "--summary 50 --cir auto|bypass|full (default auto: bypass then full on failure)\n"
                "--replay FILE (paced diagnostic, no hardware)\n";
            return 0;
        } else if (key=="--args") o.args=value();
        else if (key=="--channel") o.channel=value();
        else if (key=="--antenna") o.antenna=value();
        else if (key=="--subdev") o.subdev=value();
        else if (key=="--freq") o.freq=std::stod(value());
        else if (key=="--gain") o.gain=std::stod(value());
        else if (key=="--seconds") o.seconds=std::stod(value());
        else if (key=="--workers") o.workers=std::stoul(value());
        else if (key=="--queue") o.capacity=std::stoul(value());
        else if (key=="--raw-blocks") o.raw_blocks=std::stoul(value());
        else if (key=="--summary") o.summary=std::stoul(value());
        else if (key=="--cir") o.cir=value();
        else if (key=="--replay") o.replay=value();
        else throw std::runtime_error("unknown option "+key);
    }
    if (!o.workers || o.workers>32 || !o.capacity || o.capacity>1024 ||
        o.raw_blocks<6 || o.raw_blocks>4096 || !o.summary || o.seconds<0 ||
        !std::isfinite(o.seconds) || (o.cir!="full" && o.cir!="bypass" && o.cir!="auto"))
        throw std::runtime_error("invalid workers/queue/summary/duration/CIR configuration");
    std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    const auto taps=load<float>(std::string(UWB_TESTDATA_DIR)+"/resampler_65_48/taps_quality_minorder.txt");
    const auto tmpl=load<std::complex<float>>(std::string(UWB_TESTDATA_DIR)+"/reference_preamble_code10_998p4.cf32");
    Reporter report(o.summary);
    report.info("START mode="+std::string(o.replay.empty()?"uhd":"paced_replay")+
        " channel="+o.channel+" antenna="+o.antenna+" cir="+o.cir+
        " workers="+std::to_string(o.workers)+" summary="+std::to_string(o.summary));
    Workers workers(report,o,taps,tmpl); // All worker allocations before streaming.
    pipeline::PipelineConfig cfg;
    cfg.uhd_args=o.args; cfg.uhd_channel=o.channel; cfg.uhd_antenna=o.antenna;
    cfg.uhd_subdev=o.subdev; cfg.native_rate=fs; cfg.uhd_center_freq=o.freq; cfg.uhd_gain=o.gain;
    std::unique_ptr<pipeline::ISc16StreamSource> inner;
    if (o.replay.empty()) inner=std::make_unique<pipeline::UhdDeviceSc16Source>(cfg);
    else inner=std::make_unique<pipeline::ReplaySc16Source>(o.replay,fs,true);
    core::FixedToneCancelConfig tone; tone.enable=true;
    pipeline::FixedToneCancelSc16Source source(std::move(inner),tone,block,o.raw_blocks);
    // Touch ring pages before UHD starts, rather than faulting them in recv.
    std::vector<std::complex<int16_t>> ring(history_size);
    std::vector<std::complex<int16_t>> incoming(block);
    Acquisition acq;
    if (!source.open()) throw std::runtime_error("cannot open UHD/replay or tone configuration");
    if (std::abs(source.actual_rx_rate()-fs)>.5) throw std::runtime_error("unexpected RX rate");
    if (!source.start_stream()) throw std::runtime_error("cannot start RX stream");
    report.info("STREAM_STARTED native_rate=737280000 tone_hz=-232960000 period_ms=20 training_ms=0.5");
    const auto started=Clock::now(); auto snapshot=started;
    int64_t end=0, valid_begin=0; uint64_t id=0, gaps=0;
    int64_t pending_seed=-1; uint64_t pending_id=0;
    bool failed=false;
    double frontend_us=0; uint64_t frontend_samples=0;
    while (!interrupted && (o.seconds==0 || std::chrono::duration<double>(Clock::now()-started).count()<o.seconds)) {
        pipeline::StreamReadMeta meta;
        // Decorator may change the absolute origin at a gap. Receive to a small
        // staging block so the ring's absolute mapping always stays correct.
        const auto n=source.read_samples(incoming.data(),block,meta);
        if (meta.is_gap || (n && meta.start_sample_abs!=end)) {
            ++gaps;
            report.info("RX_GAP sample="+std::to_string(meta.start_sample_abs)+
                        " error_code="+std::to_string(meta.error_code));
            if (pending_seed>=0) report.result(pending_id,pending_seed,"INVALID_GAP");
            pending_seed=-1; end=meta.start_sample_abs; valid_begin=end; acq.reset(end);
        }
        if (n) {
            const auto frontend_start = Clock::now();
            const size_t at=size_t(end)&(history_size-1), first=std::min(n,history_size-at);
            std::copy_n(incoming.data(),first,ring.data()+at);
            std::copy_n(incoming.data()+first,n-first,ring.data());
            end+=n; valid_begin=std::max(valid_begin,end-int64_t(history_size));
            while (acq.probe+Acquisition::lag<end) {
                const auto seed=acq.scan(ring);
                if (seed>=0) {
                    ++id;
                    if (pending_seed>=0) report.result(id,seed,"OVERLAPPING_CANDIDATE");
                    else if (seed-int64_t(pre)<valid_begin) report.result(id,seed,"INCOMPLETE_WINDOW");
                    else {pending_seed=seed; pending_id=id;}
                }
            }
            if (pending_seed>=0 && end>=pending_seed-int64_t(pre)+int64_t(window)) {
                if (pending_seed-int64_t(pre)<valid_begin)
                    report.result(pending_id,pending_seed,"HISTORY_OVERRUN");
                else workers.submit(pending_id,pending_seed,ring);
                pending_seed=-1;
            }
            frontend_us += elapsed_us(frontend_start, Clock::now());
            frontend_samples += n;
        }
        if (Clock::now()-snapshot>std::chrono::milliseconds(100)) {
            report.snapshot(source.stats(),gaps,workers.backlog()); snapshot=Clock::now();
        }
        std::string error;
        if (source.fatal_error(&error)) {report.info("SOURCE_ERROR "+error);failed=true;break;}
        error = workers.failure();
        if (!error.empty()) {report.info("WORKER_ERROR "+error);failed=true;break;}
        if (meta.is_eof || (!n && source.is_eof())) break;
    }
    source.close(); // Stop hardware before draining workers / printing.
    report.snapshot(source.stats(),gaps,workers.backlog());
    if (pending_seed>=0) report.result(pending_id,pending_seed,"INCOMPLETE_WINDOW");
    workers.stop();
    report.snapshot(source.stats(),gaps,workers.backlog());
    const auto& st = source.stats();
    std::ostringstream costs;
    costs << std::fixed << std::setprecision(3) << "COST_TOTAL frontend_us=" << frontend_us
          << " frontend_samples=" << frontend_samples << " estimate_us=" << st.estimate_us
          << " cancel_us=" << st.cancel_us << " cancelled_samples=" << st.cancelled_samples
          << " reader_us=" << st.reader_us;
    report.info(costs.str());
    report.finish();
    return failed ? 2 : 0;
} catch (const std::exception& e) {
    std::cerr << "ERROR " << e.what() << std::endl; return 2;
}
