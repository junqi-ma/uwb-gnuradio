// Native UHD -> production fixed-tone source -> streaming autocorrelation
// acquisition -> bounded packet jobs -> existing C++ demod -> console reporter.
//
// Two receive modes (docs/performance/X410_目标周期锁定与随机干扰双路径修改规划_2026-11-15.md):
//
//   continuous_monitor : the device streams continuously and the host drains
//                        it.  The target path cuts predicted windows after the
//                        lock; the interference path keeps autocorrelation over
//                        the whole received stream.
//   timed_burst        : finite device-timed RX windows.  Samples outside a
//                        planned window are NOT observable; the result is
//                        marked with the monitoring blind time.
//
// Two synchronization strategies:
//
//   acquire (default)  : legacy wide acquisition, exactly as before.
//   locked             : target slots use SyncStrategy::LockedBounded (bounded
//                        ROI, no wide scan, no long backtrack); acquisition and
//                        interference windows keep the legacy strategy.
//
// The lock state machine lives in core::TargetLockCore (GNU Radio independent,
// scheduler-thread only).  DSP workers never touch the USRP or the lock; they
// push bounded observations back.
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_fixed_tone_cancel_source.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_qm35_acquisition_tracker.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_target_lock_core.h>
#include <gnuradio/uwb/uwb_timed_rx_plan.h>
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
constexpr double work_fs = 998.4e6;
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
    // Default is the measured-fastest pairing: the fused CMF requires non-Bypass
    // soft chips, so `full` is its prerequisite.  With `auto` (Bypass first) the
    // fused path is never admitted and demod is 1556 us instead of ~1100 us.
    // A refused integer grid degrades Full->Bypass per packet in the demod
    // core, so an untrimmed clock costs ~1.5 ms/packet, not ~15 ms.
    std::string cir = "full";
    std::string mode = "continuous";   // continuous | timed
    std::string sync = "acquire";      // acquire | locked
    double freq = 6489.6e6, gain = 60, seconds = 0;
    double period_ms = 5.0;
    size_t workers = 2, capacity = 24, raw_blocks = 128, summary = 50;
    size_t lock_observations = defaults::kLockObservations;
    size_t holdover = defaults::kHoldoverMissCount;
    size_t reacquire = defaults::kReacquireMissCount;
    size_t locked_roi = 256;
    size_t post_guard = 8192;
    bool interference = true;
    bool tone = true;        // fixed-tone cancellation (continuous mode only)
    double tone_period_ms = 20.0;   // model update cadence (diagnostic sweep)
    double tone_training_ms = 0.5;  // training window at each period start
    bool fused = true;       // fused payload + PHR CMF/despread
    // Integer chip-grid admission tolerance in ppm.  Defaults to the physical
    // ceiling (1 sample of drift over the 127 B payload); see
    // Qm35825Profile::fused_gate_ppm for why a tighter gate is counterproductive.
    double fused_gate_ppm = demod::core::kIntegerGridCeilingPpm;
    // On-demand soft-chip prefix.  Tri-state: "auto" follows --fused, because
    // the short prefix only pays off together with the fused CMF.  Measured on
    // the X410 1 s capture with fused=off, the on-demand prefix always failed
    // and fell back to the full stream, costing +7.3% demod for nothing.
    std::string ondemand = "auto";
    bool ondemand_resolved = false; // set from --ondemand/--fused in main
    double timed_lead_us = 300.0;
    double timed_t0_ms = 0.0;
    size_t timed_lookahead = 4, timed_alignment = 4;
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
                bool full_retry = false, const PacketCost* cost = nullptr,
                const std::string& tag = std::string()) {
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
        if (!tag.empty()) s << ' ' << tag;
        if (r) s << " sync=" << r->timing.detected_peaks
                 << " sfd=" << r->sfd.ok << " phr=" << r->phr.ok
                 << " len=" << r->phr.psdu_length << " cfo_khz=" << r->cfo.cfo_hz / 1000
                 << " fit=" << r->cfo.peaks_used << " fcs=" << r->payload.fcs_pass
                 << " crc=" << crc_ok(r->payload.bytes) << " latency_ms=" << latency_ms;
        if (r) s << " full_retry=" << full_retry;
        if (r) s << " sync_strategy=" << demod::sync_strategy_name(r->sync_strategy)
                 << " roi=" << r->timing.bounded_roi_samples
                 << " bounded_ok=" << r->timing.bounded_success
                 << " bounded_fail=" << r->timing.bounded_failures
                 << " legacy_scans=" << r->timing.bounded_legacy_scans
                 << " backtrack=" << r->timing.backtrack_symbols;
        // Per-stage decomposition of the demod.  Parents and children are never
        // summed together; `stage_total_us` is the authoritative total.
        if (r) s << " st_total=" << r->stage_total_us
                 << " st_timing=" << r->stage_timing_us
                 << " st_cfo=" << r->stage_cfo_us
                 << " st_sfd=" << r->stage_sfd_us
                 << " st_cir=" << r->stage_cir_us
                 << " st_nssfd=" << r->stage_ns_sfd_us
                 << " st_phr=" << r->stage_phr_us
                 << " st_payload=" << r->stage_payload_us
                 << " st_fallback=" << r->stage_softchip_fallback_us
                 << " st_fb_count=" << r->stage_softchip_fallbacks
                 << " ondemand_ext=" << r->cir.on_demand_extended
                 << " ondemand_fb=" << r->cir.on_demand_fallbacks
                 << " chips=" << r->cir.soft_chip_count
                 // Chip-grid offset in ppm.  The fused CMF requires an exact
                 // integer grid (|spc-2| <= 1e-9, i.e. <= 0.0005 ppm); a real
                 // X410 clock is tens of ppm off, so a nonzero value here is
                 // exactly why the fused path is refused.
                 << " spc_ppm=" << (r->cir.samples_per_chip > 0.0
                                        ? (r->cir.samples_per_chip / 2.0 - 1.0) * 1e6
                                        : 0.0)
                 << " taps=" << r->cir.cir_filter_taps.size()
                 << " grid_reject=" << r->stage_grid_rejects
                 << " st_fused=" << r->stage_fused_attempts
                 // SFD children (already inside st_sfd; never added again).
                 << " sfd_boot=" << r->stage_sfd_bootstrap_us
                 << " sfd_tail=" << r->stage_sfd_tail_retrack_us
                 << " sfd_cforef=" << r->stage_sfd_cfo_refit_us
                 << " sfd_final=" << r->stage_sfd_final_us
                 << " sfd_other=" << r->stage_sfd_anchor_other_us
                 << " tail_fb=" << r->stage_tail_retrack_fallbacks
                 << " sfd_win=" << r->sfd.search_windows
                 << " sfd_coarse=" << r->sfd.coarse_correlations
                 << " sfd_fine=" << r->sfd.fine_correlations
                 // Link quality proxies: correlation metrics, not energies.
                 << std::fixed << std::setprecision(4)
                 << " tmetric=" << r->timing.metric
                 << " sfdmetric=" << r->sfd.metric
                 << " cirmetric=" << r->cir.cir_peak_metric
                 << " phr_corr=" << r->phr.secded_corrected
                 << " phr_unc=" << r->phr.secded_uncorrectable
                 << " psdu=";
        if (r) {
            char hex[3 * 12 + 1];
            const size_t m = std::min<size_t>(r->payload.bytes.size(), 12);
            for (size_t q = 0; q < m; ++q)
                std::snprintf(hex + 3 * q, 4, "%02x ", r->payload.bytes[q]);
            hex[3 * m] = 0;
            s << hex;
        }
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
    size_t count = 0;
    uint64_t id = 0;
    int64_t seed = 0;            // absolute native predicted/acquired origin
    int64_t window_start = 0;    // absolute native window origin
    int64_t predicted_rel_work = 0;
    uint64_t slot_index = std::numeric_limits<uint64_t>::max();
    uint64_t generation = 0;
    bool target_slot = false;
    bool feeds_lock = false;
    Clock::time_point ready;
    Job() : iq(window) {}
};

struct PendingSubmit {
    uint64_t id = 0;
    int64_t seed = 0;
    int64_t window_start = 0;
    size_t count = 0;
    int64_t predicted_rel_work = 0;
    uint64_t slot_index = std::numeric_limits<uint64_t>::max();
    uint64_t generation = 0;
    double metric = 0.0;
    bool target_slot = false;
    bool feeds_lock = false;
    bool is_candidate = false; // autocorrelation candidate, classify at submit
};

struct Workers {
    Reporter& report;
    core::TargetLockCore& lock;
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<Job> jobs;
    std::deque<size_t> free, pending;
    std::vector<std::thread> threads;
    size_t ready = 0;
    bool stopping = false;
    std::string error;
    // Native <-> work coordinate conversion for lock feedback.  Both the
    // prediction and the observation go through the same function, so a
    // constant group delay cancels in the residual; the absolute t0 stays
    // consistent with the mapped observation convention.
    double resample_group_delay = 0.0;
    std::atomic<uint64_t> interference_events{0};
    std::atomic<uint64_t> target_dedup{0};

    Workers(Reporter& r, core::TargetLockCore& lk, const Options& o,
            const std::vector<float>& taps,
            const std::vector<std::complex<float>>& tmpl)
        : report(r), lock(lk), jobs(o.capacity) {
        resample_group_delay = taps.empty() ? 0.0 : 0.5 * (double(taps.size()) - 1.0);
        for (size_t i = 0; i < jobs.size(); ++i) free.push_back(i);
        try {
            for (size_t worker = 0; worker < o.workers; ++worker)
                threads.emplace_back([&, worker] {
                    try {
                        core::RationalResampler65_48Core rs(taps);
                        rs.set_fixed_phase_schedule(true);
                        std::vector<std::complex<float>> y(
                            rs.expected_output_length(window));
                        demod::core::DemodScratch scratch; scratch.reserve(y.size());
                        // Acquisition / interference profile: unchanged legacy
                        // wide acquisition.
                        demod::Qm35825Profile acq;
                        acq.code_index = 10; acq.preamble_repetitions = 256;
                        acq.cir_repetitions = 246;
                        acq.sfd_mode = "4z2";
                        acq.cir_soft_chip_mode = o.cir == "full"
                            ? demod::CirSoftChipMode::Full
                            : demod::CirSoftChipMode::Bypass;
                        acq.rx_fast_sfd = true; acq.rx_fast_cfo_rotation = true;
                        acq.rx_seed_refine_tail = true;
                        acq.rx_softchip_on_demand = o.ondemand_resolved;
                        acq.rx_fused_payload_cmf = o.fused;
                        acq.rx_fused_phr_cmf = o.fused;
                        acq.fused_gate_ppm = o.fused_gate_ppm;
                        // Blind sparse autocorrelation seeds can precede or
                        // follow the true first SYNC by several symbols.
                        acq.timing_search_margin = 131072;
                        acq.timing_max_backtrack_symbols = 128;
                        // Locked profile: same PHY, bounded first-SYNC ROI.
                        demod::Qm35825Profile locked = acq;
                        locked.sync_strategy = demod::SyncStrategy::LockedBounded;
                        locked.locked_roi_samples = o.locked_roi;
                        locked.locked_allow_backtrack = false;
                        locked.timing_search_margin = 0;
                        locked.timing_max_backtrack_symbols = 0;
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
                                const bool bounded =
                                    job.target_slot && o.sync == "locked";
                                const demod::Qm35825Profile& p =
                                    bounded ? locked : acq;
                                rs.reset();
                                const size_t y_need =
                                    rs.expected_output_length(job.count);
                                size_t made = rs.process_sc16(job.iq.data(), job.count,
                                                              1.f/32768, y.data(),
                                                              y.size()).produced;
                                while (made < y_need) {
                                    const auto n = rs.flush(y.data()+made, y.size()-made);
                                    if (!n) break;
                                    made += n;
                                }
                                const auto resample_end = Clock::now();
                                cost.resample_us = elapsed_us(compute_start, resample_end);
                                auto r = demod::core::demodulate_one(y.data(), made, p,
                                    job.id, job.predicted_rel_work, 0, tmpl, scratch);
                                const bool retry = o.cir == "auto" && r.timing.ok && r.sfd.ok &&
                                    (r.status != demod::DemodStatus::Success ||
                                     !r.payload.fcs_pass || !crc_ok(r.payload.bytes));
                                const auto demod_end = Clock::now();
                                cost.demod_us = elapsed_us(resample_end, demod_end);
                                if (retry) {
                                    auto full = p;
                                    full.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
                                    r = demod::core::demodulate_one(y.data(), made, full,
                                        job.id, job.predicted_rel_work, 0, tmpl, scratch);
                                }
                                const auto compute_end = Clock::now();
                                cost.retry_us = retry ? elapsed_us(demod_end, compute_end) : 0;
                                cost.compute_us = elapsed_us(compute_start, compute_end);
                                cost.cpu_us = thread_cpu_us()-cpu_start;

                                // ---- lock feedback ------------------------
                                core::TargetObservation obs;
                                obs.packet_id = job.id;
                                obs.schedule_generation = job.generation;
                                obs.slot_index = job.slot_index;
                                obs.window_origin = job.window_start;
                                obs.predicted_origin = job.seed;
                                obs.timing_ok = r.timing.ok;
                                obs.sfd_ok = r.sfd.ok;
                                obs.phr_ok = r.phr.ok;
                                obs.fcs_pass = r.payload.fcs_pass &&
                                               crc_ok(r.payload.bytes);
                                obs.code_index = 10;
                                obs.preamble_repetitions = 256;
                                obs.sfd_mode = "4z2";
                                obs.quality = r.timing.metric;
                                obs.from_target_window = job.target_slot;
                                if (r.timing.ok) {
                                    const int64_t native_rel =
                                        core::map_obs_sample_to_native(
                                            r.timing.preamble_start_sample,
                                            work_fs, fs, resample_group_delay);
                                    obs.observed_origin = job.window_start + native_rel;
                                    obs.residual = obs.observed_origin - job.seed;
                                }
                                if (job.feeds_lock)
                                    lock.push_observation(obs);

                                std::ostringstream tag;
                                if (r.timing.ok) {
                                    tag << "start="
                                        << (job.window_start +
                                            core::map_obs_sample_to_native(
                                                r.timing.preamble_start_sample,
                                                work_fs, fs, resample_group_delay))
                                        << ' ';
                                }
                                if (job.target_slot) {
                                    tag << "path=target slot=" << job.slot_index
                                        << " gen=" << job.generation
                                        << " pred=" << job.seed
                                        << " win=" << job.window_start
                                        << " residual=" << obs.residual;
                                } else if (job.slot_index ==
                                           std::numeric_limits<uint64_t>::max()) {
                                    tag << "path=acquire";
                                } else {
                                    tag << "path=interference slot=" << job.slot_index
                                        << " offset=" << (job.seed - job.window_start);
                                }
                                report.result(job.id, job.seed, status_name(r.status), &r,
                                    elapsed_us(job.ready, compute_end)/1000, retry, &cost,
                                    tag.str());
                                // Interference / identity diagnostics.  A
                                // decode that is not the locked target identity
                                // is an interference event; a same-identity
                                // decode far from the grid is reported as
                                // ambiguous and never moves the lock.
                                if (!job.target_slot && r.timing.ok && r.sfd.ok) {
                                    const bool identity = r.payload.fcs_pass &&
                                        crc_ok(r.payload.bytes);
                                    if (!identity) {
                                        ++interference_events;
                                        std::ostringstream s;
                                        s << std::fixed << std::setprecision(3)
                                          << "INTERFERENCE start="
                                          << (obs.observed_origin >= 0 ? obs.observed_origin
                                                                       : job.seed)
                                          << " metric=" << r.timing.metric
                                          << " sfd=" << r.sfd.ok
                                          << " fcs=" << r.payload.fcs_pass
                                          << " reason=identity_mismatch";
                                        report.info(s.str());
                                    }
                                }
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
    /** Submit from the ring buffer at absolute native `window_start`. */
    bool submit(uint64_t id, int64_t window_start, size_t count, int64_t seed,
                int64_t predicted_rel_work, uint64_t slot_index, uint64_t generation,
                bool target_slot, bool feeds_lock,
                const std::vector<std::complex<int16_t>>& ring) {
        size_t slot;
        { std::lock_guard<std::mutex> l(mutex);
          if (free.empty()) return false;
          slot = free.front(); free.pop_front(); }
        auto& job = jobs[slot];
        job.id = id; job.seed = seed; job.window_start = window_start;
        job.count = std::min(count, window);
        job.predicted_rel_work = predicted_rel_work;
        job.slot_index = slot_index; job.generation = generation;
        job.target_slot = target_slot; job.feeds_lock = feeds_lock;
        job.ready = Clock::now();
        const size_t offset = size_t(window_start) & (history_size-1);
        const size_t first = std::min(job.count, history_size-offset);
        std::copy_n(ring.data()+offset, first, job.iq.data());
        std::copy_n(ring.data(), job.count-first, job.iq.data()+first);
        { std::lock_guard<std::mutex> l(mutex); pending.push_back(slot); }
        cv.notify_one();
        return true;
    }
    /** Submit from an already assembled buffer (timed_burst path). */
    bool submit_buffer(uint64_t id, int64_t window_start, const std::complex<int16_t>* src,
                       size_t count, int64_t seed, int64_t predicted_rel_work,
                       uint64_t slot_index, uint64_t generation, bool target_slot,
                       bool feeds_lock) {
        size_t slot;
        { std::lock_guard<std::mutex> l(mutex);
          if (free.empty()) return false;
          slot = free.front(); free.pop_front(); }
        auto& job = jobs[slot];
        job.id = id; job.seed = seed; job.window_start = window_start;
        job.count = std::min(count, window);
        job.predicted_rel_work = predicted_rel_work;
        job.slot_index = slot_index; job.generation = generation;
        job.target_slot = target_slot; job.feeds_lock = feeds_lock;
        job.ready = Clock::now();
        std::copy_n(src, job.count, job.iq.data());
        { std::lock_guard<std::mutex> l(mutex); pending.push_back(slot); }
        cv.notify_one();
        return true;
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
    int64_t scan(const std::vector<std::complex<int16_t>>& ring, double* out_metric = nullptr) {
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
                holdoff=seed+589824; first=-1;
                if (out_metric) *out_metric = peak;
                peak=0; return seed;
            }
        } else if (first >= 0 && at-last > 30000) { first=-1; peak=0; }
        return -1;
    }
};

// ---------------------------------------------------------------------------
// Device-timed replay source (timed_burst without hardware).
//
// The file is a continuous timeline; this source only delivers the planned
// windows and marks the space between them as a planned gap.  It is the
// timestamped fake source the plan requires for exercising the plan/assembly
// and error paths without claiming a timed UHD board run.
// ---------------------------------------------------------------------------
class TimedReplaySc16Source : public pipeline::ISc16StreamSource {
public:
    TimedReplaySc16Source(const std::string& file, double rate,
                          const core::TimedRxPlanConfig& cfg, bool paced = true)
        : file_(file), rate_(rate), plan_(cfg), paced_(paced) {}
    bool open() override {
        f_.open(file_, std::ios::binary | std::ios::ate);
        if (!f_) return false;
        const auto bytes = f_.tellg();
        if (bytes <= 0) return false;
        total_ = size_t(bytes) / sizeof(std::complex<int16_t>);
        eof_ = false;
        t0_ = Clock::now();
        return true;
    }
    void close() override { if (f_.is_open()) f_.close(); eof_ = true; }
    double actual_rx_rate() const override { return rate_; }
    bool is_eof() const override { return eof_; }
    // The scheduler thread queues windows while the decorator's reader thread
    // assembles them, so every access to the plan is serialised.  Holding the
    // lock across the file read is deliberate: a planned window is far larger
    // than one recv block, and the queue side must never observe a half-built
    // plan.
    bool queue_window(int64_t start, size_t samples, uint64_t generation) {
        core::TimedRxRequest r;
        r.logical_start = start;
        r.logical_samples = samples;
        r.generation = generation;
        std::lock_guard<std::mutex> l(plan_mutex_);
        return plan_.queue_request(r);
    }
    core::TimedRxPlanStats plan_stats() {
        std::lock_guard<std::mutex> l(plan_mutex_);
        return plan_.stats();
    }
    uint64_t delivered_samples() const { return delivered_.load(); }
    size_t read_samples(std::complex<int16_t>* dest, size_t max_samples,
                        pipeline::StreamReadMeta& meta) override {
        meta = pipeline::StreamReadMeta{};
        if (!f_.is_open() || max_samples == 0) { eof_ = true; meta.is_eof = true; return 0; }
        std::lock_guard<std::mutex> l(plan_mutex_);
        // Serve the tail of an assembled window first: a planned window is far
        // larger than one recv block.
        if (out_valid_) {
            const size_t n = std::min(max_samples, out_buf_.size() - out_off_);
            std::copy_n(out_buf_.begin() + static_cast<long>(out_off_), n, dest);
            meta.start_sample_abs = out_start_ + int64_t(out_off_);
            meta.has_time_spec = true;
            meta.time_spec_s = double(meta.start_sample_abs) / rate_;
            meta.is_gap = (out_off_ == 0) && out_gap_;
            meta.gap_samples = (out_off_ == 0) ? out_gap_samples_ : 0;
            meta.error_code = out_err_;
            out_off_ += n;
            if (out_off_ >= out_buf_.size())
                out_valid_ = false;
            delivered_.fetch_add(n);
            return n;
        }
        std::vector<std::complex<int16_t>> chunk(65536);
        // Issue every request whose window is still in the future.  The replay
        // has no real-time constraint, so "now" is placed just before the
        // requested start and the lead time is always satisfied.
        for (;;) {
            core::TimedRxWindowSpec spec;
            if (!plan_.next_command(next_issue_s_ - 1.0, spec)) break;
            // Read the planned window from the file and deliver it in chunks.
            size_t remaining = spec.device_samples;
            int64_t pos = spec.device_start;
            while (remaining > 0) {
                const size_t n = std::min(remaining, chunk.size());
                const size_t got = read_file(pos, chunk.data(), n);
                core::TimedRxChunk c;
                c.device_start = pos;
                c.count = got;
                c.data = chunk.data();
                c.has_time_spec = true;
                c.time_spec_s = double(pos) / rate_;
                plan_.on_chunk(c);
                if (got < n) { plan_.flush_partial(); break; }
                pos += int64_t(got);
                remaining -= got;
            }
            next_issue_s_ = double(spec.device_start + int64_t(spec.device_samples)) / rate_;
        }
        core::TimedRxAssembled w;
        if (!plan_.take_assembled(w)) {
            if (at_file_end_) { eof_ = true; meta.is_eof = true; }
            return 0;
        }
        // Paced replay: the fake device delivers a window no earlier than its
        // device time, so queue depth and drops reflect the plan rather than
        // the speed of a file read.  Unpaced mode is for plan-only QA.
        if (paced_) {
            const auto due = t0_ + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(w.spec.device_time_s));
            std::this_thread::sleep_until(due);
        }
        out_buf_ = std::move(w.samples);
        out_start_ = w.spec.device_start;
        out_gap_ = w.planned_gap_samples > 0;
        out_gap_samples_ = w.planned_gap_samples;
        out_err_ = int(w.terminal);
        out_off_ = 0;
        out_valid_ = !out_buf_.empty();
        if (!out_valid_)
            return 0;
        const size_t n = std::min(max_samples, out_buf_.size());
        std::copy_n(out_buf_.begin(), n, dest);
        meta.start_sample_abs = out_start_;
        meta.has_time_spec = true;
        meta.time_spec_s = w.spec.device_time_s;
        meta.is_gap = out_gap_;
        meta.gap_samples = out_gap_samples_;
        meta.error_code = out_err_;
        out_off_ = n;
        if (out_off_ >= out_buf_.size())
            out_valid_ = false;
        delivered_.fetch_add(n);
        return n;
    }
private:
    size_t read_file(int64_t start, std::complex<int16_t>* dst, size_t n) {
        if (start < 0 || size_t(start) >= total_) { at_file_end_ = true; return 0; }
        const size_t avail = total_ - size_t(start);
        n = std::min(n, avail);
        f_.clear();
        f_.seekg(std::streamoff(start) * std::streamoff(sizeof(std::complex<int16_t>)));
        f_.read(reinterpret_cast<char*>(dst),
                std::streamsize(n * sizeof(std::complex<int16_t>)));
        const size_t got = size_t(f_.gcount()) / sizeof(std::complex<int16_t>);
        if (got < n || size_t(start) + got >= total_) at_file_end_ = true;
        return got;
    }
    std::string file_;
    double rate_;
    std::mutex plan_mutex_;
    core::TimedRxPlanCore plan_;
    std::ifstream f_;
    size_t total_ = 0;
    bool eof_ = false;
    bool at_file_end_ = false;
    double next_issue_s_ = 0.0;
    std::vector<std::complex<int16_t>> out_buf_;
    int64_t out_start_ = 0;
    size_t out_off_ = 0;
    bool out_valid_ = false;
    bool out_gap_ = false;
    int64_t out_gap_samples_ = 0;
    int out_err_ = 0;
    std::atomic<uint64_t> delivered_{0};
    bool paced_ = true;
    Clock::time_point t0_{};
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
                "--summary 50 --cir full|auto|bypass (default full: the fused CMF needs it)\n"
                "--replay FILE (paced diagnostic, no hardware)\n"
                "--mode continuous|timed (default continuous; timed needs --replay)\n"
                "--sync acquire|locked (default acquire; locked uses a bounded target ROI)\n"
                "--period-ms 5 --pre-guard N --post-guard N (native samples)\n"
                "--locked-roi N (bounded first-SYNC half-width, default 256)\n"
                "--lock-observations 3 --holdover 3 --reacquire 8\n"
                "--interference on|off (default on)\n"
                "--tone on|off (fixed-tone cancellation, default on)\n"
                "--tone-period-ms 20 --tone-training-ms 0.5 (model cadence)\n"
                "--fused on|off (fused payload+PHR CMF, default on)\n"
                "--fused-gate-ppm 6.468 (integer chip-grid admission; ceiling = 1 sample of drift)\n"
                "--ondemand auto|on|off (on-demand soft-chip prefix, default auto = follow --fused)\n"
                "--timed-lead-us 300 --timed-lookahead 4 --timed-alignment 4\n"
                "--timed-t0-ms 0 (first expected target time; timed_burst has no blind acquisition)\n";
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
        else if (key=="--mode") o.mode=value();
        else if (key=="--sync") o.sync=value();
        else if (key=="--period-ms") o.period_ms=std::stod(value());
        else if (key=="--post-guard") o.post_guard=std::stoul(value());
        else if (key=="--locked-roi") o.locked_roi=std::stoul(value());
        else if (key=="--lock-observations") o.lock_observations=std::stoul(value());
        else if (key=="--holdover") o.holdover=std::stoul(value());
        else if (key=="--reacquire") o.reacquire=std::stoul(value());
        else if (key=="--interference") o.interference=(value()!="off");
        else if (key=="--tone") o.tone=(value()!="off");
        else if (key=="--tone-period-ms") o.tone_period_ms=std::stod(value());
        else if (key=="--tone-training-ms") o.tone_training_ms=std::stod(value());
        else if (key=="--fused") o.fused=(value()!="off");
        else if (key=="--fused-gate-ppm") o.fused_gate_ppm=std::stod(value());
        else if (key=="--ondemand") o.ondemand=value();
        else if (key=="--timed-lead-us") o.timed_lead_us=std::stod(value());
        else if (key=="--timed-lookahead") o.timed_lookahead=std::stoul(value());
        else if (key=="--timed-alignment") o.timed_alignment=std::stoul(value());
        else if (key=="--timed-t0-ms") o.timed_t0_ms=std::stod(value());
        else throw std::runtime_error("unknown option "+key);
    }
    if (!o.workers || o.workers>32 || !o.capacity || o.capacity>1024 ||
        o.raw_blocks<6 || o.raw_blocks>4096 || !o.summary || o.seconds<0 ||
        !std::isfinite(o.seconds) || (o.cir!="full" && o.cir!="bypass" && o.cir!="auto"))
        throw std::runtime_error("invalid workers/queue/summary/duration/CIR configuration");
    if (o.mode!="continuous" && o.mode!="timed")
        throw std::runtime_error("--mode must be continuous or timed");
    if (o.sync!="acquire" && o.sync!="locked")
        throw std::runtime_error("--sync must be acquire or locked");
    if (!(o.tone_period_ms > 0.0) || !(o.tone_training_ms > 0.0) ||
        o.tone_training_ms >= o.tone_period_ms)
        throw std::runtime_error("invalid --tone-period-ms/--tone-training-ms");
    if (o.ondemand!="auto" && o.ondemand!="on" && o.ondemand!="off")
        throw std::runtime_error("--ondemand must be auto, on or off");
    // The gate is a drift budget: above the physical ceiling (1 sample over the
    // 127 B payload = 6.5 ppm) the integer grid cannot stay on the correlation
    // peak, so accepting it would produce silently wrong soft chips.
    if (!(o.fused_gate_ppm > 0.0) || !std::isfinite(o.fused_gate_ppm) ||
        o.fused_gate_ppm > demod::core::kIntegerGridCeilingPpm)
        throw std::runtime_error("--fused-gate-ppm must be in (0, 6.468] "
                                 "(1 sample of drift over the 127 B payload)");
    o.ondemand_resolved =
        (o.ondemand == "auto") ? o.fused : (o.ondemand == "on");
    if (!(o.period_ms>0.0) || !std::isfinite(o.period_ms))
        throw std::runtime_error("invalid --period-ms");
    if (o.locked_roi==0 || o.lock_observations<2 || o.holdover==0 ||
        o.reacquire<=o.holdover || o.timed_lookahead==0 || o.timed_alignment==0 ||
        o.timed_t0_ms<0.0 || !std::isfinite(o.timed_t0_ms))
        throw std::runtime_error("invalid lock/timed configuration");
    // The timed UHD command path is NOT enabled: it must be validated on a
    // board before it can be claimed.  Failing closed here is deliberate.
    if (o.mode=="timed" && o.replay.empty())
        throw std::runtime_error("--mode timed requires --replay: the device-timed "
                                 "UHD command path is not validated on hardware yet");
    std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    const auto taps=load<float>(std::string(UWB_TESTDATA_DIR)+"/resampler_65_48/taps_quality_minorder.txt");
    const auto tmpl=load<std::complex<float>>(std::string(UWB_TESTDATA_DIR)+"/reference_preamble_code10_998p4.cf32");

    // ---- PHY-derived target window geometry ------------------------------
    // 127 B is the maximum PSDU, so the capture capacity never depends on the
    // PHR of the packet being decoded.  The 998.4 MS/s work-grid span is mapped
    // to native samples with the same 48/65 ratio the resampler uses.
    demod::Qm35825Profile geom;
    geom.code_index = 10; geom.preamble_repetitions = 256; geom.sfd_mode = "4z2";
    geom.max_psdu_bytes = 127;
    const size_t work_span = mod::packet_samples_998p4(
        geom.preamble_repetitions, 8, geom.max_psdu_bytes);
    const size_t native_span =
        size_t(std::llround(double(work_span) * 48.0 / 65.0)) + 1024;
    const size_t pre_guard = pre;
    const size_t post_guard = o.post_guard;
    if (pre_guard + native_span + post_guard > window)
        throw std::runtime_error("target window geometry exceeds the job buffer");

    core::TargetLockConfig lock_cfg;
    lock_cfg.sample_rate = fs;
    lock_cfg.nominal_period_s = o.period_ms * 1e-3;
    lock_cfg.verify_observations = o.lock_observations;
    lock_cfg.holdover_miss_count = o.holdover;
    lock_cfg.reacquire_miss_count = o.reacquire;
    lock_cfg.code_index = 10;
    lock_cfg.preamble_repetitions = 256;
    lock_cfg.sfd_mode = "4z2";
    lock_cfg.geometry = core::target_window_geometry(
        /*sync_span=*/size_t(std::llround(256.0 * 1016.0 * 48.0 / 65.0)),
        /*sfd_span=*/size_t(std::llround(8.0 * 1016.0 * 48.0 / 65.0)),
        /*phr_span=*/size_t(std::llround(21.0 * 512.0 * 2.0 * 48.0 / 65.0)),
        /*payload_span=*/size_t(std::llround(1208.0 * 64.0 * 2.0 * 48.0 / 65.0)),
        /*pulse_halo=*/1024, pre_guard, post_guard);
    core::TargetLockCore lock(lock_cfg);
    core::TargetLockState last_state = lock.state();

    Reporter report(o.summary);
    report.info("START mode="+o.mode+" sync="+o.sync+
        " channel="+o.channel+" antenna="+o.antenna+" cir="+o.cir+
        " workers="+std::to_string(o.workers)+" summary="+std::to_string(o.summary)+
        " period_ms="+std::to_string(o.period_ms)+
        " pre_guard="+std::to_string(pre_guard)+
        " capture="+std::to_string(lock_cfg.geometry.capture)+
        " post_guard="+std::to_string(post_guard)+
        " window_capacity="+std::to_string(lock_cfg.geometry.capacity())+
        " interference="+std::string(o.interference?"on":"off")+
        " fused="+std::string(o.fused?"on":"off")+
        " ondemand="+std::string(o.ondemand_resolved?"on":"off"));
    Workers workers(report,lock,o,taps,tmpl); // All worker allocations before streaming.

    const auto emit_lock = [&](const char* why) {
        const auto st = lock.stats();
        std::ostringstream s;
        s << std::fixed << std::setprecision(3) << "LOCK state="
          << core::target_lock_state_name(lock.state())
          << " why=" << why << " gen=" << lock.generation()
          << " epoch=" << lock.epoch()
          << " period_samples=" << lock.period_samples()
          << " period_ppm=" << st.period_ppm
          << " t0=" << lock.t0_exact()
          << " last_valid=" << st.last_valid_origin
          << " lock_slots=" << st.lock_slots
          << " holdover_len=" << st.holdover_length;
        report.info(s.str());
        last_state = lock.state();
    };

    pipeline::PipelineConfig cfg;
    cfg.uhd_args=o.args; cfg.uhd_channel=o.channel; cfg.uhd_antenna=o.antenna;
    cfg.uhd_subdev=o.subdev; cfg.native_rate=fs; cfg.uhd_center_freq=o.freq; cfg.uhd_gain=o.gain;
    std::unique_ptr<pipeline::ISc16StreamSource> inner;
    TimedReplaySc16Source* timed = nullptr;
    if (!o.replay.empty()) {
        if (o.mode == "timed") {
            core::TimedRxPlanConfig tcfg;
            tcfg.sample_rate = fs;
            tcfg.alignment = o.timed_alignment;
            tcfg.max_lookahead = o.timed_lookahead;
            tcfg.command_lead_s = o.timed_lead_us * 1e-6;
            auto ts = std::make_unique<TimedReplaySc16Source>(o.replay, fs, tcfg);
            timed = ts.get();
            inner = std::move(ts);
        } else {
            inner=std::make_unique<pipeline::ReplaySc16Source>(o.replay,fs,true);
        }
    } else {
        inner=std::make_unique<pipeline::UhdDeviceSc16Source>(cfg);
    }
    // Fixed-tone cancellation is a CONTINUOUS-stream module: it trains on the
    // first 0.5 ms of every 20 ms period.  Under timed_burst the stream has
    // planned gaps, so the training window can land on the packet itself and
    // corrupt it.  The device-timed tone schedule exists as a core helper
    // (core::merge_physical_rx_intervals) with QA, but it is NOT wired into the
    // live source yet, so timed mode runs with cancellation off rather than
    // silently subtracting a packet-derived model.
    core::FixedToneCancelConfig tone;
    tone.enable = o.tone && (o.mode != "timed");
    tone.period_s = o.tone_period_ms * 1e-3;
    tone.training_s = o.tone_training_ms * 1e-3;
    if (o.mode == "timed")
        report.info("TONE_CANCEL disabled reason=timed_burst_training_unscheduled");
    else if (!o.tone)
        report.info("TONE_CANCEL disabled reason=operator_off");
    // In continuous mode the tone decorator sits between the device and the
    // app.  In timed mode it must NOT: the decorator's reader thread would
    // spin on the empty space between planned windows and the scheduler (this
    // thread) could never queue the next window.  Tone cancellation is off in
    // timed mode anyway, so the source is used directly.
    std::unique_ptr<pipeline::ISc16StreamSource> source_owner;
    pipeline::FixedToneCancelSc16Source* decorator = nullptr;
    pipeline::ISc16StreamSource* src = nullptr;
    if (!tone.enable) {
        // No decorator at all when cancellation is off: wrapping a source that
        // is never cancelled only adds a bounded ring that can overflow.
        source_owner = std::move(inner);
        src = source_owner.get();
    } else {
        auto dec = std::make_unique<pipeline::FixedToneCancelSc16Source>(
            std::move(inner), tone, block, o.raw_blocks);
        decorator = dec.get();
        source_owner = std::move(dec);
        src = decorator;
    }
    const auto source_stats = [&]() -> pipeline::FixedToneCancelStats {
        if (decorator)
            return decorator->stats();
        pipeline::FixedToneCancelStats z{};
        z.source_samples = uint64_t(timed ? timed->delivered_samples() : 0);
        return z;
    };
    // Touch ring pages before UHD starts, rather than faulting them in recv.
    std::vector<std::complex<int16_t>> ring(history_size);
    std::vector<std::complex<int16_t>> incoming(block);
    Acquisition acq;
    if (!src->open()) throw std::runtime_error("cannot open UHD/replay or tone configuration");
    if (std::abs(src->actual_rx_rate()-fs)>.5) throw std::runtime_error("unexpected RX rate");
    if (!src->start_stream()) throw std::runtime_error("cannot start RX stream");
    report.info("STREAM_STARTED native_rate=737280000 tone_hz=-232960000 period_ms=20 training_ms=0.5");
    const auto started=Clock::now(); auto snapshot=started;
    int64_t end=0, valid_begin=0; uint64_t id=0, gaps=0;
    int64_t pending_seed=-1; uint64_t pending_id=0;
    std::deque<PendingSubmit> pending;
    core::TargetSlotPlan pending_plan;
    bool have_pending_plan = false;
    uint64_t pending_slot = 0;
    struct AssocPending {
        uint64_t slot;
        int64_t start;
        double metric;
    };
    std::vector<AssocPending> assoc;
    uint64_t dedup_confirmed = 0, dedup_ambiguous = 0;
    bool failed=false;
    double frontend_us=0; uint64_t frontend_samples=0;
    uint64_t target_slots=0, target_dropped=0, target_expired=0, dedup=0, unknown=0;
    struct TimedPending {
        uint64_t slot_index;
        uint64_t generation;
        int64_t window_start;
        size_t count;
        int64_t predicted_origin;
    };
    std::deque<TimedPending> timed_pending;
    int64_t timed_cursor = std::llround(o.timed_t0_ms * 1e-3 * fs);
    const int64_t nominal_period_samples = std::llround(o.period_ms * 1e-3 * fs);

    // Queue a bounded lookahead of device-timed windows.  Before the lock the
    // grid is the nominal period; after the lock it is the predicted one.  A
    // window already queued is never rewritten.  This must be callable BEFORE
    // the first read: the read blocks until a window exists.
    const auto queue_timed_windows = [&]() {
        if (!timed)
            return;
        while (timed->plan_stats().requested <
                   timed->plan_stats().issued + o.timed_lookahead &&
               timed_cursor < end + int64_t(4 * nominal_period_samples)) {
            core::TargetSlotPlan plan;
            if (lock.next_plan(timed_cursor, plan)) {
                const size_t count = size_t(plan.window_end - plan.window_start);
                if (timed->queue_window(plan.window_start, count,
                                        plan.generation)) {
                    TimedPending tp;
                    tp.slot_index = plan.slot_index;
                    tp.generation = plan.generation;
                    tp.window_start = plan.window_start;
                    tp.count = count;
                    tp.predicted_origin = plan.predicted_origin;
                    timed_pending.push_back(tp);
                    lock.commit_plan(plan);
                    ++target_slots;
                } else {
                    lock.note_slot_outcome(plan.slot_index,
                                           core::TargetSlotOutcome::DroppedQueue);
                    ++target_dropped;
                }
                timed_cursor = plan.window_end;
            } else {
                // Blind grid: the packet is expected at `timed_cursor`, so the
                // window starts one pre_guard earlier.
                const int64_t ws = timed_cursor - int64_t(pre_guard);
                if (timed->queue_window(ws < 0 ? 0 : ws, window,
                                        lock.generation())) {
                    TimedPending tp;
                    tp.slot_index = std::numeric_limits<uint64_t>::max();
                    tp.generation = lock.generation();
                    tp.window_start = ws < 0 ? 0 : ws;
                    tp.count = window;
                    tp.predicted_origin = ws < 0 ? int64_t(pre_guard)
                                                 : timed_cursor;
                    timed_pending.push_back(tp);
                }
                timed_cursor += nominal_period_samples;
            }
        }
    };
    queue_timed_windows();

    while (!interrupted && (o.seconds==0 || std::chrono::duration<double>(Clock::now()-started).count()<o.seconds)) {
        pipeline::StreamReadMeta meta;
        // Decorator may change the absolute origin at a gap.  Receive to a small
        // staging block so the ring's absolute mapping always stays correct.
        const auto n=src->read_samples(incoming.data(),block,meta);
        if (meta.is_gap || (n && meta.start_sample_abs!=end)) {
            // A planned timed gap is expected: it is the monitoring blind
            // time, not a device error, and it must not invalidate the grid.
            const bool planned = (o.mode == "timed");
            if (!planned) ++gaps;
            report.info(std::string(planned ? "RX_PLANNED_GAP" : "RX_GAP")+
                        " sample="+std::to_string(meta.start_sample_abs)+
                        " error_code="+std::to_string(meta.error_code)+
                        " gap_samples="+std::to_string(meta.gap_samples));
            if (pending_seed>=0) report.result(pending_id,pending_seed,"INVALID_GAP");
            pending_seed=-1;
            for (const auto& ps : pending)
                report.result(ps.id, ps.seed, "INVALID_GAP");
            pending.clear();
            have_pending_plan = false;
            end=meta.start_sample_abs; valid_begin=end; acq.reset(end);
            // An unexpected gap invalidates the grid; a planned timed gap is
            // expected and must not.
            if (o.mode != "timed") lock.note_time_discontinuity();
        }
        if (n) {
            const auto frontend_start = Clock::now();
            const size_t at=size_t(end)&(history_size-1), first=std::min(n,history_size-at);
            std::copy_n(incoming.data(),first,ring.data()+at);
            std::copy_n(incoming.data()+first,n-first,ring.data());
            end+=n; valid_begin=std::max(valid_begin,end-int64_t(history_size));
            if (o.mode != "timed") {
                while (acq.probe+Acquisition::lag<end) {
                    double metric = 0.0;
                    const auto seed=acq.scan(ring,&metric);
                    if (seed<0) continue;
                    ++id;
                    if (lock.scheduled()) {
                        // Classification is deferred to submission time: the
                        // predicted target window for this slot is only
                        // committed ~0.46 ms after the packet start, i.e. after
                        // this 6 ms-window autocorrelation detection.
                        if (!o.interference) {
                            ++unknown;
                        } else if (seed-int64_t(pre_guard)<valid_begin) {
                            report.result(id,seed,"INCOMPLETE_WINDOW");
                        } else {
                            PendingSubmit ps;
                            ps.id = id;
                            ps.seed = seed;
                            ps.window_start = seed-int64_t(pre_guard);
                            ps.count = window;
                            ps.predicted_rel_work = std::llround(pre_guard*65.0/48);
                            ps.generation = lock.generation();
                            ps.metric = metric;
                            ps.is_candidate = true;
                            if (pending.size() >= 8) {
                                report.result(id,seed,"DROPPED_QUEUE");
                            } else {
                                pending.push_back(ps);
                                ++unknown;
                            }
                        }
                    } else if (pending_seed>=0) {
                        report.result(id,seed,"OVERLAPPING_CANDIDATE");
                    } else if (seed-int64_t(pre_guard)<valid_begin) {
                        report.result(id,seed,"INCOMPLETE_WINDOW");
                    } else {
                        pending_seed=seed; pending_id=id;
                    }
                }
                if (pending_seed>=0 && end>=pending_seed-int64_t(pre_guard)+int64_t(window)) {
                    if (pending_seed-int64_t(pre_guard)<valid_begin) {
                        report.result(pending_id,pending_seed,"HISTORY_OVERRUN");
                    } else {
                        const int64_t ws = pending_seed-int64_t(pre_guard);
                        workers.submit(pending_id, ws, window, pending_seed,
                                       std::llround(pre_guard*65.0/48),
                                       std::numeric_limits<uint64_t>::max(),
                                       lock.generation(), false, true, ring);
                    }
                    pending_seed=-1;
                }
            }
            frontend_us += elapsed_us(frontend_start, Clock::now());
            frontend_samples += n;
        }
        // ---- deferred candidate submissions -------------------------------
        // A candidate window is cut `pre_guard` before its seed, so it only
        // becomes available once enough samples have arrived.  Waiting here
        // (rather than dropping it) is what keeps every autocorrelation
        // candidate accounted for.
        if (!pending.empty()) {
            std::deque<PendingSubmit> still;
            for (const auto& ps : pending) {
                if (ps.window_start < valid_begin) {
                    report.result(ps.id, ps.seed, "HISTORY_OVERRUN");
                    continue;
                }
                if (ps.window_start + int64_t(ps.count) > end) {
                    still.push_back(ps);
                    continue;
                }
                if (ps.is_candidate) {
                    // Now that the target plan for this slot exists, decide
                    // whether this autocorrelation response is the target's
                    // own.  Association is confirmed from the slot result.
                    core::InterferenceCandidate c;
                    c.start_sample = ps.seed;
                    c.metric = ps.metric;
                    const auto v = lock.classify_candidate(c);
                    if (v.cls == core::CandidateClass::TargetAssociated) {
                        ++dedup;
                        if (assoc.size() < 256)
                            assoc.push_back({ v.slot_index, ps.seed, ps.metric });
                        continue;
                    }
                }
                if (!workers.submit(ps.id, ps.window_start, ps.count, ps.seed,
                                    ps.predicted_rel_work, ps.slot_index,
                                    ps.generation, ps.target_slot, ps.feeds_lock,
                                    ring))
                    report.result(ps.id, ps.seed, "DROPPED_QUEUE");
            }
            pending.swap(still);
        }
        // ---- target path (continuous) ------------------------------------
        // The chosen slot is remembered across iterations, but its window is
        // RE-COMPUTED from the current model every time: a plan that has not
        // been handed to the workers yet must follow a period/phase update.
        // (Caching the window itself cut the first post-lock slots with the
        // nominal 5 ms period and missed them by ~300 samples.)
        if (o.mode != "timed") {
            if (have_pending_plan &&
                pending_plan.generation != lock.generation()) {
                // The grid changed under us; the old plan is void.
                have_pending_plan = false;
            }
            if (!have_pending_plan) {
                have_pending_plan = lock.next_plan(end, pending_plan);
                if (have_pending_plan)
                    pending_slot = pending_plan.slot_index;
            }
            while (have_pending_plan) {
                core::TargetSlotPlan plan = lock.plan_for_slot(pending_slot);
                if (plan.window_end > end)
                    break; // still filling
                have_pending_plan = false;
                const size_t count = size_t(plan.window_end - plan.window_start);
                if (plan.window_start < valid_begin) {
                    lock.abandon_slot(plan.slot_index);
                    ++target_expired;
                } else {
                    ++id;
                    const int64_t rel = plan.predicted_origin - plan.window_start;
                    const bool ok = workers.submit(id, plan.window_start, count,
                                                   plan.predicted_origin,
                                                   std::llround(rel*65.0/48),
                                                   plan.slot_index, plan.generation,
                                                   true, true, ring);
                    if (!ok) {
                        lock.note_slot_outcome(plan.slot_index,
                                               core::TargetSlotOutcome::DroppedQueue);
                        ++target_dropped;
                    } else {
                        lock.commit_plan(plan);
                        ++target_slots;
                    }
                }
                have_pending_plan = lock.next_plan(end, pending_plan);
                if (have_pending_plan)
                    pending_slot = pending_plan.slot_index;
            }
        }
        // ---- target path (timed) -----------------------------------------
        if (o.mode == "timed" && timed) {
            // Submit every planned window whose samples are now in the ring.
            while (!timed_pending.empty()) {
                const TimedPending tp = timed_pending.front();
                if (tp.window_start + int64_t(tp.count) > end)
                    break;
                timed_pending.pop_front();
                if (tp.window_start < valid_begin) {
                    report.result(++id, tp.predicted_origin, "HISTORY_OVERRUN");
                    if (tp.slot_index != std::numeric_limits<uint64_t>::max())
                        lock.abandon_slot(tp.slot_index);
                    continue;
                }
                const int64_t rel = tp.predicted_origin - tp.window_start;
                const bool target_slot =
                    tp.slot_index != std::numeric_limits<uint64_t>::max();
                if (!workers.submit(++id, tp.window_start, tp.count,
                                    tp.predicted_origin,
                                    std::llround(rel*65.0/48), tp.slot_index,
                                    tp.generation, target_slot, true, ring)) {
                    report.result(id, tp.predicted_origin, "DROPPED_QUEUE");
                    if (target_slot)
                        lock.note_slot_outcome(tp.slot_index,
                                               core::TargetSlotOutcome::DroppedQueue);
                }
            }
            queue_timed_windows();
        }
        // ---- lock feedback ------------------------------------------------
        lock.drain_observations();
        // Resolve deferred autocorrelation associations now that the target
        // slot results are in.
        if (!assoc.empty()) {
            std::vector<AssocPending> still;
            for (const auto& a : assoc) {
                if (!lock.slot_terminal(a.slot)) {
                    still.push_back(a);
                    continue;
                }
                if (lock.slot_succeeded(a.slot)) {
                    ++dedup_confirmed;
                } else {
                    ++dedup_ambiguous;
                    std::ostringstream s;
                    s << std::fixed << std::setprecision(3)
                      << "INTERFERENCE_AMBIGUOUS slot=" << a.slot
                      << " start=" << a.start << " metric=" << a.metric
                      << " reason=target_slot_failed";
                    report.info(s.str());
                }
            }
            assoc.swap(still);
        }
        if (lock.state() != last_state) {
            emit_lock(lock.state() == core::TargetLockState::Locked ? "converged"
                     : lock.state() == core::TargetLockState::Holdover ? "target_miss"
                     : lock.state() == core::TargetLockState::Reacquire ? "grid_invalid"
                     : "seeded");
        }
        if (Clock::now()-snapshot>std::chrono::milliseconds(100)) {
            report.snapshot(source_stats(),gaps,workers.backlog()); snapshot=Clock::now();
        }
        std::string error;
        if (src->fatal_error(&error)) {report.info("SOURCE_ERROR "+error);failed=true;break;}
        error = workers.failure();
        if (!error.empty()) {report.info("WORKER_ERROR "+error);failed=true;break;}
        if (meta.is_eof || (!n && src->is_eof())) break;
    }
    src->close(); // Stop hardware before draining workers / printing.
    report.snapshot(source_stats(),gaps,workers.backlog());
    if (pending_seed>=0) report.result(pending_id,pending_seed,"INCOMPLETE_WINDOW");
    workers.stop();
    // Workers stop only after every queued job finished, so the last batch of
    // observations must still be applied before the final statistics.
    lock.drain_observations();
    report.snapshot(source_stats(),gaps,workers.backlog());
    const auto st = source_stats();
    std::ostringstream costs;
    costs << std::fixed << std::setprecision(3) << "COST_TOTAL frontend_us=" << frontend_us
          << " frontend_samples=" << frontend_samples << " estimate_us=" << st.estimate_us
          << " cancel_us=" << st.cancel_us << " cancelled_samples=" << st.cancelled_samples
          << " bypassed_samples=" << st.bypassed_samples
          << " model_updates=" << st.model_updates
          << " model_rejects=" << st.model_rejects
          << " training_windows=" << st.training_windows
          << " reader_us=" << st.reader_us;
    report.info(costs.str());
    {
        const auto ls = lock.stats();
        std::ostringstream s;
        s << "LOCK_STATS final_state=" << core::target_lock_state_name(lock.state())
          << " expected_slots=" << ls.expected_slots
          << " submitted=" << ls.submitted
          << " dropped_queue=" << ls.dropped_queue
          << " expired=" << ls.expired
          << " not_detected=" << ls.not_detected
          << " decode_failed=" << ls.decode_failed
          << " fcs_ok=" << ls.fcs_ok
          << " fcs_failed=" << ls.fcs_failed
          << " observations=" << ls.observations
          << " stale=" << ls.stale_observations
          << " duplicate=" << ls.duplicate_observations
          << " identity_rejects=" << ls.identity_rejects
          << " out_of_grid=" << ls.out_of_grid
          << " queue_full=" << ls.queue_full
          << " residual_outliers=" << ls.residual_outliers
          << " lock_entries=" << ls.lock_entries
          << " holdover_entries=" << ls.holdover_entries
          << " reacquire_entries=" << ls.reacquire_entries
          << " discontinuities=" << ls.discontinuity_events
          << " candidates=" << ls.candidates
          << " cand_target=" << ls.candidates_target
          << " cand_interference=" << ls.candidates_interference
          << " cand_unknown=" << ls.candidates_unknown
          << " period_updates=" << ls.period_updates
          << " lock_slots=" << ls.lock_slots
          << " period_samples=" << ls.period_samples
          << " period_ppm=" << ls.period_ppm
          << " scheduler_target_slots=" << target_slots
          << " scheduler_target_dropped=" << target_dropped
          << " scheduler_target_expired=" << target_expired
          << " autocorr_dedup=" << dedup
          << " autocorr_dedup_confirmed=" << dedup_confirmed
          << " autocorr_dedup_ambiguous=" << dedup_ambiguous
          << " autocorr_unknown=" << unknown
          << " interference_events=" << workers.interference_events.load();
        report.info(s.str());
    }
    if (timed) {
        const auto& ts = timed->plan_stats();
        std::ostringstream s;
        s << "TIMED_PLAN requested=" << ts.requested
          << " issued=" << ts.issued
          << " completed=" << ts.completed
          << " partial=" << ts.partial
          << " timeouts=" << ts.timeouts
          << " late_commands=" << ts.late_commands
          << " overflows=" << ts.overflows
          << " device_time_jumps=" << ts.device_time_jumps
          << " planned_gaps=" << ts.planned_gaps
          << " planned_gap_samples=" << ts.planned_gap_samples
          << " unexpected_gaps=" << ts.unexpected_gaps
          << " unexpected_gap_samples=" << ts.unexpected_gap_samples
          << " cancelled=" << ts.cancelled
          << " request_queue_full=" << ts.request_queue_full
          << " lookahead_blocked=" << ts.lookahead_blocked
          << " alignment_expanded_samples=" << ts.alignment_expanded_samples;
        report.info(s.str());
    }
    report.finish();
    return failed ? 2 : 0;
} catch (const std::exception& e) {
    std::cerr << "ERROR " << e.what() << std::endl; return 2;
}
