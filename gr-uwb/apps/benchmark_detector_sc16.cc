#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_detector_sc16.h>
#include <gnuradio/uwb/uwb_realtime_demodulator.h>

#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr size_t kPacketStart = 4992000;
constexpr size_t kSymbolLen = 1016;

// Benchmark-only sync source. One scheduler item is one complex sample for
// both formats; work() fills the granted output span with bulk memcpy and
// carries only the repeat offset across calls. No allocation occurs in work().
template <typename Sample>
class BulkRepeatSource : public gr::sync_block
{
public:
    using sptr = std::shared_ptr<BulkRepeatSource<Sample>>;

    static sptr make(const std::vector<Sample>& data)
    {
        return gnuradio::make_block_sptr<BulkRepeatSource<Sample>>(data);
    }

    explicit BulkRepeatSource(const std::vector<Sample>& data)
        : gr::sync_block("bulk_repeat_source",
                         gr::io_signature::make(0, 0, 0),
                         gr::io_signature::make(1, 1, sizeof(Sample))),
          data_(data)
    {
        if (data_.empty())
            throw std::invalid_argument("BulkRepeatSource data must not be empty");
    }

    int work(int noutput_items,
             gr_vector_const_void_star&,
             gr_vector_void_star& output_items) override
    {
        auto* out = static_cast<Sample*>(output_items[0]);
        size_t produced = 0;
        const size_t requested = static_cast<size_t>(noutput_items);
        while (produced < requested) {
            const size_t count =
                std::min(requested - produced, data_.size() - offset_);
            std::memcpy(out + produced, data_.data() + offset_,
                        count * sizeof(Sample));
            produced += count;
            offset_ += count;
            if (offset_ == data_.size())
                offset_ = 0;
        }
        return noutput_items;
    }

private:
    std::vector<Sample> data_;
    size_t offset_ = 0;
};

std::vector<gr_complex> load(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open " + path);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<gr_complex> x(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(x.data()), bytes);
    return x;
}

template <typename Detector, typename Source>
double run(const Source& src, const Detector& det, size_t item_size,
           uint64_t target, size_t buffer_items, size_t* messages)
{
    auto head = gr::blocks::head::make(item_size, target);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("detector_format_benchmark");
    tb->connect(src, 0, head, 0);
    tb->connect(head, 0, det, 0);
    tb->msg_connect(det, "packet", dbg, "store");
    if (buffer_items > 0) {
        src->set_max_output_buffer(0, -1);
        src->set_min_output_buffer(0, static_cast<long>(buffer_items));
        head->set_max_output_buffer(0, -1);
        head->set_min_output_buffer(0, static_cast<long>(buffer_items));
        src->set_max_noutput_items(static_cast<int>(buffer_items));
        head->set_max_noutput_items(static_cast<int>(buffer_items));
        det->set_max_noutput_items(static_cast<int>(buffer_items));
        tb->set_max_noutput_items(static_cast<int>(buffer_items));
    }
    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    const double sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    *messages = dbg->num_messages();
    return sec;
}

template <typename Source>
double run_source_only(const Source& src,
                       size_t item_size,
                       uint64_t target,
                       size_t buffer_items)
{
    auto head = gr::blocks::head::make(item_size, target);
    auto sink = gr::blocks::null_sink::make(item_size);
    auto tb = gr::make_top_block("format_source_benchmark");
    tb->connect(src, 0, head, 0);
    tb->connect(head, 0, sink, 0);
    if (buffer_items > 0) {
        src->set_max_output_buffer(0, -1);
        src->set_min_output_buffer(0, static_cast<long>(buffer_items));
        head->set_max_output_buffer(0, -1);
        head->set_min_output_buffer(0, static_cast<long>(buffer_items));
        src->set_max_noutput_items(static_cast<int>(buffer_items));
        head->set_max_noutput_items(static_cast<int>(buffer_items));
        tb->set_max_noutput_items(static_cast<int>(buffer_items));
    }
    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now() - t0)
        .count();
}

struct E2eResult {
    double seconds = 0.0;
    size_t packets = 0;
    uint64_t received = 0;
    uint64_t completed = 0;
    uint64_t failed = 0;
    uint64_t dropped = 0;
    size_t queue_high_watermark = 0;
    uint64_t latency_p95_us = 0;
    double worker_utilization = 0.0;
    std::string first_status;
    int64_t first_predicted = -1;
    int64_t first_detected = -1;
    uint64_t first_input_samples = 0;
};

template <typename Detector, typename Source>
E2eResult run_with_demod(const Source& src,
                         const Detector& det,
                         size_t item_size,
                         uint64_t target,
                         size_t buffer_items,
                         const std::vector<gr_complex>& demod_template,
                         size_t demod_workers,
                         size_t queue_capacity)
{
    auto head = gr::blocks::head::make(item_size, target);
    auto demod = gr::uwb::UwbRealtimeDemodulator::make_from_template(
        demod_template, demod_workers, queue_capacity, "ieee");
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("detector_demod_format_benchmark");
    tb->connect(src, 0, head, 0);
    tb->connect(head, 0, det, 0);
    tb->msg_connect(det, "packet", demod, "samples");
    tb->msg_connect(demod, "result", dbg, "store");
    if (buffer_items > 0) {
        src->set_max_output_buffer(0, -1);
        src->set_min_output_buffer(0, static_cast<long>(buffer_items));
        head->set_max_output_buffer(0, -1);
        head->set_min_output_buffer(0, static_cast<long>(buffer_items));
        src->set_max_noutput_items(static_cast<int>(buffer_items));
        head->set_max_noutput_items(static_cast<int>(buffer_items));
        det->set_max_noutput_items(static_cast<int>(buffer_items));
        tb->set_max_noutput_items(static_cast<int>(buffer_items));
    }
    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    E2eResult result;
    result.seconds = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - t0)
                         .count();
    result.packets = dbg->num_messages();
    result.received = demod->jobs_received();
    result.completed = demod->jobs_completed();
    result.failed = demod->jobs_failed();
    result.dropped = demod->jobs_dropped();
    result.queue_high_watermark = demod->queue_high_watermark();
    result.latency_p95_us = demod->latency_p95_us();
    result.worker_utilization = demod->worker_utilization_pct();
    if (dbg->num_messages() > 0) {
        const pmt::pmt_t meta = pmt::car(dbg->get_message(0));
        const pmt::pmt_t status =
            pmt::dict_ref(meta, pmt::mp("status"), pmt::PMT_NIL);
        if (pmt::is_symbol(status))
            result.first_status = pmt::symbol_to_string(status);
        result.first_predicted = pmt::to_long(pmt::dict_ref(
            meta, pmt::mp("predicted_start_sample"), pmt::from_long(-1)));
        result.first_detected = pmt::to_long(pmt::dict_ref(
            meta, pmt::mp("detected_start_sample"), pmt::from_long(-1)));
        result.first_input_samples = pmt::to_uint64(pmt::dict_ref(
            meta, pmt::mp("input_sample_count"), pmt::from_uint64(0)));
    }
    return result;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: benchmark_detector_sc16 CFILE [target_samples] "
                     "[buffer_items] [cf32-first|sc16-first] [sc16_coarse_D] "
                     "[demod_workers] [e2e_packet_rate]\n";
        return 2;
    }
    const uint64_t target = argc > 2 ? std::stoull(argv[2]) : 100000000ULL;
    const size_t buffer_items =
        argc > 3 ? static_cast<size_t>(std::stoull(argv[3])) : 1048576;
    const bool sc16_first = argc > 4 && std::string(argv[4]) == "sc16-first";
    const size_t sc16_coarse_D = argc > 5 ? std::stoull(argv[5]) : 4;
    const size_t demod_workers = argc > 6 ? std::stoull(argv[6]) : 4;
    const double e2e_packet_rate = argc > 7 ? std::stod(argv[7]) : 0.0;
    auto x = load(argv[1]);
    if (x.size() <= kPacketStart + kSymbolLen)
        throw std::runtime_error("reference cfile is too short");
    std::vector<gr_complex> tmpl(x.begin() + kPacketStart,
                                 x.begin() + kPacketStart + kSymbolLen);
    gr::uwb::core::uwb_l2_normalize(tmpl);

    std::vector<std::complex<int16_t>> sc16(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        const float re = std::max(-32768.0f, std::min(32767.0f,
            std::round(x[i].real() * 32768.0f)));
        const float im = std::max(-32768.0f, std::min(32767.0f,
            std::round(x[i].imag() * 32768.0f)));
        sc16[i] = { static_cast<int16_t>(re), static_cast<int16_t>(im) };
    }

    const double source_sec_f = run_source_only(
        BulkRepeatSource<gr_complex>::make(x), sizeof(gr_complex), target,
        buffer_items);
    const double source_sec_s = run_source_only(
        BulkRepeatSource<std::complex<int16_t>>::make(sc16),
        sizeof(std::complex<int16_t>), target, buffer_items);

    size_t n_f = 0;
    size_t n_s = 0;
    auto src_f = BulkRepeatSource<gr_complex>::make(x);
    auto det_f = gr::uwb::UwbDetector::make(tmpl);
    auto src_s = BulkRepeatSource<std::complex<int16_t>>::make(sc16);
    auto det_s = gr::uwb::UwbDetectorSc16::make(
        tmpl, 2032, 200000, 1e-3f, 100, sc16_coarse_D);
    double sec_f = 0.0;
    double sec_s = 0.0;
    if (sc16_first) {
        sec_s = run(src_s, det_s, sizeof(std::complex<int16_t>), target,
                    buffer_items, &n_s);
        sec_f = run(src_f, det_f, sizeof(gr_complex), target,
                    buffer_items, &n_f);
    } else {
        sec_f = run(src_f, det_f, sizeof(gr_complex), target,
                    buffer_items, &n_f);
        sec_s = run(src_s, det_s, sizeof(std::complex<int16_t>), target,
                    buffer_items, &n_s);
    }
    const double rate_f = target / sec_f / 1e6;
    const double rate_s = target / sec_s / 1e6;

    constexpr size_t demod_pre_trigger = 9984;
    constexpr size_t demod_capture = 309184;
    const auto golden_window = load("testdata/realtime_demod_golden/window.cfile");
    auto demod_tmpl = load("testdata/reference_preamble.bin");
    const size_t e2e_cycle_samples = e2e_packet_rate > 0.0
        ? static_cast<size_t>(std::llround(
              gr::uwb::core::kUwbSampleRateHz / e2e_packet_rate))
        : x.size();
    if (e2e_cycle_samples == 0 || golden_window.size() > e2e_cycle_samples)
        throw std::runtime_error("demod golden window exceeds benchmark cycle");
    const double actual_e2e_packet_rate =
        gr::uwb::core::kUwbSampleRateHz / e2e_cycle_samples;
    std::vector<gr_complex> e2e_x(e2e_cycle_samples, gr_complex(0.0f, 0.0f));
    std::copy(golden_window.begin(), golden_window.end(), e2e_x.begin());
    std::vector<std::complex<int16_t>> e2e_sc16(e2e_x.size());
    for (size_t i = 0; i < e2e_x.size(); ++i) {
        const auto quantize = [](float value) {
            return static_cast<int16_t>(std::max(
                -32768.0f, std::min(32767.0f, std::round(value * 32768.0f))));
        };
        e2e_sc16[i] = { quantize(e2e_x[i].real()), quantize(e2e_x[i].imag()) };
    }

    auto e2e_src_f = BulkRepeatSource<gr_complex>::make(e2e_x);
    auto e2e_det_f = gr::uwb::UwbDetector::make(
        demod_tmpl, demod_pre_trigger, demod_capture);
    auto e2e_src_s = BulkRepeatSource<std::complex<int16_t>>::make(e2e_sc16);
    auto e2e_det_s = gr::uwb::UwbDetectorSc16::make(
        demod_tmpl, demod_pre_trigger, demod_capture, 1e-3f, 100,
        sc16_coarse_D);
    constexpr size_t queue_capacity = 256;
    E2eResult e2e_f;
    E2eResult e2e_s;
    if (sc16_first) {
        e2e_s = run_with_demod(e2e_src_s, e2e_det_s,
                               sizeof(std::complex<int16_t>), target,
                               buffer_items, demod_tmpl, demod_workers,
                               queue_capacity);
        e2e_f = run_with_demod(e2e_src_f, e2e_det_f, sizeof(gr_complex), target,
                               buffer_items, demod_tmpl, demod_workers,
                               queue_capacity);
    } else {
        e2e_f = run_with_demod(e2e_src_f, e2e_det_f, sizeof(gr_complex), target,
                               buffer_items, demod_tmpl, demod_workers,
                               queue_capacity);
        e2e_s = run_with_demod(e2e_src_s, e2e_det_s,
                               sizeof(std::complex<int16_t>), target,
                               buffer_items, demod_tmpl, demod_workers,
                               queue_capacity);
    }
    std::cout << "target_samples=" << target
              << " buffer_items=" << buffer_items
              << " order=" << (sc16_first ? "sc16-first" : "cf32-first")
              << " sc16_coarse_D=" << sc16_coarse_D
              << "\n"
              << "cf32_source_MSps=" << target / source_sec_f / 1e6
              << " sc16_source_MSps=" << target / source_sec_s / 1e6 << "\n"
              << "cf32_seconds=" << sec_f << " cf32_MSps=" << rate_f
              << " packets=" << n_f
              << " chunk_min_mean_max=" << det_f->work_min_noutput_items()
              << "/" << det_f->work_mean_noutput_items()
              << "/" << det_f->work_max_noutput_items() << "\n"
              << "sc16_seconds=" << sec_s << " sc16_MSps=" << rate_s
              << " packets=" << n_s
              << " chunk_min_mean_max=" << det_s->work_min_noutput_items()
              << "/" << det_s->work_mean_noutput_items()
              << "/" << det_s->work_max_noutput_items() << "\n"
              << "speedup_sc16_over_cf32=" << rate_s / rate_f << "\n"
              << "input_bytes_ratio=0.5\n";
    const auto print_e2e = [target](const char* format, const E2eResult& result) {
        std::cout << format << "_e2e_seconds=" << result.seconds
                  << " input_MSps=" << target / result.seconds / 1e6
                  << " processing_packets_per_second="
                  << result.received / result.seconds
                  << " result_packets=" << result.packets
                  << " received=" << result.received
                  << " completed=" << result.completed
                  << " failed=" << result.failed
                  << " dropped=" << result.dropped
                  << " queue_hwm=" << result.queue_high_watermark
                  << " latency_p95_us=" << result.latency_p95_us
                  << " worker_util_pct=" << result.worker_utilization << "\n";
        std::cout << format << "_first_status=" << result.first_status
                  << " predicted=" << result.first_predicted
                  << " detected=" << result.first_detected
                  << " input_samples=" << result.first_input_samples << "\n";
    };
    std::cout << "demod_workers=" << demod_workers
              << " demod_queue_capacity=" << queue_capacity
              << " e2e_cycle_samples=" << e2e_cycle_samples
              << " realtime_packets_per_second=" << actual_e2e_packet_rate
              << "\n";
    print_e2e("cf32", e2e_f);
    print_e2e("sc16", e2e_s);
    const bool detector_ok = n_f == n_s && n_f > 0;
    const bool demod_accounted =
        e2e_f.completed + e2e_f.failed + e2e_f.dropped == e2e_f.received &&
        e2e_s.completed + e2e_s.failed + e2e_s.dropped == e2e_s.received;
    return detector_ok && demod_accounted ? 0 : 3;
}
