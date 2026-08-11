/* -*- c++ -*- */
/*
 * Resampler benchmark: Level A (built-in) + Level B (fixed 65/48).
 *
 * Modes (argv[1]):
 *   builtin | a     — GNU Radio rational_resampler_ccf (Level A baseline)
 *   kernel          — RationalResampler65_48Core direct (no scheduler)
 *   block   | b     — UwbRationalResamplerCcf65_48 flowgraph
 *   profile         — Stage-1 core micro-attribution (assemble/schedule/FIR/state)
 *   all             — builtin + kernel + block
 *
 * Usage:
 *   benchmark_resampler_65_48 [mode] [target] [buffer] [kernel] [workers]
 *   kernel names: default|avx2|volk|volk_legacy|volk_macroblock|scalar|scalar_legacy
 *   workers: 1..N (block/kernel multi-worker FIR; default 1)
 *
 * Spec: 开发方案_GNURadio软件升采样_737.28M到998.4M.md §9
 *
 * No Throttle. No file I/O on the timed path. No allocation in work().
 */

#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/filter/rational_resampler.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <time.h>
#include <vector>

namespace {

constexpr unsigned kInterp = 65;
constexpr unsigned kDecim = 48;
constexpr double kInRateHz = 737.28e6;
constexpr double kOutRateHz = 998.4e6;
// Payload bandwidth estimates at the contract rates (CF32 = 8 B/sample).
constexpr double kInPayloadGBps = 5.89824;  // 737.28e6 * 8 / 1e9
constexpr double kOutPayloadGBps = 7.98720; // 998.4e6  * 8 / 1e9
constexpr uint64_t kDefaultTarget = 2000000000ULL; // ~2.7 s @ 737.28 MS/s
constexpr size_t kDefaultBuffer = 1048576;
constexpr int kTimedRounds = 7;
constexpr double kSustainedSeconds = 30.0;
// Ring size for BulkRepeatSource: large enough to leave L3, small enough to fit.
constexpr size_t kRingSamples = 1 << 20; // 1 Mi samples = 8 MiB CF32

// ---------------------------------------------------------------------------
// Benchmark-only sync source. work() only memcpy; no allocation.
// ---------------------------------------------------------------------------
class BulkRepeatSource : public gr::sync_block
{
public:
    using sptr = std::shared_ptr<BulkRepeatSource>;

    static sptr make(const std::vector<gr_complex>& data)
    {
        return gnuradio::make_block_sptr<BulkRepeatSource>(data);
    }

    explicit BulkRepeatSource(const std::vector<gr_complex>& data)
        : gr::sync_block("bulk_repeat_source",
                         gr::io_signature::make(0, 0, 0),
                         gr::io_signature::make(1, 1, sizeof(gr_complex))),
          data_(data)
    {
        if (data_.empty())
            throw std::invalid_argument("BulkRepeatSource data must not be empty");
    }

    int work(int noutput_items,
             gr_vector_const_void_star&,
             gr_vector_void_star& output_items) override
    {
        auto* out = static_cast<gr_complex*>(output_items[0]);
        size_t produced = 0;
        const size_t requested = static_cast<size_t>(noutput_items);
        while (produced < requested) {
            const size_t count =
                std::min(requested - produced, data_.size() - offset_);
            std::memcpy(out + produced, data_.data() + offset_,
                        count * sizeof(gr_complex));
            produced += count;
            offset_ += count;
            if (offset_ == data_.size())
                offset_ = 0;
        }
        return noutput_items;
    }

private:
    std::vector<gr_complex> data_;
    size_t offset_ = 0;
};

// ---------------------------------------------------------------------------
// Sink that FNV-1a hashes every sample so the optimizer cannot drop the FIR.
// No allocation in work().
// ---------------------------------------------------------------------------
class ChecksumSink : public gr::sync_block
{
public:
    using sptr = std::shared_ptr<ChecksumSink>;

    static sptr make()
    {
        return gnuradio::make_block_sptr<ChecksumSink>();
    }

    ChecksumSink()
        : gr::sync_block("checksum_sink",
                         gr::io_signature::make(1, 1, sizeof(gr_complex)),
                         gr::io_signature::make(0, 0, 0)),
          checksum_(kFnvOffset),
          count_(0),
          sum_re_(0.0),
          sum_im_(0.0)
    {
    }

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star&) override
    {
        const auto* in = static_cast<const gr_complex*>(input_items[0]);
        uint64_t h = checksum_;
        double sr = sum_re_;
        double si = sum_im_;
        for (int i = 0; i < noutput_items; ++i) {
            uint32_t re_bits = 0;
            uint32_t im_bits = 0;
            const float re = in[i].real();
            const float im = in[i].imag();
            std::memcpy(&re_bits, &re, sizeof(re_bits));
            std::memcpy(&im_bits, &im, sizeof(im_bits));
            h ^= static_cast<uint64_t>(re_bits);
            h *= kFnvPrime;
            h ^= static_cast<uint64_t>(im_bits);
            h *= kFnvPrime;
            sr += static_cast<double>(re);
            si += static_cast<double>(im);
        }
        checksum_ = h;
        sum_re_ = sr;
        sum_im_ = si;
        count_ += static_cast<uint64_t>(noutput_items);
        return noutput_items;
    }

    uint64_t checksum() const { return checksum_; }
    uint64_t count() const { return count_; }
    // Touch sum so the compiler cannot dead-strip the accumulators.
    double sum_re() const { return sum_re_; }
    double sum_im() const { return sum_im_; }

    void reset()
    {
        checksum_ = kFnvOffset;
        count_ = 0;
        sum_re_ = 0.0;
        sum_im_ = 0.0;
    }

private:
    static constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
    static constexpr uint64_t kFnvPrime = 1099511628211ULL;
    uint64_t checksum_;
    uint64_t count_;
    double sum_re_;
    double sum_im_;
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
std::string run_cmd_first_line(const char* cmd)
{
    FILE* pipe = popen(cmd, "r");
    if (!pipe)
        return "unknown";
    char buf[512];
    std::string line;
    if (fgets(buf, sizeof(buf), pipe)) {
        line = buf;
        while (!line.empty() &&
               (line.back() == '\n' || line.back() == '\r'))
            line.pop_back();
    }
    pclose(pipe);
    return line.empty() ? "unknown" : line;
}

std::string cpu_model()
{
    std::ifstream f("/proc/cpuinfo");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("model name", 0) == 0) {
            const auto pos = line.find(':');
            if (pos != std::string::npos) {
                std::string v = line.substr(pos + 1);
                while (!v.empty() && v.front() == ' ')
                    v.erase(v.begin());
                return v;
            }
        }
    }
    return "unknown";
}

int cpu_logical_cores()
{
    std::ifstream f("/proc/cpuinfo");
    std::string line;
    int n = 0;
    while (std::getline(f, line)) {
        if (line.rfind("processor", 0) == 0)
            ++n;
    }
    return n > 0 ? n : 1;
}

std::string cpu_isa()
{
    std::ifstream f("/proc/cpuinfo");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("flags", 0) == 0 || line.rfind("Features", 0) == 0) {
            std::ostringstream oss;
            if (line.find(" avx512f") != std::string::npos ||
                line.find("\tavx512f") != std::string::npos)
                oss << "AVX-512";
            else if (line.find(" avx2") != std::string::npos)
                oss << "AVX2";
            else if (line.find(" avx") != std::string::npos)
                oss << "AVX";
            else if (line.find(" sse4_2") != std::string::npos)
                oss << "SSE4.2";
            else
                oss << "baseline";
            if (line.find(" fma ") != std::string::npos ||
                line.find(" fma\t") != std::string::npos ||
                line.find(" fma\n") != std::string::npos ||
                line.find(" fma") != std::string::npos)
                oss << "+FMA";
            return oss.str();
        }
    }
    return "unknown";
}

// VmRSS in kB from /proc/self/status; 0 on failure.
uint64_t read_vm_rss_kb()
{
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream iss(line);
            std::string key, unit;
            uint64_t kb = 0;
            iss >> key >> kb >> unit;
            return kb;
        }
    }
    return 0;
}

double process_cpu_seconds()
{
    struct timespec ts {};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) != 0)
        return 0.0;
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) * 1e-9;
}

std::string find_taps_path(const char* filename)
{
    const char* prefixes[] = {
        "testdata/resampler_65_48/",
        "../testdata/resampler_65_48/",
        "../../testdata/resampler_65_48/",
        "../../../testdata/resampler_65_48/",
        "../../../../testdata/resampler_65_48/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + filename;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    throw std::runtime_error(
        std::string("cannot find taps file ") + filename +
        " (tried testdata/resampler_65_48/ from several cwd depths)");
}

std::vector<float> load_taps_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open " + path);
    const size_t bytes = static_cast<size_t>(f.tellg());
    if (bytes % sizeof(float) != 0)
        throw std::runtime_error("taps file size not multiple of float32: " +
                                 path);
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()),
           static_cast<std::streamsize>(bytes));
    if (!f)
        throw std::runtime_error("failed reading " + path);
    return taps;
}

// Non-trivial ring data: complex exponential + low-amplitude noise-like mix.
// Generated once before any timed path.
std::vector<gr_complex> make_ring_data(size_t n)
{
    std::vector<gr_complex> data(n);
    // ~50 MHz tone at 737.28 MS/s + a weaker image-like component.
    const double w0 = 2.0 * M_PI * 50e6 / kInRateHz;
    const double w1 = 2.0 * M_PI * 120e6 / kInRateHz;
    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i);
        const float re = static_cast<float>(
            0.7 * std::cos(w0 * t) + 0.2 * std::cos(w1 * t));
        const float im = static_cast<float>(
            0.7 * std::sin(w0 * t) + 0.2 * std::sin(w1 * t));
        data[i] = gr_complex(re, im);
    }
    return data;
}

void apply_buffer_policy(const gr::block_sptr& b, size_t buffer_items)
{
    if (buffer_items == 0 || !b)
        return;
    // Source/head style: max buffer unbounded, min raised, max_noutput capped.
    b->set_max_output_buffer(0, -1);
    b->set_min_output_buffer(0, static_cast<long>(buffer_items));
    b->set_max_noutput_items(static_cast<int>(buffer_items));
}

struct RunResult {
    double wall_s = 0.0;
    double cpu_s = 0.0;
    uint64_t in_samples = 0;
    uint64_t out_samples = 0;
    uint64_t checksum = 0;
    double sum_re = 0.0;
    double sum_im = 0.0;
    uint64_t rss_before_kb = 0;
    uint64_t rss_after_kb = 0;
};

// Source-only: BulkRepeatSource → head → null_sink
RunResult run_source_only(const std::vector<gr_complex>& ring,
                          uint64_t target,
                          size_t buffer_items)
{
    auto src = BulkRepeatSource::make(ring);
    auto head = gr::blocks::head::make(sizeof(gr_complex), target);
    auto sink = gr::blocks::null_sink::make(sizeof(gr_complex));
    auto tb = gr::make_top_block("resampler_source_only");
    tb->connect(src, 0, head, 0);
    tb->connect(head, 0, sink, 0);
    if (buffer_items > 0) {
        apply_buffer_policy(src, buffer_items);
        apply_buffer_policy(head, buffer_items);
        tb->set_max_noutput_items(static_cast<int>(buffer_items));
    }
    RunResult r;
    r.in_samples = target;
    r.out_samples = target;
    r.rss_before_kb = read_vm_rss_kb();
    const double cpu0 = process_cpu_seconds();
    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    r.wall_s = std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
    r.cpu_s = process_cpu_seconds() - cpu0;
    r.rss_after_kb = read_vm_rss_kb();
    return r;
}

// Flowgraph: BulkRepeatSource → head → rational_resampler_ccf → ChecksumSink
RunResult run_resampler(const std::vector<gr_complex>& ring,
                        const std::vector<float>& taps,
                        uint64_t target,
                        size_t buffer_items,
                        size_t* out_tap_count)
{
    auto src = BulkRepeatSource::make(ring);
    auto head = gr::blocks::head::make(sizeof(gr_complex), target);
    gr::filter::rational_resampler_ccf::sptr resamp;
    if (taps.empty()) {
        // GNU Radio default design (empty taps → internal fractional_bw=0.4).
        resamp = gr::filter::rational_resampler_ccf::make(kInterp, kDecim);
    } else {
        resamp =
            gr::filter::rational_resampler_ccf::make(kInterp, kDecim, taps);
    }
    if (out_tap_count)
        *out_tap_count = resamp->taps().size();
    auto sink = ChecksumSink::make();
    auto tb = gr::make_top_block("resampler_level_a");
    tb->connect(src, 0, head, 0);
    tb->connect(head, 0, resamp, 0);
    tb->connect(resamp, 0, sink, 0);
    if (buffer_items > 0) {
        apply_buffer_policy(src, buffer_items);
        apply_buffer_policy(head, buffer_items);
        apply_buffer_policy(resamp, buffer_items);
        // Sink has no output; still cap its work grant via top_block.
        tb->set_max_noutput_items(static_cast<int>(buffer_items));
    }
    RunResult r;
    r.in_samples = target;
    r.rss_before_kb = read_vm_rss_kb();
    const double cpu0 = process_cpu_seconds();
    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    r.wall_s = std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
    r.cpu_s = process_cpu_seconds() - cpu0;
    r.rss_after_kb = read_vm_rss_kb();
    r.out_samples = sink->count();
    r.checksum = sink->checksum();
    r.sum_re = sink->sum_re();
    r.sum_im = sink->sum_im();
    return r;
}

// Level-B flowgraph: BulkRepeatSource → head → UwbRationalResamplerCcf65_48
// → ChecksumSink
RunResult run_fixed_block(const std::vector<gr_complex>& ring,
                          const std::vector<float>& taps,
                          uint64_t target,
                          size_t buffer_items,
                          size_t* out_tap_count,
                          const char** out_kernel,
                          const std::string& kernel_sel = "default",
                          int num_workers = 1)
{
    auto src = BulkRepeatSource::make(ring);
    auto head = gr::blocks::head::make(sizeof(gr_complex), target);
    auto resamp = gr::uwb::UwbRationalResamplerCcf65_48::make_from_taps(
        taps, /*tag_prop=*/false, /*reset_on_disc=*/true, num_workers);
    if (!kernel_sel.empty() && kernel_sel != "default")
        resamp->set_kernel(kernel_sel);
    if (out_tap_count)
        *out_tap_count = resamp->tap_count();
    if (out_kernel)
        *out_kernel = resamp->kernel_name();
    auto sink = ChecksumSink::make();
    auto tb = gr::make_top_block("resampler_level_b_block");
    tb->connect(src, 0, head, 0);
    tb->connect(head, 0, resamp, 0);
    tb->connect(resamp, 0, sink, 0);
    if (buffer_items > 0) {
        apply_buffer_policy(src, buffer_items);
        apply_buffer_policy(head, buffer_items);
        apply_buffer_policy(resamp, buffer_items);
        tb->set_max_noutput_items(static_cast<int>(buffer_items));
    }
    RunResult r;
    r.in_samples = target;
    r.rss_before_kb = read_vm_rss_kb();
    const double cpu0 = process_cpu_seconds();
    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    r.wall_s = std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
    r.cpu_s = process_cpu_seconds() - cpu0;
    r.rss_after_kb = read_vm_rss_kb();
    r.out_samples = sink->count();
    r.checksum = sink->checksum();
    r.sum_re = sink->sum_re();
    r.sum_im = sink->sum_im();
    return r;
}

// Level-B kernel: direct core.process on a ring (no scheduler / memcpy source).
// Processes `target` input samples by looping over the ring in fixed chunks.
// Does NOT call flush (steady-state rate measurement).
RunResult run_kernel(const std::vector<gr_complex>& ring,
                     const std::vector<float>& taps,
                     uint64_t target,
                     size_t chunk,
                     size_t* out_tap_count,
                     const char** out_kernel,
                     const std::string& kernel_sel = "default",
                     int num_workers = 1)
{
    gr::uwb::core::RationalResampler65_48Core core(taps);
    if (!kernel_sel.empty() && kernel_sel != "default")
        core.set_kernel(kernel_sel);
    core.set_num_workers(num_workers);
    if (out_tap_count)
        *out_tap_count = core.tap_count();
    if (out_kernel)
        *out_kernel = core.kernel_name();

    // Output scratch for one chunk: ceil(chunk*65/48)+65
    const size_t max_out =
        (chunk * kInterp + kDecim - 1) / kDecim + kInterp + 8;
    std::vector<gr_complex> outbuf(max_out);

    // Warm delay line so first timed samples are in steady state.
    {
        const size_t warm = std::min(ring.size(), size_t(4096));
        (void)core.process(ring.data(), warm, outbuf.data(), outbuf.size());
    }
    core.reset();
    // Re-warm after reset without counting.
    {
        const size_t warm = std::min(ring.size(), size_t(4096));
        (void)core.process(ring.data(), warm, outbuf.data(), outbuf.size());
    }

    // FNV checksum over produced outputs (prevent DCE).
    uint64_t checksum = 14695981039346656037ULL;
    constexpr uint64_t kFnvPrime = 1099511628211ULL;
    double sum_re = 0.0, sum_im = 0.0;
    uint64_t out_count = 0;
    uint64_t in_done = 0;
    size_t ring_off = 0;

    RunResult r;
    r.rss_before_kb = read_vm_rss_kb();
    const double cpu0 = process_cpu_seconds();
    const auto t0 = std::chrono::steady_clock::now();

    while (in_done < target) {
        const size_t n = static_cast<size_t>(
            std::min<uint64_t>(chunk, target - in_done));
        // Gather from ring (may wrap).
        // Fast path: contiguous slice.
        size_t produced = 0;
        size_t consumed = 0;
        if (ring_off + n <= ring.size()) {
            auto pr = core.process(ring.data() + ring_off, n, outbuf.data(),
                                   outbuf.size());
            produced = pr.produced;
            consumed = pr.consumed;
            ring_off += consumed;
            if (ring_off >= ring.size())
                ring_off = 0;
        } else {
            // Wrap: two process calls.
            const size_t n1 = ring.size() - ring_off;
            auto pr1 = core.process(ring.data() + ring_off, n1, outbuf.data(),
                                    outbuf.size());
            produced = pr1.produced;
            consumed = pr1.consumed;
            ring_off = 0;
            if (consumed < n && pr1.consumed == n1) {
                const size_t n2 = n - n1;
                auto pr2 =
                    core.process(ring.data(), n2, outbuf.data() + produced,
                                 outbuf.size() - produced);
                produced += pr2.produced;
                consumed += pr2.consumed;
                ring_off = pr2.consumed;
            }
        }
        // If core held back some inputs (output scratch full), still count
        // only consumed.  Retry remainder on next iteration via ring_off.
        in_done += consumed;
        for (size_t i = 0; i < produced; ++i) {
            uint32_t re_bits = 0, im_bits = 0;
            const float re = outbuf[i].real();
            const float im = outbuf[i].imag();
            std::memcpy(&re_bits, &re, sizeof(re_bits));
            std::memcpy(&im_bits, &im, sizeof(im_bits));
            checksum ^= re_bits;
            checksum *= kFnvPrime;
            checksum ^= im_bits;
            checksum *= kFnvPrime;
            sum_re += re;
            sum_im += im;
        }
        out_count += produced;
        if (consumed == 0) {
            // Avoid infinite loop if process cannot make progress.
            break;
        }
    }

    r.wall_s = std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0)
                   .count();
    r.cpu_s = process_cpu_seconds() - cpu0;
    r.rss_after_kb = read_vm_rss_kb();
    r.in_samples = in_done;
    r.out_samples = out_count;
    r.checksum = checksum;
    r.sum_re = sum_re;
    r.sum_im = sum_im;
    return r;
}

double median_of(std::vector<double> v)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    if (n % 2 == 1)
        return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

struct RateStats {
    double med = 0.0;
    double minv = 0.0;
    double maxv = 0.0;
};

RateStats stats_of(const std::vector<double>& v)
{
    RateStats s;
    if (v.empty())
        return s;
    s.med = median_of(v);
    s.minv = *std::min_element(v.begin(), v.end());
    s.maxv = *std::max_element(v.begin(), v.end());
    return s;
}

void print_info_block(const char* profile,
                      size_t tap_count,
                      size_t buffer_items,
                      const RunResult& r,
                      const std::string& gr_ver,
                      const std::string& compiler,
                      const std::string& cpu,
                      int cores,
                      const std::string& isa,
                      const std::string& volk_machine)
{
    const double in_msps =
        r.wall_s > 0.0 ? r.in_samples / r.wall_s / 1e6 : 0.0;
    const double out_msps =
        r.wall_s > 0.0 ? r.out_samples / r.wall_s / 1e6 : 0.0;
    const double rt_ratio = out_msps / (kOutRateHz / 1e6);
    // Estimated payload GB/s at measured rate (scale theoretical contract).
    const double read_gbps = in_msps / (kInRateHz / 1e6) * kInPayloadGBps;
    const double write_gbps = out_msps / (kOutRateHz / 1e6) * kOutPayloadGBps;
    const uint64_t expected_out =
        (r.in_samples * static_cast<uint64_t>(kInterp)) /
        static_cast<uint64_t>(kDecim);
    const int64_t drift =
        static_cast<int64_t>(r.out_samples) - static_cast<int64_t>(expected_out);

    std::cout << "--- info profile=" << profile << " ---\n"
              << "gnuradio_version=" << gr_ver << "\n"
              << "compiler=" << compiler << "\n"
              << "cpu_model=" << cpu << "\n"
              << "cpu_cores=" << cores << "\n"
              << "cpu_isa=" << isa << "\n"
              << "volk_machine=" << volk_machine << "\n"
              << "input_format=CF32 output_format=CF32\n"
              << "interpolation=" << kInterp << " decimation=" << kDecim
              << "\n"
              << "tap_count=" << tap_count
              << " taps_per_arm_approx="
              << (tap_count + kInterp - 1) / kInterp << "\n"
              << "buffer_items=" << buffer_items
              << " max_noutput_items=" << buffer_items << "\n"
              << "input_samples=" << r.in_samples
              << " output_samples=" << r.out_samples
              << " expected_output_samples=" << expected_out
              << " output_count_drift=" << drift << "\n"
              << "wall_time_s=" << std::fixed << std::setprecision(6)
              << r.wall_s << " cpu_time_s=" << r.cpu_s << "\n"
              << "input_MSps=" << std::setprecision(3) << in_msps
              << " output_MSps=" << out_msps
              << " realtime_ratio=" << std::setprecision(4) << rt_ratio
              << "\n"
              << "est_read_GBps=" << std::setprecision(4) << read_gbps
              << " est_write_GBps=" << write_gbps
              << " contract_in_GBps=" << kInPayloadGBps
              << " contract_out_GBps=" << kOutPayloadGBps << "\n"
              << "checksum=0x" << std::hex << r.checksum << std::dec
              << " sum_re=" << std::setprecision(6) << r.sum_re
              << " sum_im=" << r.sum_im << "\n"
              << "rss_before_kB=" << r.rss_before_kb
              << " rss_after_kB=" << r.rss_after_kb
              << " rss_delta_kB="
              << static_cast<int64_t>(r.rss_after_kb) -
                     static_cast<int64_t>(r.rss_before_kb)
              << "\n";
}

void print_rate_line(const char* mode,
                     const char* profile,
                     int round,
                     const RunResult& r)
{
    const double in_msps =
        r.wall_s > 0.0 ? r.in_samples / r.wall_s / 1e6 : 0.0;
    const double out_msps =
        r.wall_s > 0.0 ? r.out_samples / r.wall_s / 1e6 : 0.0;
    const double rt_ratio = out_msps / (kOutRateHz / 1e6);
    std::cout << "mode=" << mode << " profile=" << profile
              << " round=" << round << " wall_s=" << std::fixed
              << std::setprecision(6) << r.wall_s
              << " input_MSps=" << std::setprecision(3) << in_msps
              << " output_MSps=" << out_msps
              << " realtime_ratio=" << std::setprecision(4) << rt_ratio
              << " in_samples=" << r.in_samples
              << " out_samples=" << r.out_samples << " checksum=0x" << std::hex
              << r.checksum << std::dec << "\n";
}

} // namespace

enum class BenchMode { Builtin, Kernel, Block, Profile, All };

BenchMode parse_mode(const std::string& s)
{
    if (s == "builtin" || s == "a" || s == "A" || s == "flowgraph")
        return BenchMode::Builtin;
    if (s == "kernel" || s == "k")
        return BenchMode::Kernel;
    if (s == "block" || s == "b" || s == "B" || s == "fixed")
        return BenchMode::Block;
    if (s == "profile" || s == "prof")
        return BenchMode::Profile;
    if (s == "all")
        return BenchMode::All;
    throw std::invalid_argument(
        "unknown mode: " + s +
        " (use builtin|kernel|block|profile|all)");
}

void run_profile_rounds(const char* mode_name,
                        const char* profile,
                        size_t tap_count,
                        const char* kernel_name,
                        const std::vector<double>& in_rates,
                        const std::vector<double>& out_rates,
                        const std::vector<double>& rt_ratios,
                        const RunResult& last)
{
    const auto sin = stats_of(in_rates);
    const auto sout = stats_of(out_rates);
    const auto srt = stats_of(rt_ratios);
    std::cout << "mode=" << mode_name << " profile=" << profile << " summary"
              << " median_input_MSps=" << std::fixed << std::setprecision(3)
              << sin.med << " min_input_MSps=" << sin.minv
              << " max_input_MSps=" << sin.maxv
              << " median_output_MSps=" << sout.med
              << " min_output_MSps=" << sout.minv
              << " max_output_MSps=" << sout.maxv
              << " median_realtime_ratio=" << std::setprecision(4) << srt.med
              << " min_realtime_ratio=" << srt.minv
              << " max_realtime_ratio=" << srt.maxv
              << " tap_count=" << tap_count
              << " kernel=" << (kernel_name ? kernel_name : "n/a") << "\n";

    const bool min_ok =
        sin.med >= (kInRateHz / 1e6) && sout.med >= (kOutRateHz / 1e6);
    const bool prod_ok = sin.med >= (kInRateHz / 1e6) * 1.2 &&
                         sout.med >= (kOutRateHz / 1e6) * 1.2;
    std::cout << "verdict mode=" << mode_name << " profile=" << profile
              << " min_realtime_bar=" << (min_ok ? "PASS" : "FAIL")
              << " production_1p2x_bar=" << (prod_ok ? "PASS" : "FAIL")
              << " last_checksum=0x" << std::hex << last.checksum << std::dec
              << "\n";
}

int main(int argc, char** argv)
{
    // Parse: [mode] [target] [buffer] [kernel] [workers]
    //    OR: [target] [buffer] (legacy Level A → builtin)
    BenchMode mode = BenchMode::All;
    uint64_t target = kDefaultTarget;
    size_t buffer_items = kDefaultBuffer;
    std::string kernel_sel = "default";
    int num_workers = 1;
    int argi = 1;
    if (argc > 1) {
        const std::string a1 = argv[1];
        bool is_mode = false;
        try {
            mode = parse_mode(a1);
            is_mode = true;
        } catch (...) {
            is_mode = false;
        }
        if (is_mode) {
            ++argi;
        } else {
            mode = BenchMode::Builtin;
        }
    }
    if (argi < argc)
        target = std::stoull(argv[argi++]);
    if (argi < argc)
        buffer_items = static_cast<size_t>(std::stoull(argv[argi++]));
    if (argi < argc)
        kernel_sel = argv[argi++];
    if (argi < argc)
        num_workers = std::stoi(argv[argi++]);
    if (num_workers < 1)
        num_workers = 1;

    // Optional --taps <path>: add a "custom" profile with the given float32
    // taps file.  Scans remaining args so it can appear after positional ones.
    std::string custom_taps_path;
    for (int i = argi; i < argc; ++i) {
        if (std::string(argv[i]) == "--taps" && i + 1 < argc) {
            custom_taps_path = argv[i + 1];
            break;
        }
    }

    if (target < static_cast<uint64_t>(kDecim) * 16) {
        std::cerr << "target_input_samples too small\n";
        return 2;
    }

    const std::string gr_ver =
        run_cmd_first_line("gnuradio-config-info --version 2>/dev/null");
    const std::string compiler =
        run_cmd_first_line("c++ --version 2>/dev/null");
    const std::string cpu = cpu_model();
    const int cores = cpu_logical_cores();
    const std::string isa = cpu_isa();
    const std::string volk_machine =
        run_cmd_first_line("volk-config-info --machine 2>/dev/null");

    std::cout << "benchmark_resampler_65_48\n"
              << "contract: " << kInRateHz / 1e6 << " MS/s * " << kInterp
              << "/" << kDecim << " = " << kOutRateHz / 1e6 << " MS/s\n"
              << "target_input_samples=" << target
              << " buffer_items=" << buffer_items
              << " timed_rounds=" << kTimedRounds
              << " kernel_sel=" << kernel_sel
              << " num_workers=" << num_workers << "\n"
              << "payload_contract CF32_in=" << kInPayloadGBps
              << " GB/s CF32_out=" << kOutPayloadGBps << " GB/s\n";

    const std::string quality_path = find_taps_path("taps_quality.txt");
    const std::string realtime_path = find_taps_path("taps_realtime.txt");
    const auto taps_quality = load_taps_f32(quality_path);
    const auto taps_realtime = load_taps_f32(realtime_path);
    std::cout << "taps_quality_path=" << quality_path
              << " count=" << taps_quality.size() << "\n"
              << "taps_realtime_path=" << realtime_path
              << " count=" << taps_realtime.size() << "\n";
    std::vector<float> taps_custom;
    if (!custom_taps_path.empty()) {
        taps_custom = load_taps_f32(custom_taps_path);
        std::cout << "taps_custom_path=" << custom_taps_path
                  << " count=" << taps_custom.size() << "\n";
    }
    const auto ring = make_ring_data(kRingSamples);

    struct Profile {
        const char* name;
        const std::vector<float>* taps; // nullptr → GR default (builtin only)
    };

    // Build a profile list, appending a "custom" profile when --taps was given.
    auto make_profiles = [&](std::vector<Profile> base) {
        if (!custom_taps_path.empty() && !taps_custom.empty())
            base.push_back(Profile{ "custom", &taps_custom });
        return base;
    };

    // ------------------------------------------------------------------
    // Level A: built-in
    // ------------------------------------------------------------------
    if (mode == BenchMode::Builtin || mode == BenchMode::All) {
        std::cout << "\n######## Level A: built-in rational_resampler_ccf "
                     "########\n";
        if (mode == BenchMode::Builtin || mode == BenchMode::All) {
            std::cout << "\n=== source_only baseline (no resampler) ===\n";
            (void)run_source_only(ring, target, buffer_items);
            std::vector<double> in_rates;
            RunResult last;
            for (int r = 0; r < kTimedRounds; ++r) {
                last = run_source_only(ring, target, buffer_items);
                const double in_msps =
                    last.wall_s > 0.0 ? last.in_samples / last.wall_s / 1e6
                                      : 0.0;
                in_rates.push_back(in_msps);
                print_rate_line("source_only", "n/a", r + 1, last);
            }
            const auto st = stats_of(in_rates);
            std::cout << "mode=source_only profile=n/a summary"
                      << " median_input_MSps=" << std::fixed
                      << std::setprecision(3) << st.med
                      << " min_input_MSps=" << st.minv
                      << " max_input_MSps=" << st.maxv << "\n";
        }

        const auto profiles = make_profiles({
            { "default", nullptr },
            { "quality", &taps_quality },
            { "realtime", &taps_realtime },
        });
        for (const auto& prof : profiles) {
            const std::vector<float> empty;
            const std::vector<float>& taps = prof.taps ? *prof.taps : empty;
            std::cout << "\n=== flowgraph rational_resampler_ccf profile="
                      << prof.name << " ===\n";
            size_t tap_count = 0;
            {
                const uint64_t warm_n =
                    std::min(target, static_cast<uint64_t>(buffer_items) * 4);
                auto warm =
                    run_resampler(ring, taps, warm_n, buffer_items, &tap_count);
                std::cout << "warmup profile=" << prof.name
                          << " tap_count=" << tap_count
                          << " wall_s=" << std::fixed << std::setprecision(6)
                          << warm.wall_s << " checksum=0x" << std::hex
                          << warm.checksum << std::dec << "\n";
            }
            (void)run_resampler(ring, taps, target, buffer_items, nullptr);

            std::vector<double> in_rates, out_rates, rt_ratios;
            RunResult last;
            for (int r = 0; r < kTimedRounds; ++r) {
                last = run_resampler(ring, taps, target, buffer_items, nullptr);
                const double in_msps =
                    last.wall_s > 0.0 ? last.in_samples / last.wall_s / 1e6
                                      : 0.0;
                const double out_msps =
                    last.wall_s > 0.0 ? last.out_samples / last.wall_s / 1e6
                                      : 0.0;
                in_rates.push_back(in_msps);
                out_rates.push_back(out_msps);
                rt_ratios.push_back(out_msps / (kOutRateHz / 1e6));
                print_rate_line("builtin", prof.name, r + 1, last);
            }
            run_profile_rounds("builtin",
                               prof.name,
                               tap_count,
                               "volk_fir (GR)",
                               in_rates,
                               out_rates,
                               rt_ratios,
                               last);
        }
    }

    // ------------------------------------------------------------------
    // Level B kernel
    // ------------------------------------------------------------------
    if (mode == BenchMode::Kernel || mode == BenchMode::All) {
        std::cout << "\n######## Level B: fixed core kernel ########\n";
        const auto profiles = make_profiles({
            { "quality", &taps_quality },
            { "realtime", &taps_realtime },
        });
        const size_t chunk = buffer_items > 0 ? buffer_items : size_t(4096);
        for (const auto& prof : profiles) {
            std::cout << "\n=== kernel RationalResampler65_48Core profile="
                      << prof.name << " ===\n";
            size_t tap_count = 0;
            const char* kname = "scalar";
            (void)run_kernel(ring, *prof.taps, std::min(target, uint64_t(1e6)),
                             chunk, &tap_count, &kname, kernel_sel,
                             num_workers);
            std::cout << "kernel_selected=" << kname
                      << " tap_count=" << tap_count
                      << " arm_length=" << (tap_count + 64) / 65
                      << " chunk=" << chunk
                      << " workers=" << num_workers << "\n";
            (void)run_kernel(ring, *prof.taps, target, chunk, nullptr, nullptr,
                             kernel_sel, num_workers);

            std::vector<double> in_rates, out_rates, rt_ratios;
            RunResult last;
            for (int r = 0; r < kTimedRounds; ++r) {
                last = run_kernel(ring, *prof.taps, target, chunk, nullptr,
                                  &kname, kernel_sel, num_workers);
                const double in_msps =
                    last.wall_s > 0.0 ? last.in_samples / last.wall_s / 1e6
                                      : 0.0;
                const double out_msps =
                    last.wall_s > 0.0 ? last.out_samples / last.wall_s / 1e6
                                      : 0.0;
                in_rates.push_back(in_msps);
                out_rates.push_back(out_msps);
                rt_ratios.push_back(out_msps / (kOutRateHz / 1e6));
                print_rate_line("kernel", prof.name, r + 1, last);
            }
            const size_t H = (tap_count + kInterp - 1) / kInterp;
            const double med_out = median_of(out_rates);
            const double gmac = med_out * 1e6 * static_cast<double>(H) / 1e9;
            std::cout << "mode=kernel profile=" << prof.name
                      << " median_GMAC_s=" << std::fixed << std::setprecision(2)
                      << gmac << " arm_H=" << H << "\n";
            run_profile_rounds("kernel",
                               prof.name,
                               tap_count,
                               kname,
                               in_rates,
                               out_rates,
                               rt_ratios,
                               last);
        }
    }

    // ------------------------------------------------------------------
    // Level B block flowgraph
    // ------------------------------------------------------------------
    if (mode == BenchMode::Block || mode == BenchMode::All) {
        std::cout << "\n######## Level B: fixed block flowgraph ########\n";
        const auto profiles = make_profiles({
            { "quality", &taps_quality },
            { "realtime", &taps_realtime },
        });
        for (const auto& prof : profiles) {
            std::cout << "\n=== block UwbRationalResamplerCcf65_48 profile="
                      << prof.name << " ===\n";
            size_t tap_count = 0;
            const char* kname = "scalar";
            {
                const uint64_t warm_n =
                    std::min(target, static_cast<uint64_t>(buffer_items) * 4);
                auto warm = run_fixed_block(ring, *prof.taps, warm_n,
                                            buffer_items, &tap_count, &kname,
                                            kernel_sel, num_workers);
                std::cout << "warmup profile=" << prof.name
                          << " tap_count=" << tap_count
                          << " kernel=" << kname
                          << " workers=" << num_workers
                          << " wall_s=" << std::fixed << std::setprecision(6)
                          << warm.wall_s << " checksum=0x" << std::hex
                          << warm.checksum << std::dec << "\n";
            }
            (void)run_fixed_block(ring, *prof.taps, target, buffer_items,
                                  nullptr, nullptr, kernel_sel, num_workers);

            std::vector<double> in_rates, out_rates, rt_ratios;
            RunResult last;
            for (int r = 0; r < kTimedRounds; ++r) {
                last = run_fixed_block(ring, *prof.taps, target, buffer_items,
                                       nullptr, &kname, kernel_sel,
                                       num_workers);
                const double in_msps =
                    last.wall_s > 0.0 ? last.in_samples / last.wall_s / 1e6
                                      : 0.0;
                const double out_msps =
                    last.wall_s > 0.0 ? last.out_samples / last.wall_s / 1e6
                                      : 0.0;
                in_rates.push_back(in_msps);
                out_rates.push_back(out_msps);
                rt_ratios.push_back(out_msps / (kOutRateHz / 1e6));
                print_rate_line("block", prof.name, r + 1, last);
            }
            const size_t H = (tap_count + kInterp - 1) / kInterp;
            const double med_out = median_of(out_rates);
            const double gmac = med_out * 1e6 * static_cast<double>(H) / 1e9;
            std::cout << "mode=block profile=" << prof.name
                      << " median_GMAC_s=" << std::fixed << std::setprecision(2)
                      << gmac << " arm_H=" << H
                      << " workers=" << num_workers << "\n";
            run_profile_rounds("block",
                               prof.name,
                               tap_count,
                               kname,
                               in_rates,
                               out_rates,
                               rt_ratios,
                               last);
        }
    }

    // ------------------------------------------------------------------
    // Stage-1 profile attribution (core micro-benchmark)
    // ------------------------------------------------------------------
    if (mode == BenchMode::Profile) {
        std::cout << "\n######## Stage-1 core profile attribution ########\n";
        const auto profiles = make_profiles({
            { "quality", &taps_quality },
            { "realtime", &taps_realtime },
        });
        // Compare legacy vs macroblock kernels on the same input.
        const char* kernels[] = { "volk_legacy", "volk_macroblock", "avx2",
                                  "scalar_legacy" };
        const size_t chunk = buffer_items > 0 ? buffer_items : size_t(1 << 16);
        const uint64_t prof_target =
            std::min(target, static_cast<uint64_t>(20'000'000));

        for (const auto& prof : profiles) {
            for (const char* kn : kernels) {
                gr::uwb::core::RationalResampler65_48Core core(*prof.taps);
                try {
                    core.set_kernel(kn);
                } catch (...) {
                    std::cout << "skip kernel=" << kn << " (not available)\n";
                    continue;
                }
                core.set_num_workers(num_workers);
                core.enable_profiling(true);

                const size_t max_out =
                    (chunk * kInterp + kDecim - 1) / kDecim + kInterp + 8;
                std::vector<gr_complex> outbuf(max_out);
                // Warm
                (void)core.process(ring.data(),
                                   std::min(ring.size(), size_t(4096)),
                                   outbuf.data(), outbuf.size());
                core.reset();
                core.set_kernel(kn);
                core.set_num_workers(num_workers);
                core.enable_profiling(true);
                core.reset_profile();

                uint64_t in_done = 0;
                size_t ring_off = 0;
                const auto t0 = std::chrono::steady_clock::now();
                while (in_done < prof_target) {
                    const size_t n = static_cast<size_t>(
                        std::min<uint64_t>(chunk, prof_target - in_done));
                    size_t consumed = 0;
                    if (ring_off + n <= ring.size()) {
                        auto pr = core.process(ring.data() + ring_off, n,
                                               outbuf.data(), outbuf.size());
                        consumed = pr.consumed;
                        ring_off += consumed;
                        if (ring_off >= ring.size())
                            ring_off = 0;
                    } else {
                        const size_t n1 = ring.size() - ring_off;
                        auto pr1 = core.process(ring.data() + ring_off, n1,
                                                outbuf.data(), outbuf.size());
                        consumed = pr1.consumed;
                        ring_off = 0;
                        if (consumed < n && pr1.consumed == n1) {
                            auto pr2 = core.process(
                                ring.data(), n - n1, outbuf.data(),
                                outbuf.size());
                            consumed += pr2.consumed;
                            ring_off = pr2.consumed;
                        }
                    }
                    in_done += consumed;
                    if (consumed == 0)
                        break;
                }
                const double wall = std::chrono::duration<double>(
                                        std::chrono::steady_clock::now() - t0)
                                        .count();
                const auto& ps = core.profile_stats();
                const double tot = ps.ns_total > 0 ? ps.ns_total : 1.0;
                const double in_msps =
                    wall > 0 ? static_cast<double>(in_done) / wall / 1e6 : 0;
                const double out_msps =
                    wall > 0
                        ? static_cast<double>(ps.outputs) / wall / 1e6
                        : 0;
                const size_t H = core.arm_length();
                const double gmac =
                    out_msps * 1e6 * static_cast<double>(H) / 1e9;

                std::cout << std::fixed
                          << "profile_attr profile=" << prof.name
                          << " kernel=" << core.kernel_name()
                          << " workers=" << num_workers
                          << " in_samples=" << in_done
                          << " out_samples=" << ps.outputs
                          << " wall_s=" << std::setprecision(4) << wall
                          << " input_MSps=" << std::setprecision(2) << in_msps
                          << " output_MSps=" << out_msps
                          << " GMAC_s=" << gmac << "\n"
                          << "  assemble_us=" << std::setprecision(1)
                          << ps.ns_assemble / 1e3
                          << " (" << std::setprecision(1)
                          << 100.0 * ps.ns_assemble / tot << "%)"
                          << " schedule_us=" << ps.ns_schedule / 1e3
                          << " (" << 100.0 * ps.ns_schedule / tot << "%)"
                          << " fir_us=" << ps.ns_fir / 1e3
                          << " (" << 100.0 * ps.ns_fir / tot << "%)"
                          << " state_us=" << ps.ns_state / 1e3
                          << " (" << 100.0 * ps.ns_state / tot << "%)"
                          << " total_us=" << ps.ns_total / 1e3
                          << " calls=" << ps.calls
                          << " macroblocks=" << ps.macroblocks
                          << " us_per_call="
                          << (ps.calls ? ps.ns_total / 1e3 /
                                             static_cast<double>(ps.calls)
                                       : 0.0)
                          << "\n";
            }
        }
        std::cout
            << "note: kernel mode may look slower than block mode because "
               "checksum of every sample runs on the same thread as the FIR; "
               "block mode can overlap ChecksumSink with the resampler via "
               "the GR scheduler.\n";
    }

    std::cout << "\ndone.\n";
    return 0;
}
