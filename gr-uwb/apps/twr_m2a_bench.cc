/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A A13 observation program: allocation counts and latency percentiles for
 * the PREPARED `M2aContext` (and, for comparison, the cold one-shot wrapper).
 *
 * G0 appendix A.4 (docs/twr/M2-A_G0接口与数字坐标.md) is the contract:
 *
 *   * allocations are counted by overriding the global `operator new`,
 *     `operator new[]`, the aligned overloads AND `malloc`/`calloc`/`realloc`.
 *     The counters are std::atomic and never allocate; `operator new` calls
 *     glibc's `__libc_malloc` DIRECTLY so the malloc counter does not
 *     double-count C++ allocations.  COUNTER SCOPE: only the regions explicitly
 *     bracketed by set_counting(true/false) are counted; allocations made by
 *     the C runtime / libc inside those regions are included, and allocations
 *     NOT routed through these hooks are NOT claimed to be zero.
 *   * timing and allocation run in SEPARATE passes.
 *   * the matrix is 2 rates x 3 frames x 2 formats, each with a repeated-frame
 *     set and a changed timestamp/seq/addr set; clean and long-search/failure
 *     paths are reported separately.
 *   * >= 1000 hot iterations per group; sample count, P50/P95/P99/max, cold
 *     latency and RSS are reported.  P99.9 is never reported (sample count is
 *     not sufficient for it).
 *
 * It uses the PREPARED context for the matrix.  The long-search / failure paths
 * use the PUBLIC `m2a_demod_work` stage (not a private pipeline) and are
 * labelled as such.  This is an OBSERVATION tool, not a QA gate: it does not
 * assert thresholds.  It writes CSV to stdout (or --csv PATH); it never writes
 * into the source tree.
 *
 * EVIDENCE BOUNDARY: software timings do NOT establish a hardware reply-delay
 * lower bound, a 1 GS/s real-time rate, or dual-RX viability.  Nothing here is
 * a hardware measurement.
 */

#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_phy.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <new>
#include <sstream>
#include <string>
#include <vector>

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "testdata"
#endif

namespace twr = gr::uwb::twr;
namespace mod = gr::uwb::mod;
using gr_complex = std::complex<float>;

// ===========================================================================
// Allocation counters (G0 A.4).  Defined at global scope so they interpose on
// the whole program.  Counting is OFF unless explicitly enabled, so program
// start-up / argument parsing / CSV printing are not counted.
// ===========================================================================
namespace {
std::atomic<uint64_t> g_new{ 0 };
std::atomic<uint64_t> g_newarr{ 0 };
std::atomic<uint64_t> g_aligned{ 0 };
std::atomic<uint64_t> g_malloc_c{ 0 };
std::atomic<uint64_t> g_calloc_c{ 0 };
std::atomic<uint64_t> g_realloc_c{ 0 };
std::atomic<bool> g_count{ false };
} // namespace

extern "C" {
void* __libc_malloc(size_t);
void* __libc_calloc(size_t, size_t);
void* __libc_realloc(void*, size_t);
void __libc_free(void*);
}

static inline bool m2a_bench_counting()
{
    return g_count.load(std::memory_order_relaxed);
}

void* operator new(std::size_t n)
{
    if (m2a_bench_counting())
        g_new.fetch_add(1, std::memory_order_relaxed);
    void* p = __libc_malloc(n ? n : 1);
    if (!p)
        throw std::bad_alloc();
    return p;
}
void* operator new[](std::size_t n)
{
    if (m2a_bench_counting())
        g_newarr.fetch_add(1, std::memory_order_relaxed);
    void* p = __libc_malloc(n ? n : 1);
    if (!p)
        throw std::bad_alloc();
    return p;
}
void* operator new(std::size_t n, std::align_val_t a)
{
    if (m2a_bench_counting())
        g_aligned.fetch_add(1, std::memory_order_relaxed);
    void* p = nullptr;
    if (::posix_memalign(&p, static_cast<size_t>(a), n ? n : 1) != 0)
        throw std::bad_alloc();
    return p;
}
void* operator new[](std::size_t n, std::align_val_t a)
{
    if (m2a_bench_counting())
        g_aligned.fetch_add(1, std::memory_order_relaxed);
    void* p = nullptr;
    if (::posix_memalign(&p, static_cast<size_t>(a), n ? n : 1) != 0)
        throw std::bad_alloc();
    return p;
}
void operator delete(void* p) noexcept { __libc_free(p); }
void operator delete[](void* p) noexcept { __libc_free(p); }
void operator delete(void* p, std::size_t) noexcept { __libc_free(p); }
void operator delete[](void* p, std::size_t) noexcept { __libc_free(p); }
void operator delete(void* p, std::align_val_t) noexcept { __libc_free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { __libc_free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept
{
    __libc_free(p);
}
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept
{
    __libc_free(p);
}

extern "C" void* malloc(size_t n)
{
    if (m2a_bench_counting())
        g_malloc_c.fetch_add(1, std::memory_order_relaxed);
    return __libc_malloc(n ? n : 1);
}
extern "C" void* calloc(size_t a, size_t b)
{
    if (m2a_bench_counting())
        g_calloc_c.fetch_add(1, std::memory_order_relaxed);
    return __libc_calloc(a, b);
}
extern "C" void* realloc(void* p, size_t n)
{
    if (m2a_bench_counting())
        g_realloc_c.fetch_add(1, std::memory_order_relaxed);
    return __libc_realloc(p, n);
}

namespace {

// ===========================================================================
// Counter plumbing
// ===========================================================================

struct AllocCounts {
    uint64_t new_calls = 0;
    uint64_t newarr_calls = 0;
    uint64_t aligned_calls = 0;
    uint64_t malloc_calls = 0;
    uint64_t calloc_calls = 0;
    uint64_t realloc_calls = 0;
    uint64_t total() const
    {
        return new_calls + newarr_calls + aligned_calls + malloc_calls +
               calloc_calls + realloc_calls;
    }
};

AllocCounts read_counts()
{
    AllocCounts c;
    c.new_calls = g_new.load(std::memory_order_relaxed);
    c.newarr_calls = g_newarr.load(std::memory_order_relaxed);
    c.aligned_calls = g_aligned.load(std::memory_order_relaxed);
    c.malloc_calls = g_malloc_c.load(std::memory_order_relaxed);
    c.calloc_calls = g_calloc_c.load(std::memory_order_relaxed);
    c.realloc_calls = g_realloc_c.load(std::memory_order_relaxed);
    return c;
}

void reset_counts()
{
    g_new.store(0, std::memory_order_relaxed);
    g_newarr.store(0, std::memory_order_relaxed);
    g_aligned.store(0, std::memory_order_relaxed);
    g_malloc_c.store(0, std::memory_order_relaxed);
    g_calloc_c.store(0, std::memory_order_relaxed);
    g_realloc_c.store(0, std::memory_order_relaxed);
}

void set_counting(bool on) { g_count.store(on, std::memory_order_relaxed); }

AllocCounts delta(const AllocCounts& a, const AllocCounts& b)
{
    AllocCounts d;
    d.new_calls = b.new_calls - a.new_calls;
    d.newarr_calls = b.newarr_calls - a.newarr_calls;
    d.aligned_calls = b.aligned_calls - a.aligned_calls;
    d.malloc_calls = b.malloc_calls - a.malloc_calls;
    d.calloc_calls = b.calloc_calls - a.calloc_calls;
    d.realloc_calls = b.realloc_calls - a.realloc_calls;
    return d;
}

// ===========================================================================
// RSS (read OUTSIDE any timed / counted region)
// ===========================================================================

size_t rss_kb()
{
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream is(line.substr(6));
            size_t kb = 0;
            is >> kb;
            return kb;
        }
    }
    return 0;
}

// ===========================================================================
// Percentiles (nearest-rank on a sorted sample; caller sorts)
// ===========================================================================

double percentile_sorted(const std::vector<double>& v, double p)
{
    if (v.empty())
        return 0.0;
    size_t idx = static_cast<size_t>(std::ceil(p / 100.0 * static_cast<double>(v.size())));
    if (idx == 0)
        idx = 1;
    if (idx > v.size())
        idx = v.size();
    return v[idx - 1];
}

// ===========================================================================
// Taps + frames + config
// ===========================================================================

std::string testdata_dir() { return UWB_TESTDATA_DIR; }

std::vector<float> load_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        std::cerr << "cannot open " << path << "\n";
        std::exit(2);
    }
    const auto bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(bytes));
    return v;
}

const std::vector<float>& tx_taps_for(twr::M2aNativeRate r)
{
    static const std::vector<float> t48 =
        load_f32(testdata_dir() + "/twr/m2a/taps/tx_48_65.f32");
    static const std::vector<float> t32 =
        load_f32(testdata_dir() + "/twr/m2a/taps/tx_32_65.f32");
    return (r == twr::M2aNativeRate::Uc200_737280000) ? t48 : t32;
}

const std::vector<float>& rx_taps_for(twr::M2aNativeRate r)
{
    static const std::vector<float> t65_48 =
        load_f32(testdata_dir() + "/resampler_65_48/taps_quality_minorder.txt");
    static const std::vector<float> t65_32 =
        load_f32(testdata_dir() + "/resampler_65_32/taps_quality_minorder.txt");
    return (r == twr::M2aNativeRate::Uc200_737280000) ? t65_48 : t65_32;
}

twr::Frame make_frame(twr::FrameType t, int variant)
{
    twr::Frame f;
    f.version = twr::kFrameVersion;
    f.function_code = t;
    f.session_id = static_cast<uint16_t>(0x1000 + variant);
    f.seq = static_cast<uint16_t>(0x0042 + 7 * variant);
    f.pan_id = 0x1234;
    f.src_addr = static_cast<uint16_t>(0x0005 + variant);
    f.dst_addr = static_cast<uint16_t>(0x0007 + variant);
    f.flags = twr::make_flags(true);
    switch (t) {
    case twr::FrameType::Poll:
        break;
    case twr::FrameType::Response:
        f.set(twr::TimestampField::T2B, 0x1122334455ULL + variant);
        f.set(twr::TimestampField::T3B, 0x2233445566ULL + variant);
        break;
    case twr::FrameType::Final:
        f.set(twr::TimestampField::T1A, 0x00000000A1ULL + variant);
        f.set(twr::TimestampField::T4A, 0x00000000A2ULL + variant);
        f.set(twr::TimestampField::T5A, 0x00000000A3ULL + variant);
        break;
    default:
        break;
    }
    return f;
}

twr::M2aConfig make_cfg(twr::M2aNativeRate rate, twr::M2aIqFormat fmt)
{
    twr::M2aConfig c;
    c.native_rate = rate;
    c.iq_format = fmt;
    c.code_index = 9;
    c.sync_repetitions = 64;
    c.sfd_mode = "ieee";
    c.insert_sts = false;
    c.ranging = true;
    c.peak_amplitude = 0.8f;
    c.tx_taps = tx_taps_for(rate);
    c.rx_taps = rx_taps_for(rate);
    return c;
}

// ===========================================================================
// One measured group
// ===========================================================================

struct TimingStats {
    size_t iters = 0;
    double cold_us = 0.0;
    double p50_us = 0.0;
    double p95_us = 0.0;
    double p99_us = 0.0;
    double max_us = 0.0;
    size_t rss_start_kb = 0;
    size_t rss_end_kb = 0;
    size_t rss_peak_kb = 0;
};

// Allocation pass: prepare / first-frame / hot-frame counts for a prepared
// context running `frame` `iters` times.  Timing is NOT measured here.
struct AllocBreakdown {
    AllocCounts prepare;
    AllocCounts first_frame;
    AllocCounts hot_total;
    size_t hot_iters = 0;
};

AllocBreakdown measure_alloc(twr::M2aContext& ctx, const twr::Frame& frame,
                             const twr::FrameProfile& prof,
                             const twr::M2aConfig& cfg, size_t iters,
                             bool& ok)
{
    AllocBreakdown b;
    ok = false;
    set_counting(true);
    reset_counts();
    AllocCounts before = read_counts();
    std::string why;
    if (!ctx.prepare(cfg, why)) {
        set_counting(false);
        std::cerr << "prepare failed: " << why << "\n";
        return b;
    }
    AllocCounts after_prepare = read_counts();
    b.prepare = delta(before, after_prepare);

    twr::M2aResult out;
    AllocCounts before_first = read_counts();
    bool r = ctx.run(frame, prof, out, why);
    AllocCounts after_first = read_counts();
    b.first_frame = delta(before_first, after_first);
    if (!r || !out.ok) {
        set_counting(false);
        std::cerr << "first frame failed: " << why << "\n";
        return b;
    }

    AllocCounts before_hot = read_counts();
    for (size_t i = 0; i < iters; ++i) {
        twr::M2aResult o;
        if (!ctx.run(frame, prof, o, why) || !o.ok) {
            set_counting(false);
            std::cerr << "hot frame " << i << " failed: " << why << "\n";
            return b;
        }
    }
    AllocCounts after_hot = read_counts();
    b.hot_total = delta(before_hot, after_hot);
    b.hot_iters = iters;
    set_counting(false);
    ok = true;
    return b;
}

// Timing pass: cold latency (prepare + first frame) then `iters` hot frames.
TimingStats measure_timing(twr::M2aContext& ctx, const twr::Frame& frame,
                           const twr::FrameProfile& prof,
                           const twr::M2aConfig& cfg, size_t iters, bool& ok)
{
    using clock = std::chrono::steady_clock;
    TimingStats s;
    ok = false;
    std::string why;

    auto t0 = clock::now();
    if (!ctx.prepare(cfg, why)) {
        std::cerr << "prepare failed: " << why << "\n";
        return s;
    }
    twr::M2aResult out;
    if (!ctx.run(frame, prof, out, why) || !out.ok) {
        std::cerr << "cold frame failed: " << why << "\n";
        return s;
    }
    auto t1 = clock::now();
    s.cold_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    // RSS baseline is taken AFTER prepare + the first frame, so the reported
    // growth measures the HOT LOOP only (a leak), not the one-time prepare
    // capacity reservation.
    s.rss_start_kb = rss_kb();
    s.rss_peak_kb = s.rss_start_kb;

    std::vector<double> us;
    us.reserve(iters);
    for (size_t i = 0; i < iters; ++i) {
        auto a = clock::now();
        twr::M2aResult o;
        const bool r = ctx.run(frame, prof, o, why) && o.ok;
        auto b = clock::now();
        us.push_back(std::chrono::duration<double, std::micro>(b - a).count());
        if (!r) {
            std::cerr << "hot frame " << i << " failed: " << why << "\n";
            return s;
        }
        if ((i % 100) == 0)
            s.rss_peak_kb = std::max(s.rss_peak_kb, rss_kb());
    }
    s.rss_end_kb = rss_kb();
    s.rss_peak_kb = std::max(s.rss_peak_kb, s.rss_end_kb);
    std::sort(us.begin(), us.end());
    s.iters = us.size();
    s.p50_us = percentile_sorted(us, 50.0);
    s.p95_us = percentile_sorted(us, 95.0);
    s.p99_us = percentile_sorted(us, 99.0);
    s.max_us = us.empty() ? 0.0 : us.back();
    ok = true;
    return s;
}

// ===========================================================================
// CSV
// ===========================================================================

std::ostream* g_csv = &std::cout;

void csv_header()
{
    *g_csv << "section,group,rate,frame,format,variant,iters,cold_us,"
              "p50_us,p95_us,p99_us,max_us,rss_start_kb,rss_end_kb,rss_peak_kb,"
              "alloc_prepare,alloc_first_frame,alloc_hot_total,"
              "alloc_hot_per_iter,alloc_new,alloc_newarr,alloc_aligned,"
              "alloc_malloc,alloc_calloc,alloc_realloc\n";
}

void csv_row(const std::string& section, const std::string& group,
             const std::string& rate, const std::string& frame,
             const std::string& format, const std::string& variant,
             const TimingStats& t, const AllocBreakdown& a)
{
    *g_csv << section << ',' << group << ',' << rate << ',' << frame << ','
           << format << ',' << variant << ',' << t.iters << ',' << t.cold_us
           << ',' << t.p50_us << ',' << t.p95_us << ',' << t.p99_us << ','
           << t.max_us << ',' << t.rss_start_kb << ',' << t.rss_end_kb << ','
           << t.rss_peak_kb << ',' << a.prepare.total() << ','
           << a.first_frame.total() << ',' << a.hot_total.total() << ','
           << (a.hot_iters ? static_cast<double>(a.hot_total.total()) /
                                 static_cast<double>(a.hot_iters)
                           : 0.0)
           << ',' << a.hot_total.new_calls << ',' << a.hot_total.newarr_calls
           << ',' << a.hot_total.aligned_calls << ',' << a.hot_total.malloc_calls
           << ',' << a.hot_total.calloc_calls << ',' << a.hot_total.realloc_calls
           << '\n';
}

// ===========================================================================
// Long-search / failure path (PUBLIC m2a_demod_work stage, not the context)
// ===========================================================================

struct StageObservation {
    TimingStats timing;
    AllocCounts alloc_total;
    size_t iters = 0;
};

// Band-limited work waveform with a leading blank, built through the PUBLIC
// one-shot stages (never a private pipeline).
std::vector<gr_complex> bandlimited_with_blank(const twr::M2aConfig& cfg,
                                               size_t blank, std::string& why)
{
    twr::Frame f = make_frame(twr::FrameType::Poll, 0);
    const twr::FrameProfile prof;
    std::vector<uint8_t> mac;
    if (!twr::encode(f, prof, mac, why))
        return {};
    twr::M2aStageTrace tr;
    std::vector<gr_complex> w, nat, rwx;
    if (!twr::m2a_modulate_to_work(mac.data(), mac.size(), cfg, w, tr, why))
        return {};
    if (!twr::m2a_tx_resample(w.data(), w.size(), cfg, nat, tr, why))
        return {};
    if (!twr::m2a_rx_resample(nat.data(), nat.size(), cfg, rwx, tr, why))
        return {};
    std::vector<gr_complex> buf(blank, gr_complex(0.0f, 0.0f));
    buf.insert(buf.end(), rwx.begin(), rwx.end());
    return buf;
}

StageObservation observe_stage(const std::vector<gr_complex>& buf,
                               const twr::M2aConfig& cfg, size_t iters,
                               bool expect_success)
{
    using clock = std::chrono::steady_clock;
    StageObservation obs;
    std::string why;
    // Allocation pass.
    set_counting(true);
    reset_counts();
    for (size_t i = 0; i < iters; ++i) {
        gr::uwb::demod::DemodResult res;
        twr::M2aStageTrace tr;
        twr::m2a_demod_work(buf.data(), buf.size(), cfg, res, tr, why);
    }
    obs.alloc_total = read_counts();
    set_counting(false);

    // Timing pass.
    TimingStats s;
    s.rss_start_kb = rss_kb();
    s.rss_peak_kb = s.rss_start_kb;
    std::vector<double> us;
    us.reserve(iters);
    for (size_t i = 0; i < iters; ++i) {
        gr::uwb::demod::DemodResult res;
        twr::M2aStageTrace tr;
        auto a = clock::now();
        twr::m2a_demod_work(buf.data(), buf.size(), cfg, res, tr, why);
        auto b = clock::now();
        us.push_back(std::chrono::duration<double, std::micro>(b - a).count());
        if ((i % 100) == 0)
            s.rss_peak_kb = std::max(s.rss_peak_kb, rss_kb());
    }
    s.rss_end_kb = rss_kb();
    s.rss_peak_kb = std::max(s.rss_peak_kb, s.rss_end_kb);
    std::sort(us.begin(), us.end());
    s.iters = us.size();
    s.p50_us = percentile_sorted(us, 50.0);
    s.p95_us = percentile_sorted(us, 95.0);
    s.p99_us = percentile_sorted(us, 99.0);
    s.max_us = us.empty() ? 0.0 : us.back();
    obs.timing = s;
    obs.iters = iters;
    (void)expect_success;
    return obs;
}

} // namespace

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv)
{
    size_t iters = 1000;
    size_t stage_iters = 1000;
    std::string csv_path;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--iters" && i + 1 < argc)
            iters = static_cast<size_t>(std::strtoul(argv[++i], nullptr, 10));
        else if (a == "--stage-iters" && i + 1 < argc)
            stage_iters = static_cast<size_t>(std::strtoul(argv[++i], nullptr, 10));
        else if (a == "--csv" && i + 1 < argc)
            csv_path = argv[++i];
        else if (a == "--help") {
            std::cout << "usage: twr_m2a_bench [--iters N] [--stage-iters N] "
                         "[--csv PATH]\n"
                         "  counts global new/new[]/aligned new + "
                         "malloc/calloc/realloc; CSV to stdout by default\n";
            return 0;
        } else {
            std::cerr << "unknown argument: " << a << "\n";
            return 2;
        }
    }

    std::ofstream csv_file;
    if (!csv_path.empty()) {
        csv_file.open(csv_path);
        if (!csv_file) {
            std::cerr << "cannot open --csv " << csv_path << "\n";
            return 2;
        }
        g_csv = &csv_file;
    }

    const twr::FrameProfile prof;
    const twr::M2aNativeRate rates[] = { twr::M2aNativeRate::Uc200_737280000,
                                         twr::M2aNativeRate::Cg400_491520000 };
    const twr::FrameType frames[] = { twr::FrameType::Poll,
                                      twr::FrameType::Response,
                                      twr::FrameType::Final };
    const twr::M2aIqFormat formats[] = { twr::M2aIqFormat::Cf32,
                                         twr::M2aIqFormat::Sc16 };

    csv_header();

    // ---- matrix: 2 rates x 3 frames x 2 formats x 2 variants --------------
    for (auto rate : rates) {
        for (auto ft : frames) {
            for (auto fmt : formats) {
                const twr::M2aConfig cfg = make_cfg(rate, fmt);
                for (int variant = 0; variant < 2; ++variant) {
                    const twr::Frame f = make_frame(ft, variant);
                    const std::string rname = twr::m2a_native_rate_to_string(rate);
                    const std::string fname = twr::frame_type_to_string(ft);
                    const std::string fmtname = twr::m2a_iq_format_to_string(fmt);
                    const std::string vname = variant == 0 ? "repeat" : "changed";
                    const std::string group = rname + "/" + fname + "/" + fmtname +
                                              "/" + vname;
                    std::cerr << "[matrix] " << group << " ...\n";

                    twr::M2aContext actx;
                    bool aok = false;
                    AllocBreakdown ab =
                        measure_alloc(actx, f, prof, cfg, iters, aok);
                    if (!aok) {
                        std::cerr << "  allocation pass failed\n";
                        continue;
                    }
                    twr::M2aContext tctx;
                    bool tok = false;
                    TimingStats ts =
                        measure_timing(tctx, f, prof, cfg, iters, tok);
                    if (!tok) {
                        std::cerr << "  timing pass failed\n";
                        continue;
                    }
                    csv_row("matrix", group, rname, fname, fmtname, vname, ts, ab);
                }
            }
        }
    }

    // ---- cold wrapper vs prepared (one representative group) --------------
    {
        const twr::M2aNativeRate rate = twr::M2aNativeRate::Uc200_737280000;
        const twr::Frame f = make_frame(twr::FrameType::Final, 0);
        const twr::M2aConfig cfg = make_cfg(rate, twr::M2aIqFormat::Cf32);
        const std::string rname = twr::m2a_native_rate_to_string(rate);
        const std::string group = rname + "/final/cf32/cold-vs-prepared";
        std::cerr << "[compare] " << group << " ...\n";

        // Prepared path (timing + allocation).
        twr::M2aContext pctx;
        bool aok = false;
        AllocBreakdown ab = measure_alloc(pctx, f, prof, cfg, iters, aok);
        twr::M2aContext tctx;
        bool tok = false;
        TimingStats pts = measure_timing(tctx, f, prof, cfg, iters, tok);
        if (aok && tok)
            csv_row("prepared", group, rname, "final", "cf32", "repeat", pts, ab);

        // Cold wrapper path: `m2a_native_roundtrip` prepares + runs one frame.
        using clock = std::chrono::steady_clock;
        set_counting(true);
        reset_counts();
        for (size_t i = 0; i < iters; ++i) {
            twr::M2aResult out;
            std::string why;
            twr::m2a_native_roundtrip(f, prof, cfg, out, why);
        }
        AllocCounts cold_alloc = read_counts();
        set_counting(false);
        AllocBreakdown cab;
        cab.hot_total = cold_alloc;
        cab.hot_iters = iters;

        TimingStats cts;
        std::vector<double> us;
        us.reserve(iters);
        cts.rss_start_kb = rss_kb();
        cts.rss_peak_kb = cts.rss_start_kb;
        for (size_t i = 0; i < iters; ++i) {
            twr::M2aResult out;
            std::string why;
            auto a = clock::now();
            twr::m2a_native_roundtrip(f, prof, cfg, out, why);
            auto b = clock::now();
            us.push_back(std::chrono::duration<double, std::micro>(b - a).count());
            if ((i % 100) == 0)
                cts.rss_peak_kb = std::max(cts.rss_peak_kb, rss_kb());
        }
        cts.rss_end_kb = rss_kb();
        cts.rss_peak_kb = std::max(cts.rss_peak_kb, cts.rss_end_kb);
        std::sort(us.begin(), us.end());
        cts.iters = us.size();
        cts.p50_us = percentile_sorted(us, 50.0);
        cts.p95_us = percentile_sorted(us, 95.0);
        cts.p99_us = percentile_sorted(us, 99.0);
        cts.max_us = us.empty() ? 0.0 : us.back();
        csv_row("cold_wrapper", group, rname, "final", "cf32", "repeat", cts, cab);
    }

    // ---- long-search and failure paths (PUBLIC stage) ---------------------
    {
        const twr::M2aConfig cfg =
            make_cfg(twr::M2aNativeRate::Uc200_737280000, twr::M2aIqFormat::Cf32);
        std::string why;
        const size_t blank = 8192;
        std::vector<gr_complex> longbuf = bandlimited_with_blank(cfg, blank, why);
        if (longbuf.empty()) {
            std::cerr << "long-search buffer build failed: " << why << "\n";
        } else {
            std::cerr << "[stage] long_search blank=" << blank << " ...\n";
            StageObservation o = observe_stage(longbuf, cfg, stage_iters, true);
            AllocBreakdown ab;
            ab.hot_total = o.alloc_total;
            ab.hot_iters = o.iters;
            csv_row("long_search", "uc200/poll/cf32/blank8192",
                    twr::m2a_native_rate_to_string(twr::M2aNativeRate::Uc200_737280000),
                    "poll", "cf32", "repeat", o.timing, ab);
        }

        // Failure path: a noise-only window of comparable length.
        const size_t nfail = longbuf.empty() ? 131072 : longbuf.size();
        std::vector<gr_complex> noise(nfail);
        uint32_t s = 0x9E3779B9u;
        for (size_t i = 0; i < nfail; ++i) {
            s = s * 1664525u + 1013904223u;
            const float re = (static_cast<float>((s >> 8) & 0xffff) / 32768.0f) - 1.0f;
            s = s * 1664525u + 1013904223u;
            const float im = (static_cast<float>((s >> 8) & 0xffff) / 32768.0f) - 1.0f;
            noise[i] = gr_complex(0.3f * re, 0.3f * im);
        }
        std::cerr << "[stage] failure/no-packet n=" << nfail << " ...\n";
        StageObservation o = observe_stage(noise, cfg, stage_iters, false);
        AllocBreakdown ab;
        ab.hot_total = o.alloc_total;
        ab.hot_iters = o.iters;
        csv_row("failure", "uc200/nopacket/cf32",
                twr::m2a_native_rate_to_string(twr::M2aNativeRate::Uc200_737280000),
                "nopacket", "cf32", "repeat", o.timing, ab);
    }

    if (csv_file.is_open())
        csv_file.flush();
    return 0;
}
