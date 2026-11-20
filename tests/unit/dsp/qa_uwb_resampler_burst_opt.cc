/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA: rational resampler burst-mode optimisations, round 2 (2026-11-11).
 *
 * Phase A1 — lazy multi-worker schedule tables.
 * Phase C  — kernel candidates C1 (fixed-phase schedule) and C2 (conversion
 *            cache), both explicit switches and both default OFF.
 *
 * The constructor of RationalResamplerLmCore used to do
 *     sched_arm_buf_.resize(1 << 20);   // 1 MiB (vector<uint8_t>)
 *     sched_win_buf_.resize(1 << 20);   // 4 MiB (vector<uint32_t>)
 * unconditionally, i.e. 5 MiB of zero-initialisation per instance.  Those two
 * tables are read ONLY by fir_parallel()/fir_parallel_sc16(), which are
 * reached only from run_mt()/run_mt_sc16(), i.e. only when the effective
 * worker count is > 1.  Burst workers force set_num_workers(1), so every DSP
 * worker paid for tables it never touched.
 *
 * What this file proves (explicit capacity counters, never RSS):
 *
 *   1. construct + full process_sc16/flush/reset cycles over 256..4096 sample
 *      windows leave schedule_scratch_allocated() == false, for both 65/48 and
 *      65/32 (and the FC32 process() path as well);
 *   2. the FIRST true multi-worker call sizes the tables (entries > 0,
 *      bytes == entries * 5) and its output is BIT-identical, sample by
 *      sample on real and imag, to the single-core run of the same input;
 *   3. repeated MT calls neither reallocate nor change
 *      schedule_scratch_entries(), and stay bit-identical;
 *   4. single -> multi -> single -> multi worker toggling keeps working and
 *      stays bit-identical to the matching single-core reference;
 *   5. short input (ninput < H), tiny max_out (many process calls) and
 *      3-chunk streaming reproduce the one-shot output exactly, for both
 *      realtime_minorder and quality_minorder taps of both rates;
 *   6. the 48/65 TX-direction instantiation (same template, Decim > Interp)
 *      shows the same lazy behaviour and the same bit-exact MT/serial match;
 *   7. the pre-existing contract is untouched: expected_output_length, the
 *      produced/consumed counts, and reset/flush semantics.
 *
 * Phase C adds the candidate gate (see the big comment above the candidate
 * helpers): default vs C1 must be BIT-identical (C2/both removed 2026-11-15),
 * plus the C1
 * descriptor table is recomputed here from first principles for every legal
 * starting phase and compared field by field.
 *
 * Taps come from the real files under testdata/; nothing here synthesises a
 * filter, so a wrong taps path fails loudly instead of silently degrading
 * into a pass.
 */

#define BOOST_TEST_MODULE qa_uwb_resampler_burst_opt
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_resample_window.h>

#include <algorithm>
#include <complex>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using openuwb::core::RationalResampler48_65Core;
using openuwb::core::RationalResampler65_32Core;
using openuwb::core::RationalResampler65_48Core;
using gr_complex = std::complex<float>;
using sc16 = std::complex<int16_t>;

namespace {

// Sc16ScalePolicy::UnitRange and RawInteger, the two production scalings.
constexpr float kUnitScale = 1.0f / 32768.0f;
constexpr float kRawScale = 1.0f;

// Bytes held per schedule-scratch entry: 1 B arm + 4 B window offset.
constexpr size_t kScratchBytesPerEntry = sizeof(uint8_t) + sizeof(uint32_t);

// --------------------------------------------------------------- test data --

/** Resolve a path relative to testdata/.  Empty result => not found. */
std::string find_testdata(const std::string& rel)
{
    std::vector<std::string> cands;
#ifdef UWB_TESTDATA_DIR
    cands.push_back(std::string(UWB_TESTDATA_DIR) + "/" + rel);
#endif
    // CTest CWD is the build dir (gr-uwb/build-*); a hand run is usually one
    // level deeper (gr-uwb/build-*/lib).
    const char* prefixes[] = { "", "../", "../../", "../../../", "../../../../" };
    for (const char* p : prefixes)
        cands.push_back(std::string(p) + "testdata/" + rel);
    for (const auto& c : cands) {
        std::ifstream f(c, std::ios::binary);
        if (f)
            return c;
    }
    return std::string();
}

std::vector<float> load_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    BOOST_REQUIRE(f);
    return v;
}

/** Load (and cache) a real taps file; a missing file fails the test. */
const std::vector<float>& real_taps(const std::string& rel)
{
    static std::map<std::string, std::vector<float>> cache;
    const auto it = cache.find(rel);
    if (it != cache.end())
        return it->second;
    const std::string path = find_testdata(rel);
    BOOST_REQUIRE_MESSAGE(
        !path.empty(),
        "cannot locate testdata/" + rel +
            " (set UWB_TESTDATA_DIR or run from the build tree)");
    auto ins = cache.emplace(rel, load_f32(path));
    BOOST_TEST_MESSAGE("taps[" << rel << "] T=" << ins.first->second.size()
                               << " from " << path);
    return ins.first->second;
}

const std::vector<float>& taps_65_48_realtime()
{
    return real_taps("resampler_65_48/taps_realtime_minorder.txt");
}
const std::vector<float>& taps_65_48_quality()
{
    return real_taps("resampler_65_48/taps_quality_minorder.txt");
}
const std::vector<float>& taps_65_32_realtime()
{
    return real_taps("resampler_65_32/taps_realtime_minorder.txt");
}
const std::vector<float>& taps_65_32_quality()
{
    return real_taps("resampler_65_32/taps_quality_minorder.txt");
}
const std::vector<float>& taps_48_65_tx()
{
    return real_taps("twr/m2a/taps/tx_48_65.f32");
}

// ------------------------------------------------------------- input data --

/**
 * Deterministic SC16 stimulus: LCG noise plus pinned extremes (both rails,
 * zero) so the scale and tail paths also see saturated and zero samples.
 */
std::vector<sc16> make_sc16_input(size_t n, uint32_t seed)
{
    std::vector<sc16> v(n);
    uint32_t s = seed | 1u;
    for (size_t i = 0; i < n; ++i) {
        s = s * 1664525u + 1013904223u;
        const int32_t re = static_cast<int32_t>(s >> 16) - 32768;
        s = s * 1664525u + 1013904223u;
        const int32_t im = static_cast<int32_t>(s >> 16) - 32768;
        v[i] = sc16(static_cast<int16_t>(re), static_cast<int16_t>(im));
    }
    if (n > 8) {
        v[0] = sc16(32767, -32768);
        v[1] = sc16(-32768, 32767);
        v[2] = sc16(0, 0);
        v[n - 3] = sc16(32767, 32767);
        v[n - 2] = sc16(-32768, -32768);
        v[n - 1] = sc16(0, 0);
    }
    return v;
}

std::vector<gr_complex> to_cf32(const std::vector<sc16>& in, float scale)
{
    std::vector<gr_complex> v(in.size());
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = gr_complex(static_cast<float>(in[i].real()) * scale,
                          static_cast<float>(in[i].imag()) * scale);
    return v;
}

// -------------------------------------------------------------- runners ----

/**
 * One process call with a generous output grant plus the full flush tail.
 * `nworkers_override` != 0 forces the multi-worker path for this call only
 * (that is the only way to reach run_mt() while nworkers_ stays 1).
 */
template <typename Core>
std::vector<gr_complex> run_oneshot_sc16(Core& core,
                                         const std::vector<sc16>& in,
                                         float scale,
                                         int nworkers_override = 0)
{
    const size_t expect =
        Core::expected_output_length(in.size(), core.tap_count());
    std::vector<gr_complex> out(expect + 8192, gr_complex(0.0f, 0.0f));
    size_t produced = 0;
    if (!in.empty()) {
        auto r = core.process_sc16(in.data(), in.size(), scale, out.data(),
                                   out.size(), nworkers_override);
        BOOST_REQUIRE_EQUAL(r.consumed, in.size());
        produced = r.produced;
    }
    while (true) {
        if (produced >= out.size())
            out.resize(produced + 8192);
        const size_t n =
            core.flush(out.data() + produced, out.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    out.resize(produced);
    return out;
}

/** Same, for the FC32 process() path. */
template <typename Core>
std::vector<gr_complex> run_oneshot_cf32(Core& core,
                                         const std::vector<gr_complex>& in,
                                         int nworkers_override = 0)
{
    const size_t expect =
        Core::expected_output_length(in.size(), core.tap_count());
    std::vector<gr_complex> out(expect + 8192, gr_complex(0.0f, 0.0f));
    size_t produced = 0;
    if (!in.empty()) {
        auto r = core.process(in.data(), in.size(), out.data(), out.size(),
                              nworkers_override);
        BOOST_REQUIRE_EQUAL(r.consumed, in.size());
        produced = r.produced;
    }
    while (true) {
        if (produced >= out.size())
            out.resize(produced + 8192);
        const size_t n =
            core.flush(out.data() + produced, out.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    out.resize(produced);
    return out;
}

struct StreamResult {
    std::vector<gr_complex> out;
    size_t process_calls = 0;
    size_t flush_calls = 0;
};

/**
 * Stream `in` through SC16 in `chunks` (empty => one chunk) with a per-call
 * output cap of `max_out`, then drain the tail with the same cap.  Proves that
 * small grants plus many calls reproduce the one-shot result exactly.
 */
template <typename Core>
StreamResult run_stream_sc16(Core& core,
                             const std::vector<sc16>& in,
                             float scale,
                             size_t max_out,
                             const std::vector<size_t>& chunks,
                             int nworkers_override = 0)
{
    StreamResult res;
    std::vector<gr_complex> buf(std::max<size_t>(max_out, 1));
    size_t off = 0;
    size_t ci = 0;
    while (off < in.size()) {
        size_t cur = (ci < chunks.size())
                         ? std::min(chunks[ci], in.size() - off)
                         : (in.size() - off);
        ++ci;
        while (cur > 0) {
            auto r = core.process_sc16(in.data() + off, cur, scale, buf.data(),
                                       buf.size(), nworkers_override);
            ++res.process_calls;
            res.out.insert(res.out.end(), buf.begin(),
                           buf.begin() + r.produced);
            off += r.consumed;
            cur -= r.consumed;
            BOOST_REQUIRE_MESSAGE(
                r.produced > 0 || r.consumed > 0,
                "process_sc16 made no progress: produced=" +
                    std::to_string(r.produced) +
                    " consumed=" + std::to_string(r.consumed) +
                    " remaining=" + std::to_string(cur) +
                    " max_out=" + std::to_string(buf.size()));
        }
    }
    while (true) {
        const size_t n = core.flush(buf.data(), buf.size());
        ++res.flush_calls;
        res.out.insert(res.out.end(), buf.begin(), buf.begin() + n);
        if (n == 0)
            break;
    }
    return res;
}

// ------------------------------------------------------------ assertions ---

/**
 * Bit-identical comparison: per sample on real AND imag (BOOST_CHECK_EQUAL,
 * as the round-2 gate requires) plus one whole-vector memcmp that also
 * catches -0.0 vs +0.0, which float == hides.
 */
void require_bit_identical(const std::vector<gr_complex>& got,
                           const std::vector<gr_complex>& ref,
                           const std::string& what)
{
    BOOST_REQUIRE_MESSAGE(got.size() == ref.size(),
                          what + ": length " + std::to_string(got.size()) +
                              " != reference " + std::to_string(ref.size()));
    for (size_t i = 0; i < got.size(); ++i) {
        BOOST_CHECK_EQUAL(got[i].real(), ref[i].real());
        BOOST_CHECK_EQUAL(got[i].imag(), ref[i].imag());
    }
    BOOST_CHECK_MESSAGE(
        got.empty() ||
            std::memcmp(got.data(), ref.data(),
                        got.size() * sizeof(gr_complex)) == 0,
        what + ": bit pattern differs (memcmp) although float == held");
}

/** A fresh single-core instance must own no schedule scratch at all. */
template <typename Core>
void require_single_core_unallocated(const Core& core, const std::string& what)
{
    BOOST_CHECK_MESSAGE(!core.schedule_scratch_allocated(),
                        what + ": schedule tables allocated on a single-core "
                               "instance (entries=" +
                            std::to_string(core.schedule_scratch_entries()) +
                            ")");
    BOOST_CHECK_EQUAL(core.schedule_scratch_entries(), 0u);
    BOOST_CHECK_EQUAL(core.schedule_scratch_bytes(), 0u);
}

/** Capacity must be one coherent allocation of kScratchBytesPerEntry/entry. */
void require_scratch_geometry(size_t entries, size_t bytes,
                              const std::string& what)
{
    BOOST_CHECK_GT(entries, 0u);
    BOOST_CHECK_EQUAL(bytes, entries * kScratchBytesPerEntry);
    // The old unconditional 5 MiB (1 Mi entries) is what must be gone from the
    // single-core path; the lazy table must stay far below it.
    BOOST_CHECK_LT(bytes, (1u << 20) * kScratchBytesPerEntry);
}

/** Generous flush-room for one drain call. */
template <typename Core>
size_t tail_capacity(const Core& core)
{
    return 64 * core.arm_length() + 4096;
}

// ------------------------------------------------------------ test bodies --

/** 1. Construction with nworkers == 1 allocates nothing (65/48 and 65/32). */
template <typename Core>
void check_construction(const std::vector<float>& taps, const std::string& tag)
{
    Core core(taps);
    BOOST_CHECK_EQUAL(core.num_workers(), 1);
    require_single_core_unallocated(core, tag + " ctor");

    // set_num_workers(1) is the burst-worker setting; it must not allocate.
    core.set_num_workers(1);
    require_single_core_unallocated(core, tag + " set_num_workers(1)");

    // Nor must anything a single-core caller normally does.
    auto in = make_sc16_input(1024, 0x9e3779b9u);
    auto y = run_oneshot_sc16(core, in, kUnitScale);
    BOOST_CHECK_EQUAL(
        y.size(), Core::expected_output_length(in.size(), taps.size()));
    require_single_core_unallocated(core, tag + " after single-core run");
}

/** 2. A full process_sc16 + flush + reset cycle over 256..4096 windows. */
template <typename Core>
void check_full_cycle(const std::vector<float>& taps, const std::string& tag)
{
    const std::vector<size_t> sizes = { 256, 512, 1024, 2048, 4096 };
    for (size_t n : sizes) {
        Core core(taps);
        auto in = make_sc16_input(n, 0x1234u + static_cast<uint32_t>(n));
        const size_t expect = Core::expected_output_length(n, taps.size());
        for (int cycle = 0; cycle < 3; ++cycle) {
            auto y = run_oneshot_sc16(core, in, kUnitScale);
            BOOST_CHECK_EQUAL(y.size(), expect);
            BOOST_CHECK_EQUAL(core.input_items(), static_cast<uint64_t>(n));
            BOOST_CHECK_EQUAL(core.output_items(),
                              static_cast<uint64_t>(expect));
            BOOST_CHECK(core.flush_complete());
            require_single_core_unallocated(
                core, tag + " n=" + std::to_string(n) + " cycle=" +
                          std::to_string(cycle));
            core.reset();
            BOOST_CHECK_EQUAL(core.input_items(), 0u);
            BOOST_CHECK_EQUAL(core.output_items(), 0u);
            BOOST_CHECK_EQUAL(core.phase(), 0u);
        }
        BOOST_CHECK_EQUAL(core.resets(), 3u);
    }
}

/** 3. The FC32 process() path is equally lazy and equally bit-exact. */
template <typename Core>
void check_fc32_path(const std::vector<float>& taps, const std::string& tag)
{
    auto in = to_cf32(make_sc16_input(4096, 0xdeadbeefu), kUnitScale);

    Core ref(taps);
    auto y_ref = run_oneshot_cf32(ref, in);
    require_single_core_unallocated(ref, tag + " FC32 single-core");

    Core mt(taps);
    require_single_core_unallocated(mt, tag + " FC32 before MT");
    auto y_mt = run_oneshot_cf32(mt, in, /*nworkers_override=*/4);
    BOOST_CHECK_MESSAGE(mt.schedule_scratch_allocated(),
                        tag + " FC32 MT did not size the schedules");
    require_scratch_geometry(mt.schedule_scratch_entries(),
                             mt.schedule_scratch_bytes(), tag + " FC32 MT");
    require_bit_identical(y_mt, y_ref, tag + " FC32 MT(4) vs single-core");
}

/** 4. First MT call: allocates, and matches the serial run bit for bit. */
template <typename Core>
void check_first_mt_call(const std::vector<float>& taps,
                         const std::string& tag)
{
    const size_t N = 4096;
    auto in = make_sc16_input(N, 0x55aa33ccu);

    for (float scale : { kUnitScale, kRawScale }) {
        const std::string st = (scale == kRawScale) ? "raw" : "unit";

        Core ref(taps);
        auto y_ref = run_oneshot_sc16(ref, in, scale);
        require_single_core_unallocated(ref, tag + " " + st + " reference");

        Core mt(taps);
        BOOST_CHECK_EQUAL(mt.schedule_scratch_entries(), 0u);
        auto y_mt = run_oneshot_sc16(mt, in, scale, /*nworkers_override=*/4);

        BOOST_CHECK_MESSAGE(mt.schedule_scratch_allocated(),
                            tag + " " + st +
                                ": first MT call did not size the schedules");
        const size_t entries = mt.schedule_scratch_entries();
        const size_t bytes = mt.schedule_scratch_bytes();
        require_scratch_geometry(entries, bytes, tag + " " + st + " first MT");
        BOOST_TEST_MESSAGE(tag << " " << st << ": entries 0 -> " << entries
                               << ", bytes 0 -> " << bytes << " ("
                               << (bytes / 1024) << " KiB)");

        require_bit_identical(y_mt, y_ref,
                              tag + " " + st + " MT(4) vs single-core");
        BOOST_CHECK_EQUAL(mt.input_items(), ref.input_items());
        BOOST_CHECK_EQUAL(mt.output_items(), ref.output_items());
        BOOST_CHECK_EQUAL(mt.phase(), ref.phase());
        BOOST_CHECK(mt.flush_complete());
    }
}

/** 5. Repeated MT calls reuse the same capacity and stay bit-identical. */
template <typename Core>
void check_mt_reuse(const std::vector<float>& taps, const std::string& tag)
{
    auto in = make_sc16_input(2048, 0x0badf00du);

    Core ref(taps);
    auto y1_ref = run_oneshot_sc16(ref, in, kUnitScale);

    Core mt(taps);
    auto y1 = run_oneshot_sc16(mt, in, kUnitScale, 4);
    const size_t entries1 = mt.schedule_scratch_entries();
    const size_t bytes1 = mt.schedule_scratch_bytes();
    require_scratch_geometry(entries1, bytes1, tag + " MT run 1");
    require_bit_identical(y1, y1_ref, tag + " MT run 1");

    // Second MT run: capacity unchanged (reset() clears stream state only; it
    // must never touch -- let alone shrink -- the schedule tables).
    mt.reset();
    auto y2 = run_oneshot_sc16(mt, in, kUnitScale, 4);
    BOOST_CHECK_EQUAL(mt.schedule_scratch_entries(), entries1);
    BOOST_CHECK_EQUAL(mt.schedule_scratch_bytes(), bytes1);
    BOOST_CHECK_EQUAL(y2.size(), y1.size());
    require_bit_identical(y2, y1, tag + " MT run 2 (after reset) vs run 1");

    // Third MT run after two more reset/flush cycles: same capacity, same bits.
    for (int k = 0; k < 3; ++k) {
        mt.reset();
        auto yk = run_oneshot_sc16(mt, in, kUnitScale, 4);
        require_bit_identical(
            yk, y1, tag + " MT repeat cycle " + std::to_string(k + 3));
        BOOST_CHECK_EQUAL(mt.schedule_scratch_entries(), entries1);
    }

    // Many MT calls in a row: capacity is stable, outputs stay identical.
    Core mt_many(taps);
    std::vector<gr_complex> many;
    for (int k = 0; k < 8; ++k) {
        auto yk = run_oneshot_sc16(mt_many, in, kUnitScale, 4);
        many.insert(many.end(), yk.begin(), yk.end());
        BOOST_CHECK_EQUAL(mt_many.schedule_scratch_entries(), entries1);
        mt_many.reset();
    }
    std::vector<gr_complex> expect8;
    expect8.reserve(y1.size() * 8);
    for (int k = 0; k < 8; ++k)
        expect8.insert(expect8.end(), y1.begin(), y1.end());
    require_bit_identical(many, expect8, tag + " 8 consecutive MT calls");
}

/** 6. single -> multi -> single -> multi toggling. */
template <typename Core>
void check_worker_toggle(const std::vector<float>& taps,
                         const std::string& tag)
{
    const size_t N = 8192;
    auto in = make_sc16_input(N, 0x31415926u);

    // Chunks deliberately not multiples of 48/65/32 so phase carries over.
    const std::vector<size_t> chunks = { 1024, 2048, 1500,
                                         N - 1024 - 2048 - 1500 };
    const std::vector<int> workers = { 1, 4, 1, 4 };

    Core ref(taps); // stays single-core for the whole stream
    Core tog(taps);

    std::vector<gr_complex> y_ref;
    std::vector<gr_complex> y_tog;
    size_t entries_after_first_mt = 0;
    size_t off = 0;
    for (size_t ci = 0; ci < chunks.size(); ++ci) {
        const size_t n = std::min(chunks[ci], N - off);
        BOOST_REQUIRE_GT(n, 0u);

        const int w = workers[ci % workers.size()];
        const bool had_mt_before = (entries_after_first_mt != 0);
        if (!had_mt_before)
            require_single_core_unallocated(
                tog, tag + " toggle chunk " + std::to_string(ci) +
                         " before any MT call");

        tog.set_num_workers(w);
        BOOST_CHECK_EQUAL(tog.num_workers(), w);
        BOOST_CHECK_EQUAL(ref.num_workers(), 1);

        const size_t cap =
            (n * Core::kInterp) / Core::kDecim + 4 * tog.arm_length() + 256;
        std::vector<gr_complex> buf_r(cap);
        std::vector<gr_complex> buf_t(cap);

        auto rr = ref.process_sc16(in.data() + off, n, kUnitScale, buf_r.data(),
                                   buf_r.size());
        BOOST_REQUIRE_EQUAL(rr.consumed, n);
        y_ref.insert(y_ref.end(), buf_r.begin(), buf_r.begin() + rr.produced);

        auto rt = tog.process_sc16(in.data() + off, n, kUnitScale, buf_t.data(),
                                   buf_t.size());
        BOOST_REQUIRE_EQUAL(rt.consumed, n);
        y_tog.insert(y_tog.end(), buf_t.begin(), buf_t.begin() + rt.produced);

        BOOST_CHECK_LT(tog.phase(), Core::kInterp);
        off += n;

        if (w > 1) {
            // This chunk must have engaged run_mt() and sized the tables.
            BOOST_CHECK_MESSAGE(tog.schedule_scratch_allocated(),
                                tag + " toggle chunk " + std::to_string(ci) +
                                    " w=" + std::to_string(w) +
                                    " did not size the schedules");
            if (entries_after_first_mt == 0)
                entries_after_first_mt = tog.schedule_scratch_entries();
            BOOST_CHECK_EQUAL(tog.schedule_scratch_entries(),
                              entries_after_first_mt);
        } else if (had_mt_before) {
            // Back to one worker: capacity is retained, never shrunk.
            BOOST_CHECK_EQUAL(tog.schedule_scratch_entries(),
                              entries_after_first_mt);
        } else {
            require_single_core_unallocated(
                tog, tag + " toggle chunk " + std::to_string(ci) + " w=1");
        }
    }
    BOOST_CHECK_GT(entries_after_first_mt, 0u);
    require_scratch_geometry(entries_after_first_mt,
                             entries_after_first_mt * kScratchBytesPerEntry,
                             tag + " toggle");

    // Drain both tails with the same grant.
    std::vector<gr_complex> tail(tail_capacity(tog));
    size_t n_tail = 0;
    while ((n_tail = ref.flush(tail.data(), tail.size())) > 0)
        y_ref.insert(y_ref.end(), tail.begin(), tail.begin() + n_tail);
    n_tail = 0;
    while ((n_tail = tog.flush(tail.data(), tail.size())) > 0)
        y_tog.insert(y_tog.end(), tail.begin(), tail.begin() + n_tail);

    BOOST_CHECK_EQUAL(y_tog.size(),
                      Core::expected_output_length(N, taps.size()));
    require_bit_identical(y_tog, y_ref, tag + " toggle 1-4-1-4");
    BOOST_CHECK_EQUAL(tog.input_items(), ref.input_items());
    BOOST_CHECK_EQUAL(tog.output_items(), ref.output_items());
}

/**
 * 7. Short input (ninput < H), tiny max_out (many calls) and 3-chunk
 *    streaming, all bit-identical to the one-shot run.
 */
template <typename Core>
void check_streaming_equivalence(const std::vector<float>& taps,
                                 const std::string& tag)
{
    // ---- short inputs, including one shorter than the FIR arm ----
    {
        Core probe(taps);
        const size_t H = probe.arm_length();
        const std::vector<size_t> ns = { 1, 3, H / 2, H - 1, H, H + 1, H + 7 };
        for (size_t n : ns) {
            Core ref(taps);
            auto in = make_sc16_input(n, 0x77u + static_cast<uint32_t>(n));
            auto y_ref = run_oneshot_sc16(ref, in, kUnitScale);

            Core chunked(taps);
            auto st = run_stream_sc16(chunked, in, kUnitScale, 4096, { 1 });
            BOOST_CHECK_GT(st.process_calls, 0u);
            require_bit_identical(
                st.out, y_ref,
                tag + " short n=" + std::to_string(n) + " (H=" +
                    std::to_string(H) + ") sample-at-a-time");
            BOOST_CHECK_EQUAL(st.out.size(),
                              Core::expected_output_length(n, taps.size()));
        }
    }

    // ---- tiny max_out: several process calls per window ----
    {
        const size_t N = 4096;
        auto in = make_sc16_input(N, 0xabcdef01u);
        Core ref(taps);
        auto y_ref = run_oneshot_sc16(ref, in, kUnitScale);

        const std::vector<size_t> caps = { 16, 65, 66, 128, 1024 };
        for (size_t cap : caps) {
            Core core(taps);
            auto st = run_stream_sc16(core, in, kUnitScale, cap, {});
            BOOST_CHECK_GT(st.process_calls, 1u);
            BOOST_TEST_MESSAGE(tag << " max_out=" << cap << ": "
                                   << st.process_calls << " process calls, "
                                   << st.flush_calls << " flush calls");
            require_bit_identical(st.out, y_ref,
                                  tag + " tiny max_out=" + std::to_string(cap));

            // A multi-worker grant that is too small to engage run_mt()
            // (max_out < 64) must fall back to the serial schedule, produce
            // the identical stream, and NOT allocate the schedule tables.
            if (cap < 64) {
                Core mt_fallback(taps);
                auto sf = run_stream_sc16(mt_fallback, in, kUnitScale, cap, {},
                                          /*nworkers_override=*/4);
                require_bit_identical(
                    sf.out, y_ref,
                    tag + " tiny max_out=" + std::to_string(cap) + " w=4");
                require_single_core_unallocated(
                    mt_fallback,
                    tag + " max_out=" + std::to_string(cap) +
                        " below the MT entry threshold");
            }
        }
    }

    // ---- 3-chunk streaming equals one-shot, for both scalings ----
    {
        const size_t N = 4000;
        auto in = make_sc16_input(N, 0x5a5a5a5au);
        const size_t c0 = N / 3;
        const size_t c1 = N / 3;
        const size_t c2 = N - c0 - c1;
        for (float scale : { kUnitScale, kRawScale }) {
            const std::string st_name =
                (scale == kRawScale) ? "raw" : "unit";

            Core ref(taps);
            auto y_ref = run_oneshot_sc16(ref, in, scale);

            Core chunked(taps);
            auto st = run_stream_sc16(chunked, in, scale, 4096, { c0, c1, c2 });
            BOOST_CHECK_EQUAL(st.process_calls, 3u);
            require_bit_identical(
                st.out, y_ref, tag + " 3-chunk stream (" + st_name + ")");
            BOOST_CHECK_EQUAL(st.out.size(),
                              Core::expected_output_length(N, taps.size()));
            require_single_core_unallocated(chunked,
                                            tag + " 3-chunk " + st_name);
        }
    }
}

/** 8. The 48/65 TX direction (same template, Decim > Interp). */
void check_48_65()
{
    const auto& taps = taps_48_65_tx();
    using Core = RationalResampler48_65Core;

    Core core(taps);
    BOOST_CHECK_EQUAL(core.num_workers(), 1);
    require_single_core_unallocated(core, "48/65 ctor");

    auto in = make_sc16_input(1024, 0x2468ace0u);
    auto y_single = run_oneshot_sc16(core, in, kUnitScale);
    BOOST_CHECK_EQUAL(y_single.size(),
                      Core::expected_output_length(in.size(), taps.size()));
    require_single_core_unallocated(core, "48/65 single-core run");

    Core mt(taps);
    auto y_mt = run_oneshot_sc16(mt, in, kUnitScale, /*nworkers_override=*/3);
    BOOST_CHECK_MESSAGE(mt.schedule_scratch_allocated(),
                        "48/65 MT did not size the schedules");
    require_scratch_geometry(mt.schedule_scratch_entries(),
                             mt.schedule_scratch_bytes(), "48/65 MT");
    require_bit_identical(y_mt, y_single, "48/65 MT(3) vs single-core");

    // Short input and a tiny grant still match (Decim > Interp resume path).
    Core chunked(taps);
    auto st = run_stream_sc16(chunked, in, kUnitScale, 32, { 7, 100, 917 });
    require_bit_identical(st.out, y_single, "48/65 chunked max_out=32");
    require_single_core_unallocated(chunked, "48/65 chunked single-core");
}

/** 9. Pre-existing length contract, recomputed independently here. */
template <typename Core>
void check_length_contract(const std::vector<float>& taps,
                           const std::string& tag)
{
    const size_t T = taps.size();
    Core core(taps);
    const uint64_t ns[] = { 0,  1,   2,   3,   47,  48,   49,   64,
                            65, 95,  96,  127, 128, 1024, 4000, 4096 };
    size_t prev = 0;
    for (size_t i = 0; i < sizeof(ns) / sizeof(ns[0]); ++i) {
        const uint64_t n = ns[i];
        const int64_t num =
            (static_cast<int64_t>(n) - 1) *
                static_cast<int64_t>(Core::kInterp) +
            static_cast<int64_t>(T);
        const size_t expect =
            (num <= 0) ? 0u
                       : static_cast<size_t>(
                             (num + static_cast<int64_t>(Core::kDecim) - 1) /
                             static_cast<int64_t>(Core::kDecim));
        BOOST_CHECK_EQUAL(Core::expected_output_length(n, T), expect);
        BOOST_CHECK_EQUAL(core.expected_output_length(n), expect);
        if (i > 0)
            BOOST_CHECK_GE(expect, prev);
        prev = expect;
    }
    // N = 0 still yields the filter tail (T >= Interp for every real profile).
    BOOST_CHECK_GT(Core::expected_output_length(0, T), 0u);
    BOOST_CHECK_EQUAL(core.tap_count(), T);
    BOOST_TEST_MESSAGE(tag << " T=" << T << " H=" << core.arm_length()
                           << " len(0)=" << Core::expected_output_length(0, T)
                           << " len(4096)="
                           << Core::expected_output_length(4096, T));
}

// ===========================================================================
//  Phase C (2026-11-11): resampler kernel candidate C1
//
//  C1 = set_fixed_phase_schedule(true)  : per-phase 4-output descriptor table
//       (arm tap offsets + window offsets relative to the macroblock base +
//       precomputed win[j]==win[j-1] reuse mask) and 4-way FIR kernels
//       compile-time specialised for H_pad == 24 / 48, generic otherwise.
//
//  The C2 candidate (set_convert_cache) was measured, NOT retained and
//  REMOVED in the 2026-11-15 native cleanup, together with its "both"
//  combination; this QA keeps the C1 bit-identity gate.
//
//  C1 defaults OFF.  The gate is bit-identity with the DEFAULT path: every
//  real and imag float equal plus a whole-vector memcmp, over
//    * the two switch states (none / C1),
//    * both shipped profiles (H_pad 24 realtime, H_pad 48 quality),
//    * both rates that share the 65/48-style schedule (65/48, 65/32) and the
//      48/65 TX direction (H_pad 64 => the generic fallback),
//    * both scale modes (UnitRange 1/32768, RawInteger 1.0),
//    * pinned full-scale rails (+32767, -32767, -32768), zero, random, a
//      single full-scale impulse, short windows (n < H), tail-only runs and
//      lengths that are not whole multiples of Decim,
//    * chunked streaming, tiny max_out grants, and EVERY legal starting phase
//      of a full macroblock (phase-primed with max_out == 1 calls).
// ===========================================================================

using Variant = int;
constexpr Variant kVariantDefault = 0;
constexpr Variant kVariantC1 = 1;

const char* variant_switch_name(Variant v)
{
    switch (v) {
    case kVariantC1:
        return "c1_fixed_phase";
    default:
        return "default";
    }
}

template <typename Core>
void apply_variant(Core& core, Variant v)
{
    core.set_fixed_phase_schedule(v == kVariantC1);
}

// Samples compared bit-for-bit by the Phase-C gate (reported at the end).
uint64_t g_candidate_samples = 0;
uint64_t g_candidate_comparisons = 0;

/** Bit-identity gate that also counts what it compared. */
void require_bit_identical_counted(const std::vector<gr_complex>& got,
                                   const std::vector<gr_complex>& ref,
                                   const std::string& what)
{
    require_bit_identical(got, ref, what);
    g_candidate_samples += static_cast<uint64_t>(got.size());
    ++g_candidate_comparisons;
}

/** SC16 input patterns: pins the values the gates name explicitly. */
enum class Pattern { Random, Rails, Zero, Impulse };

std::vector<sc16> make_pattern(size_t n, Pattern p, uint32_t seed)
{
    if (p == Pattern::Random)
        return make_sc16_input(n, seed);
    std::vector<sc16> v(n, sc16(0, 0));
    if (n == 0)
        return v;
    switch (p) {
    case Pattern::Rails: {
        // Full-scale positive, both negative rails (incl. the -32767
        // asymmetric one) and zero, interleaved so every FIR window sees all.
        static const int16_t rails[] = { 32767, -32768, -32767, 0 };
        for (size_t i = 0; i < n; ++i)
            v[i] = sc16(rails[i % 4], rails[(i + 2) % 4]);
        break;
    }
    case Pattern::Zero:
        break;
    case Pattern::Impulse:
        v[n / 2] = sc16(32767, -32768);
        break;
    case Pattern::Random:
        break;
    }
    return v;
}

const char* pattern_name(Pattern p)
{
    switch (p) {
    case Pattern::Rails:
        return "rails";
    case Pattern::Zero:
        return "zero";
    case Pattern::Impulse:
        return "impulse";
    default:
        return "random";
    }
}

/**
 * Feed `in` through `core` in one call with output cap `max_out`, collecting
 * the produced samples; never flushes.  Used to prime a phase and then to run
 * the rest of the stream into the macroblock path.
 */
template <typename Core>
std::vector<gr_complex> feed_sc16(Core& core,
                                  const std::vector<sc16>& in,
                                  float scale,
                                  size_t max_out)
{
    std::vector<gr_complex> acc;
    std::vector<gr_complex> buf(std::max<size_t>(max_out, 1));
    size_t off = 0;
    while (off < in.size()) {
        size_t cur = in.size() - off;
        while (cur > 0) {
            auto r = core.process_sc16(in.data() + off, cur, scale, buf.data(),
                                       buf.size());
            acc.insert(acc.end(), buf.begin(), buf.begin() + r.produced);
            off += r.consumed;
            cur -= r.consumed;
            BOOST_REQUIRE_MESSAGE(r.produced > 0 || r.consumed > 0,
                                  "candidate feed made no progress");
        }
    }
    return acc;
}

/** Drain the flush tail with a generous grant. */
template <typename Core>
void drain_sc16(Core& core, std::vector<gr_complex>& acc)
{
    std::vector<gr_complex> buf(tail_capacity(core));
    size_t n = 0;
    while ((n = core.flush(buf.data(), buf.size())) > 0)
        acc.insert(acc.end(), buf.begin(), buf.begin() + n);
}

/**
 * Run one scenario (a callable taking Core&) on the default path and on the
 * three candidate combinations, and require bit-identity.  Also requires that
 * the requested candidate is the one the core actually dispatches to, so a
 * silently-ignored switch cannot pass this gate.
 */
template <typename Core, typename Fn>
void check_scenario(const std::vector<float>& taps,
                    const std::string& tag,
                    Fn&& scenario)
{
#if !UWB_RESAMP_HAVE_AVX2
    // The C1 candidate is an AVX2/FMA-only implementation.  In a SCALAR
    // build the switch is inert by design, so there is no candidate to
    // compare; skip instead of asserting a variant that cannot exist.
    (void)taps;
    (void)tag;
    (void)scenario;
    BOOST_TEST_MESSAGE("SKIP " << tag << ": C1 candidate requires AVX2/FMA");
    return;
#else
    Core ref(taps);
    apply_variant(ref, kVariantDefault);
    BOOST_REQUIRE_EQUAL(std::string(ref.sc16_kernel_variant()), "default");
    const std::vector<gr_complex> y_ref = scenario(ref);

    const Variant variants[] = { kVariantC1 };
    for (Variant v : variants) {
        Core cand(taps);
        apply_variant(cand, v);
        BOOST_REQUIRE_EQUAL(std::string(cand.sc16_kernel_variant()),
                            variant_switch_name(v));
        const std::vector<gr_complex> y = scenario(cand);
        require_bit_identical_counted(y, y_ref,
                                      tag + " [" + variant_switch_name(v) + "]");
    }
#endif
}

/** 10. Defaults, switch semantics and C1 table contents. */
template <typename Core>
void check_candidate_api(const std::vector<float>& taps, const std::string& tag)
{
    Core core(taps);

    // ---- defaults: the C1 candidate is OFF ----
    BOOST_CHECK_EQUAL(core.fixed_phase_schedule(), false);
    BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()), "default");
    BOOST_CHECK_EQUAL(core.fixed_phase_table_built(), false);

    // Disabling is reversible.
    apply_variant(core, kVariantDefault);
    BOOST_CHECK_EQUAL(core.fixed_phase_schedule(), false);
    BOOST_CHECK_EQUAL(std::string(core.sc16_kernel_variant()), "default");

    const size_t H_pad = core.arm_length_pad();

    // ---- C1 table: rebuilt from first principles here ----
    apply_variant(core, kVariantC1);
    BOOST_CHECK(core.fixed_phase_table_built());
    BOOST_CHECK_GE(Core::kInterp, 1u);
    const uint32_t G = Core::macroblock_groups();
    const uint32_t T = Core::macroblock_tail();
    BOOST_CHECK_EQUAL(4 * G + T, Core::kInterp);

    for (uint32_t p = 0; p < Core::kInterp; ++p) {
        uint32_t arm = p;   // (p + o*Decim) mod Interp
        uint32_t win = 0;   // floor((p + o*Decim) / Interp)
        for (uint32_t g = 0; g < G; ++g) {
            const auto& d = core.fixed_phase_group(p, g);
            uint8_t reuse = 0;
            for (uint32_t j = 0; j < 4; ++j) {
                BOOST_REQUIRE_EQUAL(d.win[j], static_cast<uint16_t>(win));
                BOOST_REQUIRE_EQUAL(
                    d.tap[j],
                    static_cast<uint16_t>(static_cast<size_t>(arm) * 2 * H_pad));
                if (j > 0 && d.win[j] == d.win[j - 1])
                    reuse |= static_cast<uint8_t>(1u << j);
                // advance exactly like the production schedule tables
                arm += Core::kDecim;
                while (arm >= Core::kInterp) {
                    arm -= Core::kInterp;
                    ++win;
                }
            }
            BOOST_REQUIRE_EQUAL(d.reuse, reuse);
            BOOST_REQUIRE_EQUAL(d.pad[0], 0);
        }
        for (uint32_t t = 0; t < T; ++t) {
            const auto& d = core.fixed_phase_tail(p, t);
            BOOST_REQUIRE_EQUAL(d.win, static_cast<uint16_t>(win));
            BOOST_REQUIRE_EQUAL(
                d.tap,
                static_cast<uint16_t>(static_cast<size_t>(arm) * 2 * H_pad));
            arm += Core::kDecim;
            while (arm >= Core::kInterp) {
                arm -= Core::kInterp;
                ++win;
            }
        }
        // After all Interp outputs: cursor advanced by exactly Decim and the
        // phase is unchanged (that is why ctr_ survives a macroblock).
        BOOST_REQUIRE_EQUAL(win, static_cast<uint32_t>(Core::kDecim));
        BOOST_REQUIRE_EQUAL(arm, p);
    }

    // Tap offsets must point at real polyphase arms: the descriptor's tap[j]
    // is arm * 2 * H_pad and the second output of phase 0 uses the arm the
    // upfirdn recurrence reaches after one Decim step.
    {
        const auto& d0 = core.fixed_phase_group(0, 0);
        BOOST_CHECK_EQUAL(d0.win[0], 0u);
        BOOST_CHECK_EQUAL(d0.tap[0], 0u);
        uint32_t arm1 = Core::kDecim;
        while (arm1 >= Core::kInterp)
            arm1 -= Core::kInterp;
        BOOST_CHECK_EQUAL(core.fixed_phase_group(0, 0).tap[1],
                          static_cast<uint16_t>(arm1 * 2 * H_pad));
    }

    // set_kernel() to a non-AVX2 kernel makes the candidates inert; the
    // variant string must say so instead of pretending they are active.
    {
        Core scalar(taps);
        scalar.set_kernel("scalar");
        apply_variant(scalar, kVariantC1);
        BOOST_CHECK_EQUAL(std::string(scalar.sc16_kernel_variant()), "default");
    }
}

/** 11. Full bit-identity matrix: patterns x lengths x scales x variants. */
template <typename Core>
void check_candidate_matrix(const std::vector<float>& taps,
                            const std::string& tag)
{
    std::vector<size_t> lengths;
    {
        Core probe(taps);
        const size_t H = probe.arm_length();
        // short windows (n < H), exactly H, H+1, non-macroblock lengths
        // (Decim = 48 => +1, +23, +47 remainders), plus a large window.
        const size_t base = 48 * 25; // 1200
        lengths = { 0,   1,   2,   3,      H / 2, H - 1, H,   H + 1,
                    H + 7, base, base + 1, base + 23, base + 47,
                    4096, 4096 + 31, 4000 };
    }
    const Pattern patterns[] = { Pattern::Random, Pattern::Rails,
                                 Pattern::Zero, Pattern::Impulse };
    const float scales[] = { kUnitScale, kRawScale };

    for (size_t n : lengths) {
        if (n == 0)
            continue; // no process() call at all; flush-only is covered below
        for (Pattern p : patterns) {
            for (float scale : scales) {
                const std::string st =
                    tag + " n=" + std::to_string(n) + " " + pattern_name(p) +
                    ((scale == kRawScale) ? " raw" : " unit");
                auto in = make_pattern(n, p, 0x51ed270bu + static_cast<uint32_t>(n));
                check_scenario<Core>(taps, st, [&](Core& c) {
                    return run_oneshot_sc16(c, in, scale);
                });
            }
        }
    }

    // N == 0: the pure filter tail (flush without any process call).
    {
        auto in = make_pattern(0, Pattern::Zero, 1u);
        check_scenario<Core>(taps, tag + " n=0 tail-only", [&](Core& c) {
            return run_oneshot_sc16(c, in, kUnitScale);
        });
    }

    // Production window sizes: 56000 native samples is the pipeline's
    // 56 ms @ 998.4 MS/s burst window (and 56000 + 17 keeps it off the
    // macroblock grid).
    for (size_t n : { 56000ul, 56017ul }) {
        auto in = make_sc16_input(n, 0xfeedfaceu + static_cast<uint32_t>(n));
        for (float scale : { kUnitScale, kRawScale }) {
            check_scenario<Core>(
                taps,
                tag + " production n=" + std::to_string(n) +
                    ((scale == kRawScale) ? " raw" : " unit"),
                [&](Core& c) { return run_oneshot_sc16(c, in, scale); });
        }
    }
}

/** 12. Every legal starting phase, tiny grants, chunked streaming. */
template <typename Core>
void check_candidate_phases(const std::vector<float>& taps, const std::string& tag)
{
    const size_t N = 4096;
    auto in = make_sc16_input(N, 0x2f0f5d3du);
    const std::vector<size_t> chunks = { 1, 5, 96, 47, 1000, 2947 };

    for (float scale : { kUnitScale, kRawScale }) {
        const std::string st = tag + ((scale == kRawScale) ? " raw" : " unit");

        // ---- tiny max_out grants: many process calls, tiny remainder paths
        // followed by full macroblocks; the phase at each call is whatever the
        // stream gives, so this also covers non-phase-0 macroblock starts.
        for (size_t cap : { 1u, 2u, 3u, 63u, 64u, 65u, 66u, 67u, 130u }) {
            check_scenario<Core>(
                taps, st + " max_out=" + std::to_string(cap), [&](Core& c) {
                    return run_stream_sc16(c, in, scale, cap, {}).out;
                });
        }

        // ---- chunked streaming with a large grant (macroblock starts on a
        // mid-stream phase because the chunk sizes are not multiples of Decim)
        check_scenario<Core>(taps, st + " 6-chunk", [&](Core& c) {
            return run_stream_sc16(c, in, scale, 4096, chunks).out;
        });
        check_scenario<Core>(taps, st + " sample-at-a-time", [&](Core& c) {
            return run_stream_sc16(c, in, scale, 4096, { 1 }).out;
        });

        // ---- EVERY starting phase p in [0, Interp) is exercised as the
        // first phase of a full macroblock.  Priming with max_out == 1 forces
        // the remainder path one output at a time until ctr_ == p; the rest of
        // the stream is then handed over with a large grant, so the macroblock
        // loop starts at phase p with all 4*G+T outputs hitting the candidate
        // kernels/schedule for that phase.
        for (uint32_t p = 0; p < Core::kInterp; ++p) {
            // k outputs land on phase p  <=>  (k*Decim) mod Interp == p
            uint32_t k = 0;
            for (uint32_t cand = 0; cand < Core::kInterp; ++cand) {
                if ((cand * Core::kDecim) % Core::kInterp == p) {
                    k = cand;
                    break;
                }
            }
            check_scenario<Core>(
                taps, st + " phase=" + std::to_string(p), [&](Core& c) {
                    std::vector<gr_complex> y;
                    std::vector<gr_complex> one(1);
                    size_t off = 0;
                    // Prime: exactly k outputs, one output per call, with a
                    // few input samples per call so the local input is not
                    // exhausted and the phase wrap always completes.
                    while (c.output_items() < k) {
                        const size_t avail = std::min<size_t>(4, in.size() - off);
                        BOOST_REQUIRE_GT(avail, 0u);
                        auto r = c.process_sc16(in.data() + off, avail, scale,
                                                one.data(), 1);
                        if (r.produced > 0)
                            y.push_back(one[0]);
                        off += r.consumed;
                    }
                    // ctr_ < Interp is only invariant for Decim <= Interp: for
                    // Decim > Interp the documented exhaustion path may return
                    // from the middle of the phase wrap (process_sc16() and
                    // flush have an explicit repair for that), so the exact
                    // phase is asserted only for the RX directions.
                    if (Core::kDecim <= Core::kInterp)
                        BOOST_REQUIRE_EQUAL(c.phase(), p);
                    std::vector<sc16> rest(in.begin() + off, in.end());
                    auto mid = feed_sc16(c, rest, scale, 4096);
                    y.insert(y.end(), mid.begin(), mid.end());
                    drain_sc16(c, y);
                    return y;
                });
        }
    }

    // ---- the candidate switches are stream-safe: toggling them mid-stream
    // (which only changes kernel selection, never state) is bit-identical to
    // keeping one default core for the whole stream.
    check_scenario<Core>(taps, tag + " mid-stream toggle", [&](Core& c) {
        std::vector<gr_complex> y;
        size_t off = 0;
        const size_t c0 = N / 4;
        for (int phase = 0; phase < 4; ++phase) {
            const size_t n = std::min(c0, N - off);
            if (n == 0)
                break;
            std::vector<sc16> part(in.begin() + off, in.begin() + off + n);
            auto yp = feed_sc16(c, part, kUnitScale, 4096);
            y.insert(y.end(), yp.begin(), yp.end());
            off += n;
            // flip the switch (default -> C1 -> default)
            static const Variant seq[4] = { kVariantC1, kVariantDefault,
                                            kVariantDefault, kVariantDefault };
            apply_variant(c, seq[phase % 4]);
        }
        drain_sc16(c, y);
        return y;
    });
}

/** 13. The multi-worker schedule and the FC32 path are untouched by C1. */
template <typename Core>
void check_candidate_other_paths(const std::vector<float>& taps,
                                 const std::string& tag)
{
    const size_t N = 4096;
    auto in = make_sc16_input(N, 0x0d15ea5eu);

    // run_mt_sc16 keeps the pre-existing per-output schedule; the candidate
    // must not change it.
    {
        Core serial(taps);
        apply_variant(serial, kVariantDefault);
        auto y_ref = run_oneshot_sc16(serial, in, kUnitScale);

        for (Variant v : { kVariantC1 }) {
            Core mt(taps);
            apply_variant(mt, v);
            auto y = run_oneshot_sc16(mt, in, kUnitScale, /*nworkers=*/4);
            require_bit_identical_counted(
                y, y_ref, tag + " MT(4) [" + variant_switch_name(v) + "]");
        }
    }

    // FC32 process(): unchanged by the SC16-only candidates.
    {
        auto fin = to_cf32(make_sc16_input(N, 0x1234abcdu), kUnitScale);
        Core ref(taps);
        auto y_ref = run_oneshot_cf32(ref, fin);
        for (Variant v : { kVariantC1 }) {
            Core c(taps);
            apply_variant(c, v);
            auto y = run_oneshot_cf32(c, fin);
            require_bit_identical_counted(
                y, y_ref, tag + " FC32 [" + variant_switch_name(v) + "]");
        }
    }
}

// ===========================================================================
//  RS-1: zero-copy whole-window entry
//
//  process_window_sc16() must reproduce reset + process_sc16 + flush exactly,
//  including the leading-zero and trailing-zero boundary macroblocks and every
//  short window.  Both the default schedule and the C1 fixed-phase variant
//  are covered, so the batched macroblock entry (RS-2) is on the hook too.
// ===========================================================================
template <typename Core>
void check_window_entry(const std::vector<float>& taps, const std::string& tag)
{
    static const size_t lens[] = { 0,   1,   7,   47,  48,   49,
                                   65,  128, 1024, 4096, 56000 };
    for (int fp_i = 0; fp_i < 2; ++fp_i) {
        const bool fp = (fp_i == 1);
        for (size_t n : lens) {
            const std::string what =
                tag + " N=" + std::to_string(n) +
                (fp ? " C1" : " default");
            auto in = make_sc16_input(n, 0x3c6ef372u + static_cast<uint32_t>(n));

            Core ref(taps);
            ref.set_fixed_phase_schedule(fp);
            const std::vector<gr_complex> y_ref =
                run_oneshot_sc16(ref, in, kUnitScale);

            Core win(taps);
            win.set_fixed_phase_schedule(fp);
            std::vector<gr_complex> y(y_ref.size());
            const auto r = win.process_window_sc16(
                in.empty() ? nullptr : in.data(), n, kUnitScale, y.data(),
                y.size());
            BOOST_REQUIRE_EQUAL(r.produced, y.size());
            BOOST_REQUIRE_EQUAL(r.consumed, n);
            BOOST_CHECK(win.flush_complete());
            BOOST_CHECK_EQUAL(win.input_items(), static_cast<uint64_t>(n));
            BOOST_CHECK_EQUAL(win.output_items(),
                              static_cast<uint64_t>(y.size()));
            require_bit_identical_counted(y, y_ref, what + " window entry");

            // resample_sc16_window() is the production wrapper around it.
            std::vector<gr_complex> y2;
            const size_t made = openuwb::core::resample_sc16_window(
                win, in.empty() ? nullptr : in.data(), n, kUnitScale, y2);
            BOOST_REQUIRE_EQUAL(made, y_ref.size());
            BOOST_REQUIRE_EQUAL(y2.size(), y_ref.size());
            require_bit_identical_counted(y2, y_ref, what + " resample_window");
        }
    }
}

} // namespace

BOOST_AUTO_TEST_CASE(test_candidate_api_and_geometry)
{
    check_candidate_api<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                   "65/48 rt");
    check_candidate_api<RationalResampler65_48Core>(taps_65_48_quality(),
                                                    "65/48 q");
    check_candidate_api<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                    "65/32 rt");
    check_candidate_api<RationalResampler65_32Core>(taps_65_32_quality(),
                                                    "65/32 q");
    check_candidate_api<RationalResampler48_65Core>(taps_48_65_tx(), "48/65 tx");
}

BOOST_AUTO_TEST_CASE(test_candidate_bit_identity_matrix)
{
    check_candidate_matrix<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                       "65/48 rt");
    check_candidate_matrix<RationalResampler65_48Core>(taps_65_48_quality(),
                                                       "65/48 q");
    check_candidate_matrix<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                       "65/32 rt");
    check_candidate_matrix<RationalResampler48_65Core>(taps_48_65_tx(),
                                                       "48/65 tx");
}

BOOST_AUTO_TEST_CASE(test_candidate_all_phases_and_streaming)
{
    check_candidate_phases<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                       "65/48 rt");
    check_candidate_phases<RationalResampler65_48Core>(taps_65_48_quality(),
                                                       "65/48 q");
    check_candidate_phases<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                       "65/32 rt");
    check_candidate_phases<RationalResampler48_65Core>(taps_48_65_tx(),
                                                       "48/65 tx");
}

BOOST_AUTO_TEST_CASE(test_candidate_mt_and_fc32_unchanged)
{
    check_candidate_other_paths<RationalResampler65_48Core>(
        taps_65_48_realtime(), "65/48 rt");
    check_candidate_other_paths<RationalResampler65_48Core>(
        taps_65_48_quality(), "65/48 q");
    check_candidate_other_paths<RationalResampler65_32Core>(
        taps_65_32_quality(), "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_candidate_bit_identity_summary)
{
    BOOST_TEST_MESSAGE(
        "Phase-C candidates: " << g_candidate_comparisons
                               << " bit-identical comparisons, "
                               << g_candidate_samples
                               << " output samples compared float-by-float "
                                  "and by memcmp");
    BOOST_CHECK_GT(g_candidate_comparisons, 0u);
    BOOST_CHECK_GT(g_candidate_samples, 0u);
}

BOOST_AUTO_TEST_CASE(test_lazy_tables_construction){
    check_construction<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                   "65/48 rt");
    check_construction<RationalResampler65_48Core>(taps_65_48_quality(),
                                                   "65/48 q");
    check_construction<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                   "65/32 rt");
    check_construction<RationalResampler65_32Core>(taps_65_32_quality(),
                                                   "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_lazy_tables_full_cycle)
{
    check_full_cycle<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                 "65/48 rt");
    check_full_cycle<RationalResampler65_48Core>(taps_65_48_quality(),
                                                 "65/48 q");
    check_full_cycle<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                 "65/32 rt");
    check_full_cycle<RationalResampler65_32Core>(taps_65_32_quality(),
                                                 "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_lazy_tables_fc32_path)
{
    check_fc32_path<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                "65/48 rt");
    check_fc32_path<RationalResampler65_48Core>(taps_65_48_quality(),
                                                "65/48 q");
    check_fc32_path<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                "65/32 rt");
    check_fc32_path<RationalResampler65_32Core>(taps_65_32_quality(),
                                                "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_first_mt_call_allocates_and_matches_serial)
{
    check_first_mt_call<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                    "65/48 rt");
    check_first_mt_call<RationalResampler65_48Core>(taps_65_48_quality(),
                                                    "65/48 q");
    check_first_mt_call<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                    "65/32 rt");
    check_first_mt_call<RationalResampler65_32Core>(taps_65_32_quality(),
                                                    "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_mt_repeat_reuses_capacity)
{
    check_mt_reuse<RationalResampler65_48Core>(taps_65_48_realtime(),
                                               "65/48 rt");
    check_mt_reuse<RationalResampler65_32Core>(taps_65_32_quality(), "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_worker_toggle_1_4_1_4)
{
    check_worker_toggle<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                    "65/48 rt");
    check_worker_toggle<RationalResampler65_48Core>(taps_65_48_quality(),
                                                    "65/48 q");
    check_worker_toggle<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                    "65/32 rt");
    check_worker_toggle<RationalResampler65_32Core>(taps_65_32_quality(),
                                                    "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_short_input_tiny_max_out_and_chunked)
{
    check_streaming_equivalence<RationalResampler65_48Core>(
        taps_65_48_realtime(), "65/48 rt");
    check_streaming_equivalence<RationalResampler65_48Core>(
        taps_65_48_quality(), "65/48 q");
    check_streaming_equivalence<RationalResampler65_32Core>(
        taps_65_32_realtime(), "65/32 rt");
    check_streaming_equivalence<RationalResampler65_32Core>(
        taps_65_32_quality(), "65/32 q");
}

BOOST_AUTO_TEST_CASE(test_common_template_path_48_65) { check_48_65(); }

BOOST_AUTO_TEST_CASE(test_expected_length_contract)
{
    check_length_contract<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                      "65/48 rt");
    check_length_contract<RationalResampler65_48Core>(taps_65_48_quality(),
                                                      "65/48 q");
    check_length_contract<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                      "65/32 rt");
    check_length_contract<RationalResampler65_32Core>(taps_65_32_quality(),
                                                      "65/32 q");
    check_length_contract<RationalResampler48_65Core>(taps_48_65_tx(),
                                                      "48/65 tx");
}

BOOST_AUTO_TEST_CASE(test_process_window_zero_copy_bit_identity)
{
    check_window_entry<RationalResampler65_48Core>(taps_65_48_realtime(),
                                                   "65/48 rt");
    check_window_entry<RationalResampler65_48Core>(taps_65_48_quality(),
                                                   "65/48 q");
    check_window_entry<RationalResampler65_32Core>(taps_65_32_realtime(),
                                                   "65/32 rt");
    check_window_entry<RationalResampler65_32Core>(taps_65_32_quality(),
                                                   "65/32 q");
    check_window_entry<RationalResampler48_65Core>(taps_48_65_tx(), "48/65 tx");
}
