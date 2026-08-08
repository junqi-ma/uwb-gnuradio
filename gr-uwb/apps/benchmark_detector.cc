/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Standalone detector benchmark (开发需求参考.md §10).
 *
 * Input is generated IN MEMORY (the reference UWB test signal repeated), so
 * the measured time is pure C++ work() throughput — no disk I/O, no Python,
 * no output materialization (null sinks).  Reports processed samples, elapsed,
 * throughput in MS/s and GB/s, and detections.
 *
 * Usage:
 *   benchmark_detector <cfile> <mode> [options]
 *     mode:
 *       reference | fast | coarse
 *       detector-gate | detector-sparse | detector-dense
 *       detector-region         — continuous Region IQ → coarse/fine (core)
 *       detector-region-stream  — min-gap packet stream → full UwbDetector
 *     options: --reps N --energy-dec D --coarse-dec D --target N --gap G
 *              --regions N  (detector-region only, default 200)
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_preamble_detector.h>
#include <pmt/pmt.h>

#include <chrono>
#include <complex>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr size_t kPacketStart = 4992000;
constexpr size_t kSymbolLen = 1016;
constexpr size_t kPacketLen = 256448;
// Energy-gate region shape (matches production UwbDetector defaults):
// pre_trigger ring = max(user 2032, 32*D + L) = 32*100 + 1016 = 4216.
// Measured closed region length on the reference packet is 264016.
constexpr size_t kRegionPreTrigger = 4216;
constexpr size_t kRegionLenMeasured = 264016;
constexpr size_t kUserPreTrigger = 2032;
constexpr size_t kCapture = 200000;
constexpr size_t kCoarseDec = 4;
constexpr size_t kCoarseReps = 1;
constexpr size_t kCoarseMargin = 16;
constexpr size_t kCoarseStride = 1;

// ---------------------------------------------------------------------------
// PatternSource: streams [packet, gap-silence, packet, ...] up to `target`
// samples without storing the whole stream (so ~1e9-sample runs fit in memory).
// ---------------------------------------------------------------------------
class PatternSource : public gr::sync_block
{
public:
    PatternSource(const std::vector<gr_complex>& pkt, size_t gap, uint64_t target)
        : gr::sync_block("pattern_source",
                         gr::io_signature::make(0, 0, 0),
                         gr::io_signature::make(1, 1, sizeof(gr_complex))),
          d_pkt(pkt),
          d_gap(gap),
          d_target(target)
    {
    }

    int work(int noutput_items,
             gr_vector_const_void_star&,
             gr_vector_void_star& output_items) override
    {
        auto* out = reinterpret_cast<gr_complex*>(output_items[0]);
        int produced = 0;
        while (produced < noutput_items && d_total < d_target) {
            if (d_cycle_pos < d_pkt.size()) {
                const size_t n = std::min(d_pkt.size() - d_cycle_pos,
                                          static_cast<size_t>(noutput_items - produced));
                std::memcpy(out + produced, d_pkt.data() + d_cycle_pos,
                            n * sizeof(gr_complex));
                d_cycle_pos += n;
                produced += static_cast<int>(n);
            } else {
                const size_t gap_left = d_gap - (d_cycle_pos - d_pkt.size());
                const size_t n = std::min(gap_left,
                                          static_cast<size_t>(noutput_items - produced));
                std::memset(out + produced, 0, n * sizeof(gr_complex));
                d_cycle_pos += n;
                produced += static_cast<int>(n);
            }
            if (d_cycle_pos >= d_pkt.size() + d_gap)
                d_cycle_pos = 0;
        }
        d_total += static_cast<uint64_t>(produced);
        if (produced == 0)
            return -1; // done
        return produced;
    }

private:
    std::vector<gr_complex> d_pkt;
    size_t d_gap;
    uint64_t d_target;
    uint64_t d_total = 0;
    size_t d_cycle_pos = 0;
};

// ---------------------------------------------------------------------------
// PduCounter: message sink that counts PDUs without storing payloads.
// ---------------------------------------------------------------------------
class PduCounter : public gr::block
{
public:
    PduCounter()
        : gr::block("pdu_counter",
                    gr::io_signature::make(0, 0, 0),
                    gr::io_signature::make(0, 0, 0))
    {
        message_port_register_in(pmt::mp("packet"));
        set_msg_handler(pmt::mp("packet"), [this](pmt::pmt_t msg) {
            ++d_count;
            pmt::pmt_t data = pmt::cdr(msg);
            if (pmt::is_c32vector(data))
                d_total_samples += pmt::length(data);
        });
    }

    size_t count() const { return d_count; }
    size_t total_samples() const { return d_total_samples; }

private:
    size_t d_count = 0;
    size_t d_total_samples = 0;
};

// ---------------------------------------------------------------------------
// Energy-gate-only benchmark: runs UwbDetectorStateMachine (decimated gate +
// cross-chunk region buffering) directly, WITHOUT the coarse/fine preamble
// confirmation and PDU emission.  This isolates the energy-gate stage cost.
// ---------------------------------------------------------------------------
int run_gate_bench(const std::string& cfile, uint64_t target, size_t gap)
{
    std::ifstream f(cfile, std::ios::binary);
    if (!f) {
        std::cerr << "cannot open " << cfile << "\n";
        return 1;
    }
    f.seekg(0, std::ios::end);
    size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<gr_complex> base(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(base.data()), static_cast<std::streamsize>(bytes));

    std::vector<gr_complex> pkt(base.begin() + kPacketStart,
                                base.begin() + kPacketStart + kPacketLen);

    // Sparse stream [packet + gap] in memory, matching the full-detector bench.
    std::vector<gr_complex> stream;
    stream.reserve(static_cast<size_t>(target));
    while (stream.size() < target) {
        const size_t remain = static_cast<size_t>(target - stream.size());
        const size_t pkt_n = std::min(pkt.size(), remain);
        stream.insert(stream.end(), pkt.begin(), pkt.begin() + pkt_n);
        if (stream.size() >= target)
            break;
        const size_t gap_n = std::min(gap, remain - pkt_n);
        stream.insert(stream.end(), gap_n, gr_complex(0.0f, 0.0f));
    }

    gr::uwb::UwbDetectorStateMachine sm(4216, 1e-3f, 100, 32, 8);
    size_t regions = 0;
    uint64_t total_region_iq = 0;

    const auto w0 = std::chrono::steady_clock::now();
    const auto c0 = std::clock();
    const size_t CH = 8192;
    for (size_t off = 0; off < stream.size(); off += CH) {
        const size_t n = std::min(CH, stream.size() - off);
        sm.process(stream.data() + off, n, off);
        while (sm.region_ready()) {
            const auto handle = sm.take_region();
            ++regions;
            total_region_iq += sm.region(handle).samples.size();
            sm.release_region(handle);
        }
    }
    const auto c1 = std::clock();
    const auto w1 = std::chrono::steady_clock::now();

    const double wall = std::chrono::duration<double>(w1 - w0).count();
    const double cpu = static_cast<double>(c1 - c0) / CLOCKS_PER_SEC;
    const double n = static_cast<double>(stream.size());

    std::printf("== Energy gate (state machine only, no coarse/fine) ==\n");
    std::printf("processed samples : %zu\n", stream.size());
    std::printf("elapsed time      : %.3f s\n", wall);
    std::printf("CPU time          : %.3f s  (utilization %.0f%%)\n", cpu,
                wall > 0 ? 100.0 * cpu / wall : 0.0);
    std::printf("throughput        : %.1f MS/s\n", n / wall / 1e6);
    std::printf("throughput        : %.2f GB/s\n", n * 8.0 / wall / 1e9);
    // Output of the energy-gate stage is a sparse stream of Region objects
    // (pre-trigger + raw IQ above the threshold).  Report the average IQ rate
    // of those Region payloads, their mean length, and how much of the input
    // is retained as region IQ.
    std::printf("regions           : %zu\n", regions);
    std::printf("region IQ total   : %llu samples\n",
                static_cast<unsigned long long>(total_region_iq));
    std::printf("avg region length : %.0f samples\n",
                regions > 0 ? static_cast<double>(total_region_iq) / regions : 0.0);
    std::printf("region IQ rate    : %.3f MS/s (%.2f GB/s)\n",
                total_region_iq / wall / 1e6, total_region_iq * 8.0 / wall / 1e9);
    std::printf("capture fraction  : %.3f%% of input\n",
                100.0 * total_region_iq / n);
    std::printf("\n");
    return 0;
}

int run_detector_bench(const std::string& cfile,
                       uint64_t target,
                       size_t gap,
                       const std::string& label)
{
    std::ifstream f(cfile, std::ios::binary);
    if (!f) {
        std::cerr << "cannot open " << cfile << "\n";
        return 1;
    }
    f.seekg(0, std::ios::end);
    size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<gr_complex> base(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(base.data()), static_cast<std::streamsize>(bytes));

    std::vector<gr_complex> tmpl(base.begin() + kPacketStart,
                                 base.begin() + kPacketStart + kSymbolLen);
    std::vector<gr_complex> pkt(base.begin() + kPacketStart,
                                base.begin() + kPacketStart + kPacketLen);

    auto det = gr::uwb::UwbDetector::make(tmpl, 2032, 200000, 1e-3f, 100, 4, 1, 16);
    std::shared_ptr<gr::block> src;
    if (target <= 300000000ULL) {
        // Pre-built in-memory stream: gaps are zeroed once, so the source only
        // memcpys (no per-cycle memset generation) — isolates the detector's
        // own cost from source-generation overhead.
        std::vector<gr_complex> stream;
        stream.reserve(static_cast<size_t>(target));
        while (stream.size() < target) {
            const size_t remain = static_cast<size_t>(target - stream.size());
            const size_t pkt_n = std::min(pkt.size(), remain);
            stream.insert(stream.end(), pkt.begin(), pkt.begin() + pkt_n);
            if (stream.size() >= target)
                break;
            const size_t gap_n = std::min(gap, remain - pkt_n);
            stream.insert(stream.end(), gap_n, gr_complex(0.0f, 0.0f));
        }
        auto vs = gr::blocks::vector_source_c::make(stream);
        vs->set_max_output_buffer(0, 1 << 22); // ~4M complex = 32 MB
        src = vs;
    } else {
        auto ps = std::make_shared<PatternSource>(pkt, gap, target);
        ps->set_max_output_buffer(0, 1 << 22);
        src = ps;
    }
    auto cnt = std::make_shared<PduCounter>();
    auto tb = gr::make_top_block("bench_detector");
    tb->connect(src, 0, det, 0);
    tb->msg_connect(det, "packet", cnt, "packet");

    const auto w0 = std::chrono::steady_clock::now();
    const auto c0 = std::clock();
    tb->run();
    const auto c1 = std::clock();
    const auto w1 = std::chrono::steady_clock::now();
    const double wall = std::chrono::duration<double>(w1 - w0).count();
    const double cpu = static_cast<double>(c1 - c0) / CLOCKS_PER_SEC;
    const double n = static_cast<double>(target);

    std::printf("== UwbDetector sparse pipeline: %s ==\n", label.c_str());
    std::printf("processed samples : %llu\n", static_cast<unsigned long long>(target));
    std::printf("elapsed time      : %.3f s\n", wall);
    std::printf("CPU time          : %.3f s  (utilization %.0f%%)\n", cpu,
                wall > 0 ? 100.0 * cpu / wall : 0.0);
    std::printf("throughput        : %.1f MS/s\n", n / wall / 1e6);
    std::printf("throughput        : %.2f GB/s\n", n * 8.0 / wall / 1e9);
    std::printf("dropped regions   : %llu\n", static_cast<unsigned long long>(det->dropped_regions()));
    std::printf("detections        : %zu\n", cnt->count());
    const double captured = static_cast<double>(cnt->total_samples());
    std::printf("captured IQ       : %.0f samples (%.2f%% of input)\n", captured,
                100.0 * captured / n);
    std::printf("output stream     : %.3f MS/s (%.2f GB/s)\n",
                captured / wall / 1e6, captured * 8.0 / wall / 1e9);
    std::printf("1 GS/s target     : %s\n", (n / wall / 1e6) >= 1000.0 ? "MET" : "not yet");
    std::printf("\n");
    return 0;
}

// ---------------------------------------------------------------------------
// Load cfile packet + 1016-sample template.  Returns false on I/O error.
// ---------------------------------------------------------------------------
bool load_packet_and_template(const std::string& cfile,
                              std::vector<gr_complex>& pkt,
                              std::vector<gr_complex>& tmpl)
{
    std::ifstream f(cfile, std::ios::binary);
    if (!f) {
        std::cerr << "cannot open " << cfile << "\n";
        return false;
    }
    f.seekg(0, std::ios::end);
    size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<gr_complex> base(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(base.data()), static_cast<std::streamsize>(bytes));
    if (base.size() < kPacketStart + kPacketLen) {
        std::cerr << "cfile too short for packet slice\n";
        return false;
    }
    tmpl.assign(base.begin() + kPacketStart,
                base.begin() + kPacketStart + kSymbolLen);
    pkt.assign(base.begin() + kPacketStart,
               base.begin() + kPacketStart + kPacketLen);
    return true;
}

// Build a production-shaped Region IQ buffer: pre-trigger zeros + packet +
// trailing zeros so the total length matches the energy-gate measurement
// (264016).  This is the object the worker would receive after a gate close.
std::vector<gr_complex> build_region_iq(const std::vector<gr_complex>& pkt)
{
    std::vector<gr_complex> region(kRegionLenMeasured, gr_complex(0.0f, 0.0f));
    const size_t n = std::min(pkt.size(), kRegionLenMeasured - kRegionPreTrigger);
    std::memcpy(region.data() + kRegionPreTrigger, pkt.data(),
                n * sizeof(gr_complex));
    return region;
}

// Run the same coarse → fine → capture path as UwbDetector::publish_packet.
// Returns true if a preamble was confirmed (would emit a PDU).
bool process_region_like_detector(
    const std::vector<gr_complex>& region,
    const std::vector<gr_complex>& tmpl_ds,
    size_t sym_ds,
    gr::filter::kernel::fir_filter_ccc& fir,
    float template_energy,
    size_t template_len,
    size_t pre_trigger,
    size_t capture,
    // scratch (reused across calls)
    std::vector<gr_complex>& sig_ds,
    std::vector<float>& pow_ds,
    std::vector<float>& score_ds,
    std::vector<float>& metric_ds,
    std::vector<size_t>& coarse_peaks,
    std::vector<gr_complex>& corr,
    std::vector<float>& winpow,
    std::vector<float>& fine_metric,
    size_t& out_capture_len)
{
    out_capture_len = 0;
    const size_t n = region.size();
    if (n == 0)
        return false;

    float mx = 0.0f;
    gr::uwb::core::uwb_coarse_peaks(region.data(),
                                    0,
                                    n,
                                    tmpl_ds.data(),
                                    tmpl_ds.size(),
                                    kCoarseDec,
                                    kCoarseReps,
                                    sym_ds,
                                    /*peak_rel=*/0.5f,
                                    /*exist_frac=*/0.5f,
                                    kCoarseStride,
                                    sig_ds,
                                    pow_ds,
                                    score_ds,
                                    metric_ds,
                                    coarse_peaks,
                                    &mx);
    if (coarse_peaks.empty())
        return false;

    const size_t Lm1 = template_len - 1;
    const size_t half = kCoarseStride * kCoarseDec + kCoarseMargin;
    size_t earliest_end = n;
    float best_metric = 0.0f;
    for (size_t p : coarse_peaks) {
        if (p + Lm1 >= n)
            continue;
        const size_t center = p + Lm1;
        const size_t j0 = (center > half) ? center - half : 0;
        const size_t j1 = std::min(center + half, n - 1);
        const size_t len = j1 - j0 + 1;

        fir.filterN(corr.data(), region.data() + j0,
                    static_cast<unsigned long>(len));
        gr::uwb::core::uwb_window_power(region.data() + j0, len, template_len,
                                        winpow.data());
        gr::uwb::core::uwb_normalized_score(corr.data(), winpow.data(), len,
                                            template_energy, fine_metric.data());

        size_t local_best = 0;
        for (size_t k = 0; k < len; ++k) {
            if (fine_metric[k] > best_metric)
                best_metric = fine_metric[k];
            if (fine_metric[k] > fine_metric[local_best])
                local_best = k;
        }
        if (fine_metric[local_best] >= 0.5f) {
            const size_t end = j0 + local_best;
            if (end < earliest_end)
                earliest_end = end;
        }
    }
    if (earliest_end >= n || best_metric < 0.5f)
        return false;

    // Capture clamp matches publish_packet (region-relative, start at 0).
    // packet_start_rel = earliest_end - (L-1); lo_off from pre_trigger.
    const size_t packet_start_rel = earliest_end - Lm1;
    const size_t lo_off =
        (packet_start_rel >= pre_trigger) ? packet_start_rel - pre_trigger : 0;
    out_capture_len = std::min(pre_trigger + capture, n - lo_off);

    // Materialize capture IQ the way PDU construction does (memcpy cost).
    std::vector<gr_complex> iq(region.begin() + static_cast<std::ptrdiff_t>(lo_off),
                               region.begin() + static_cast<std::ptrdiff_t>(lo_off +
                                                                            out_capture_len));
    (void)iq;
    return true;
}

// ---------------------------------------------------------------------------
// Direct continuous Region-block stress test.
// Feeds the same Region IQ repeatedly into the production coarse/fine path
// (no energy gate, no GNU Radio scheduler).  Reports the maximum Region IQ
// sample rate the detector worker algorithm can sustain.
// ---------------------------------------------------------------------------
int run_region_core_bench(const std::string& cfile, size_t n_regions)
{
    std::vector<gr_complex> pkt, tmpl;
    if (!load_packet_and_template(cfile, pkt, tmpl))
        return 1;

    const std::vector<gr_complex> region = build_region_iq(pkt);

    // Matched-filter taps: reversed conjugated L2-normalized template.
    std::vector<gr_complex> tmpl_norm = tmpl;
    gr::uwb::core::uwb_l2_normalize(tmpl_norm);
    const float tenergy = gr::uwb::core::uwb_template_energy(tmpl_norm);
    std::vector<gr_complex> taps;
    taps.reserve(tmpl_norm.size());
    for (auto it = tmpl_norm.rbegin(); it != tmpl_norm.rend(); ++it)
        taps.push_back(std::conj(*it));
    gr::filter::kernel::fir_filter_ccc fir(taps);

    // Decimated template (same as UwbDetector::rebuild_decimated_template).
    const size_t D = kCoarseDec;
    const size_t Ld = tmpl.size() / D;
    std::vector<gr_complex> tmpl_ds;
    tmpl_ds.reserve(Ld);
    for (size_t j = 0; j < Ld; ++j) {
        const size_t m = j * D;
        tmpl_ds.push_back(std::conj(taps[tmpl.size() - 1 - m]));
    }
    gr::uwb::core::uwb_l2_normalize(tmpl_ds);
    const size_t sym_ds = Ld;

    // Scratch (pre-sized like the worker).
    std::vector<gr_complex> sig_ds, corr;
    std::vector<float> pow_ds, score_ds, metric_ds, winpow, fine_metric;
    std::vector<size_t> coarse_peaks;
    const size_t half = kCoarseStride * kCoarseDec + kCoarseMargin;
    corr.resize(2 * half + 1);
    winpow.resize(2 * half + 1);
    fine_metric.resize(2 * half + 1);
    coarse_peaks.reserve(256);

    // Warm-up (one region) so VOLK dispatch / cache is settled.
    size_t cap0 = 0;
    const bool warm_ok = process_region_like_detector(
        region, tmpl_ds, sym_ds, fir, tenergy, tmpl.size(), kUserPreTrigger,
        kCapture, sig_ds, pow_ds, score_ds, metric_ds, coarse_peaks, corr,
        winpow, fine_metric, cap0);
    if (!warm_ok) {
        std::cerr << "region core warm-up failed to detect preamble "
                     "(check region construction)\n";
        return 1;
    }

    size_t detections = 0;
    uint64_t total_capture = 0;
    const uint64_t total_region_iq =
        static_cast<uint64_t>(n_regions) * region.size();

    const auto w0 = std::chrono::steady_clock::now();
    const auto c0 = std::clock();
    for (size_t i = 0; i < n_regions; ++i) {
        size_t cap = 0;
        if (process_region_like_detector(region, tmpl_ds, sym_ds, fir, tenergy,
                                         tmpl.size(), kUserPreTrigger, kCapture,
                                         sig_ds, pow_ds, score_ds, metric_ds,
                                         coarse_peaks, corr, winpow, fine_metric,
                                         cap)) {
            ++detections;
            total_capture += cap;
        }
    }
    const auto c1 = std::clock();
    const auto w1 = std::chrono::steady_clock::now();

    const double wall = std::chrono::duration<double>(w1 - w0).count();
    const double cpu = static_cast<double>(c1 - c0) / CLOCKS_PER_SEC;
    const double region_ms =
        wall / static_cast<double>(n_regions) * 1e3;

    std::printf("== UwbDetector Region core (continuous Region blocks) ==\n");
    std::printf("region length     : %zu samples (pre=%zu + packet + tail)\n",
                region.size(), kRegionPreTrigger);
    std::printf("regions processed : %zu\n", n_regions);
    std::printf("detections        : %zu\n", detections);
    std::printf("region IQ total   : %llu samples\n",
                static_cast<unsigned long long>(total_region_iq));
    std::printf("elapsed time      : %.3f s\n", wall);
    std::printf("CPU time          : %.3f s  (utilization %.0f%%)\n", cpu,
                wall > 0 ? 100.0 * cpu / wall : 0.0);
    std::printf("time per region   : %.3f ms\n", region_ms);
    std::printf("regions / s       : %.1f\n",
                wall > 0 ? n_regions / wall : 0.0);
    // Primary metric: max Region IQ sample rate the coarse/fine path sustains.
    std::printf("region IQ rate    : %.3f MS/s (%.2f GB/s)\n",
                total_region_iq / wall / 1e6, total_region_iq * 8.0 / wall / 1e9);
    std::printf("capture IQ total  : %llu samples\n",
                static_cast<unsigned long long>(total_capture));
    std::printf("capture IQ rate   : %.3f MS/s\n",
                total_capture / wall / 1e6);
    // How many real-time 1 GS/s packets this rate supports (region ≈ 264k).
    const double max_pkt_per_s =
        wall > 0 ? static_cast<double>(n_regions) / wall : 0.0;
    std::printf("equiv packet rate : %.1f pkt/s @ region=%zu\n", max_pkt_per_s,
                region.size());
    std::printf("vs gate@1pkt/s    : gate region IQ ~1.18 MS/s; "
                "core capacity / gate load = %.0fx\n",
                (total_region_iq / wall / 1e6) / 1.18);
    std::printf("\n");
    return 0;
}

// ---------------------------------------------------------------------------
// Full UwbDetector under continuous region pressure: packet + min gap so the
// energy gate keeps closing regions as fast as possible.  Measures drops and
// effective region/capture rates through the real worker + job queue.
// ---------------------------------------------------------------------------
int run_region_stream_bench(const std::string& cfile,
                            uint64_t target,
                            size_t gap)
{
    // gap=6000 is the existing detector-dense default (above ~4000 min holdoff).
    if (gap == 0)
        gap = 6000;
    return run_detector_bench(cfile, target, gap,
                              "detector-region-stream (gap=" +
                                  std::to_string(gap) + ")");
}

} // namespace

int main(int argc, char** argv)
{
    std::string cfile = (argc > 1) ? argv[1] : "testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile";
    std::string mode = (argc > 2) ? argv[2] : "coarse";
    size_t reps = 4;          // how many times to repeat the signal in memory
    size_t ed = 100;          // energy gate decimation
    size_t cd = 8;            // coarse decimation
    uint64_t target = 500000000ULL; // detector-mode samples to process
    size_t det_gap = 2'300'000;     // detector-mode inter-packet gap (sparse)
    size_t n_regions = 200;         // detector-region continuous block count

    for (int i = 3; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        if (k == "--reps") reps = std::stoul(argv[i + 1]);
        else if (k == "--energy-dec") ed = std::stoul(argv[i + 1]);
        else if (k == "--coarse-dec") cd = std::stoul(argv[i + 1]);
        else if (k == "--target") target = std::stoull(argv[i + 1]);
        else if (k == "--gap") det_gap = std::stoul(argv[i + 1]);
        else if (k == "--regions") n_regions = std::stoul(argv[i + 1]);
    }

    if (mode == "detector" || mode == "detector-sparse" || mode == "detector-dense") {
        // UwbDetector sparse/PDU pipeline (开发需求参考.md §10, production path).
        const size_t gap = (mode == "detector-dense") ? 6000 : det_gap;
        return run_detector_bench(cfile, target, gap, mode);
    }

    if (mode == "detector-region") {
        return run_region_core_bench(cfile, n_regions);
    }

    if (mode == "detector-region-stream") {
        // Continuous region pressure via min-gap stream (default gap 6000).
        const size_t gap = (det_gap == 2'300'000) ? 6000 : det_gap;
        return run_region_stream_bench(cfile, target, gap);
    }

    if (mode == "detector-gate") {
        // Energy-gate stage only (state machine, no coarse/fine/PDU).
        return run_gate_bench(cfile, target, det_gap);
    }

    std::ifstream f(cfile, std::ios::binary);
    if (!f) {
        std::cerr << "cannot open " << cfile << "\n";
        return 1;
    }
    f.seekg(0, std::ios::end);
    size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0, std::ios::beg);
    std::vector<gr_complex> base(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(base.data()), static_cast<std::streamsize>(bytes));

    // In-memory input: repeat the reference signal so the run is long enough
    // for a stable measurement (and scales toward 1e9 samples).
    std::vector<gr_complex> sig;
    sig.reserve(base.size() * reps);
    for (size_t r = 0; r < reps; ++r)
        sig.insert(sig.end(), base.begin(), base.end());

    std::vector<gr_complex> tmpl(base.begin() + kPacketStart,
                                 base.begin() + kPacketStart + kSymbolLen);

    gr::uwb::UwbPreambleDetector::sptr blk;
    if (mode == "reference") {
        blk = gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f);
    } else if (mode == "fast") {
        blk = gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f, 1e-3f, 32, 1);
    } else {
        blk = gr::uwb::UwbPreambleDetector::make(tmpl, 0.5f, 1e-3f, 8, ed, cd, 8, 16);
    }

    auto src = gr::blocks::vector_source_c::make(sig);
    auto ns0 = gr::blocks::null_sink::make(sizeof(float));
    auto ns1 = gr::blocks::null_sink::make(sizeof(unsigned char));
    auto tb = gr::make_top_block("benchmark_detector");
    tb->connect(src, 0, blk, 0);
    tb->connect(blk, 0, ns0, 0);
    tb->connect(blk, 1, ns1, 0);

    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    const auto t1 = std::chrono::steady_clock::now();
    const double sec = std::chrono::duration<double>(t1 - t0).count();
    const double n = static_cast<double>(sig.size());

    std::printf("processed samples : %zu\n", sig.size());
    std::printf("elapsed time      : %.3f s\n", sec);
    std::printf("throughput        : %.1f MS/s\n", n / sec / 1e6);
    std::printf("throughput        : %.2f GB/s\n", n * 8.0 / sec / 1e9);
    std::printf("input rate equiv  : %.1f GS/s (%.0f MS/s)\n",
                n / sec / 1e9, n / sec / 1e6);
    std::printf("mode              : %s (energy-dec=%zu, coarse-dec=%zu, reps=%zu)\n",
                mode.c_str(), ed, cd, reps);

    return 0;
}
