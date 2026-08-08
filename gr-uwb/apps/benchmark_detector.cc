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
 *   benchmark_detector <cfile> <mode>
 *     mode: reference | fast | coarse
 *   benchmark_detector [--reps N] [--energy-dec D] [--coarse-dec D]
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector.h>
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

    const auto w0 = std::chrono::steady_clock::now();
    const auto c0 = std::clock();
    const size_t CH = 8192;
    for (size_t off = 0; off < stream.size(); off += CH) {
        const size_t n = std::min(CH, stream.size() - off);
        sm.process(stream.data() + off, n, off);
        while (sm.region_ready()) {
            const auto handle = sm.take_region();
            ++regions;
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
    // Output of the energy-gate stage: one decimated energy value per D input
    // samples (D = energy_gate_decimation = 100), so its output stream rate is
    // the input throughput divided by D.
    std::printf("output stream     : %.1f MS/s (decimated energy, D=100)\n",
                n / wall / 100.0 / 1e6);
    std::printf("regions           : %zu\n", regions);
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

    for (int i = 3; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        if (k == "--reps") reps = std::stoul(argv[i + 1]);
        else if (k == "--energy-dec") ed = std::stoul(argv[i + 1]);
        else if (k == "--coarse-dec") cd = std::stoul(argv[i + 1]);
        else if (k == "--target") target = std::stoull(argv[i + 1]);
        else if (k == "--gap") det_gap = std::stoul(argv[i + 1]);
    }

    if (mode == "detector" || mode == "detector-sparse" || mode == "detector-dense") {
        // UwbDetector sparse/PDU pipeline (开发需求参考.md §10, production path).
        const size_t gap = (mode == "detector-dense") ? 6000 : det_gap;
        return run_detector_bench(cfile, target, gap, mode);
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
