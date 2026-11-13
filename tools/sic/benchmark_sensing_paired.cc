// Paired sensing-receive benchmark for the R04 rectification.
//
// Deliberately uses ONLY the API surface that exists in both the pre-R04
// control build (921c38f) and the rectified candidate, so the same source
// compiles against either tree and the comparison is A/B/B/A between two
// binaries rather than two code paths in one binary.
//
// Measured: the outer wall clock of radar::radar_cir_one() (the sensing
// receive stage) and, when the build exposes them, the shared front-end
// sub-timers.  Correctness guard: both arms must return Ok with the same
// peak tap and the same tap count, otherwise the timings are not reported.
//
// No allocation inside the timed loop; the window is prepared once.
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using cf = std::complex<float>;

int main(int argc, char** argv)
{
    size_t sync_reps = 256;
    size_t psdu_bytes = 127;
    size_t iters = 32;
    size_t warmup = 32;
    std::string label = "arm";
    std::string csv = "-";

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value after %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--sync-reps")
            sync_reps = std::stoul(next());
        else if (a == "--psdu-bytes")
            psdu_bytes = std::stoul(next());
        else if (a == "--iters")
            iters = std::stoul(next());
        else if (a == "--warmup")
            warmup = std::stoul(next());
        else if (a == "--label")
            label = next();
        else if (a == "--csv")
            csv = next();
        else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
    }

    // ---- fixture: one real 256-SYNC / 127-byte packet on the work grid ----
    gr::uwb::mod::HrpModConfig mcfg;
    mcfg.code_index = 9;
    mcfg.sync_repetitions = sync_reps;
    mcfg.sfd_mode = "4z2";
    mcfg.peak_amplitude = 0.8f;
    const std::vector<uint8_t> psdu =
        gr::uwb::mod::make_random_psdu(psdu_bytes - 2, 12345, true);
    const size_t cap =
        gr::uwb::mod::packet_samples_998p4(sync_reps, 8, psdu.size(), false,
                                           &mcfg.pulse) +
        1024;
    gr::uwb::mod::HrpModScratch mscratch;
    mscratch.reserve(127, cap);
    std::vector<cf> packet(cap), sync(cap);
    size_t plen = 0;
    if (!gr::uwb::mod::modulate_one(psdu.data(), psdu.size(), mcfg, mscratch,
                                    packet.data(), cap, plen))
        return 1;
    packet.resize(plen);
    size_t slen = 0;
    if (!gr::uwb::mod::modulate_prefix_waveform(mcfg, mscratch, sync.data(),
                                                cap, slen))
        return 1;
    sync.resize(slen);

    const size_t sps = gr::uwb::demod::kQm35SamplesPerSymbol;
    const int64_t lead = 1997; // frame origin inside the window
    const size_t n = static_cast<size_t>(lead) + packet.size() + 4096;
    std::vector<cf> rx(n, cf(0.f, 0.f));
    std::copy(packet.begin(), packet.end(), rx.begin() + lead);

    gr::uwb::radar::RadarCirConfig cfg;
    cfg.code_index = 9;
    cfg.sync_repetitions = sync_reps;
    cfg.samples_per_symbol = sps;
    cfg.sfd_mode = "4z2";
    cfg.cir_pre = 16;
    cfg.cir_post = 100; // 116 taps
    cfg.cir_skip_initial = 10;
    cfg.cir_repetitions = sync_reps - 10;
    cfg.sfd_search_margin = 64;
    cfg.sync_refine_margin = 8;
    cfg.sfd_threshold = 0.3f;
    cfg.sync_refine_threshold = 0.3f;
    cfg.use_predicted_timing = false;

    gr::uwb::radar::RadarCirCoreScratch scratch;
    if (!gr::uwb::radar::prepare_radar_cir_core(cfg, sync.data(), sps, 16, 100,
                                                scratch)) {
        std::fprintf(stderr, "prepare failed\n");
        return 1;
    }
    gr::uwb::radar::reserve_radar_cir_core(n, scratch);
    const int64_t predicted = lead + static_cast<int64_t>(sync_reps) * sps;

    gr::uwb::radar::RadarCirResult ref;
    if (!gr::uwb::radar::radar_cir_one(rx.data(), n, predicted, cfg, scratch,
                                       ref) ||
        ref.status != gr::uwb::radar::RadarCirStatus::Ok) {
        std::fprintf(stderr, "fixture does not decode in this build\n");
        return 1;
    }

    for (size_t i = 0; i < warmup; ++i)
        gr::uwb::radar::radar_cir_one(rx.data(), n, predicted, cfg, scratch,
                                      ref);

    std::FILE* out = (csv == "-") ? stdout : std::fopen(csv.c_str(), "w");
    if (!out) {
        std::fprintf(stderr, "cannot open %s\n", csv.c_str());
        return 1;
    }
    std::fprintf(out, "# label=%s sync_reps=%zu psdu=%zu taps=%zu window=%zu\n",
                 label.c_str(), sync_reps, psdu_bytes,
                 cfg.cir_pre + cfg.cir_post, n);
    std::fprintf(out, "iter,us,status,peak_tap,tap_count,valid_reps\n");

    size_t ok = 0;
    for (size_t i = 0; i < iters; ++i) {
        gr::uwb::radar::RadarCirResult r;
        const auto t0 = std::chrono::steady_clock::now();
        const bool good = gr::uwb::radar::radar_cir_one(rx.data(), n, predicted,
                                                        cfg, scratch, r);
        const auto t1 = std::chrono::steady_clock::now();
        const double us =
            std::chrono::duration<double, std::micro>(t1 - t0).count();
        const bool match = good && r.status == ref.status &&
                           r.peak_tap == ref.peak_tap &&
                           r.tap_count == ref.tap_count;
        if (match)
            ++ok;
        std::fprintf(out, "%zu,%.3f,%d,%zu,%zu,%zu\n", i, us, int(r.status),
                     r.peak_tap, r.tap_count, r.valid_repetitions);
    }
    if (out != stdout)
        std::fclose(out);
    std::fprintf(stderr,
                 "%s: %zu/%zu iterations match the reference "
                 "(peak_tap=%zu tap_count=%zu)\n",
                 label.c_str(), ok, iters, ref.peak_tap, ref.tap_count);
    return ok == iters ? 0 : 1;
}
