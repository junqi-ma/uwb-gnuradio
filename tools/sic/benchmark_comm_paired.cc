// Paired comm sanity benchmark for the R04 rectification.
//
// Uses only the API surface present in both 921c38f and the rectified tree, so
// the same source compiles against either.  The three rectification changes do
// not target comm; this exists to show that comm stage times did not move
// obviously, not as a comm acceptance run.
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using cf = std::complex<float>;

int main(int argc, char** argv)
{
    size_t sync_reps = 64;
    size_t psdu_bytes = 32;
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
        else
            return 2;
    }

    gr::uwb::mod::HrpModConfig mcfg;
    mcfg.code_index = 10;
    mcfg.sync_repetitions = sync_reps;
    mcfg.sfd_mode = "4z2";
    mcfg.peak_amplitude = 0.8f;
    const std::vector<uint8_t> psdu =
        gr::uwb::mod::make_random_psdu(psdu_bytes - 2, 777, true);
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
    // comm's template_wf is ONE pulse-shaped SYNC symbol (1016 samples).
    sync.assign(packet.begin(), packet.begin() + gr::uwb::demod::kQm35SamplesPerSymbol);

    const size_t sps = gr::uwb::demod::kQm35SamplesPerSymbol;
    const int64_t lead = 1997;
    const size_t n = static_cast<size_t>(lead) + packet.size() + 4096;
    std::vector<cf> rx(n, cf(0.f, 0.f));
    std::copy(packet.begin(), packet.end(), rx.begin() + lead);

    auto profile = gr::uwb::demod::Qm35825Profile::Default();
    profile.code_index = 10;
    profile.preamble_repetitions = sync_reps;
    profile.cir_repetitions = sync_reps - profile.cir_skip_initial_repetitions;
    profile.timing_coarse_stride = 1;
    profile.rx_fast_sfd = true;
    profile.rx_fast_cfo_rotation = true;
    profile.rx_seed_refine_tail = true;

    gr::uwb::demod::core::DemodScratch scratch;
    scratch.reserve(n);
    const int64_t predicted = lead; // comm: predicted_start is the SYNC origin

    auto run_one = [&]() {
        return gr::uwb::demod::core::demodulate_one(rx.data(), n, profile, 0,
                                                    predicted, 0, sync, scratch);
    };

    auto ref = run_one();
    if (ref.status != gr::uwb::demod::DemodStatus::Success) {
        std::fprintf(stderr, "comm fixture does not decode: status=%d\n",
                     int(ref.status));
        return 1;
    }
    for (size_t i = 0; i < warmup; ++i)
        run_one();

    std::FILE* out = (csv == "-") ? stdout : std::fopen(csv.c_str(), "w");
    if (!out)
        return 1;
    std::fprintf(out, "# label=%s sync_reps=%zu psdu=%zu\n", label.c_str(),
                 sync_reps, psdu_bytes);
    std::fprintf(out, "iter,us,status,stage_cir_us,psdu_bytes,fcs\n");
    size_t ok = 0;
    for (size_t i = 0; i < iters; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        auto r = run_one();
        const auto t1 = std::chrono::steady_clock::now();
        const double us =
            std::chrono::duration<double, std::micro>(t1 - t0).count();
        const bool match = r.status == ref.status &&
                           r.payload.bytes == ref.payload.bytes &&
                           r.payload.fcs_pass == ref.payload.fcs_pass;
        if (match)
            ++ok;
        std::fprintf(out, "%zu,%.3f,%d,%llu,%u,%d\n", i, us, int(r.status),
                     (unsigned long long)r.stage_cir_us,
                     unsigned(r.payload.bytes.size()), int(r.payload.fcs_pass));
    }
    if (out != stdout)
        std::fclose(out);
    std::fprintf(stderr, "%s: %zu/%zu match (status=%d psdu=%u fcs=%d)\n",
                 label.c_str(), ok, iters, int(ref.status),
                 unsigned(ref.payload.bytes.size()), int(ref.payload.fcs_pass));
    return ok == iters ? 0 : 1;
}
