#include <cmath>
using std::isinf;
using std::isnan;
#include <gnuradio/uwb/uwb_kernel_backend.h>
#include <gnuradio/uwb/uwb_sic_unified.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <cstdlib>

using namespace gr::uwb;
using namespace gr::uwb::sic;
namespace simd = gr::uwb::core::simd;
using cf = std::complex<float>;

constexpr size_t kSps = 1016;
constexpr size_t kTrainLen = 8 * kSps;
constexpr double kFs = 998.4e6;

struct TestPacket {
    mod::HrpModConfig profile;
    mod::HrpModScratch mod_scratch;
    std::vector<cf> wave;
    std::vector<cf> train;
    std::vector<cf> sync;
    std::vector<uint8_t> psdu;

    explicit TestPacket(size_t code = 10, size_t psdu_len = 32)
    {
        profile.code_index = code;
        profile.sync_repetitions = 64;
        profile.sfd_mode = "4z2";
        profile.peak_amplitude = 0.8f;

        psdu = mod::make_random_psdu(psdu_len - 2, static_cast<uint32_t>(1000 + code), true);
        const size_t cap = mod::packet_samples_998p4(64, 8, psdu.size(), false, &profile.pulse) + 512;
        mod_scratch.reserve(127, cap);
        wave.resize(cap);
        size_t plen = 0;
        mod::modulate_one(psdu.data(), psdu.size(), profile,
                          mod_scratch, wave.data(), cap, plen);
        wave.resize(plen);
        train.assign(wave.begin(), wave.begin() + kTrainLen);
        sync.assign(wave.begin(), wave.begin() + kSps);
    }
};

SicUnifiedTemplates make_templates(const TestPacket& p)
{
    SicUnifiedTemplates t;
    t.sync = p.sync.data();
    t.sync_len = p.sync.size();
    t.preamble = p.wave.data();
    t.preamble_len = p.wave.size();
    return t;
}

int main() {
    // Phase-2 A/B/C/D driver: UWB_KERNEL_BACKEND="sync=avx2,cir=avx512"
    // pins one backend per kernel group so the same binary can be compared
    // against itself.  Nothing in the receiver reads this; it is a benchmark
    // and QA switch only.
    if (const char* spec = std::getenv("UWB_KERNEL_BACKEND")) {
        std::string s(spec);
        size_t pos = 0;
        while (pos <= s.size()) {
            const size_t comma = s.find(',', pos);
            const std::string item =
                s.substr(pos, comma == std::string::npos ? std::string::npos
                                                        : comma - pos);
            const size_t eq = item.find('=');
            if (eq != std::string::npos) {
                const std::string group = item.substr(0, eq);
                const std::string backend = item.substr(eq + 1);
                bool applied = false;
                for (int g = 0; g < static_cast<int>(simd::KernelGroup::Count);
                     ++g) {
                    const auto kg = static_cast<simd::KernelGroup>(g);
                    if (group == simd::kernel_group_name(kg)) {
                        simd::set_kernel_backend(kg, backend);
                        applied = true;
                        break;
                    }
                }
                if (!applied) {
                    std::cerr << "UWB_KERNEL_BACKEND: unknown group '" << group
                              << "'\n";
                    return 2;
                }
            }
            if (comma == std::string::npos)
                break;
            pos = comma + 1;
        }
    }
    std::cout << "kernel backends:";
    for (int g = 0; g < static_cast<int>(simd::KernelGroup::Count); ++g) {
        const auto kg = static_cast<simd::KernelGroup>(g);
        std::cout << " " << simd::kernel_group_name(kg) << "="
                  << simd::resolved_backend_name(kg);
    }
    std::cout << "\n";

    constexpr size_t kCode = 10;
    constexpr size_t kPsduBytes = 32;
    TestPacket pkt(kCode, kPsduBytes);
    SicUnifiedTemplates t = make_templates(pkt);

    const size_t q = 500;
    const size_t n = pkt.wave.size() + 2000;
    std::vector<cf> y(n, cf(0.f, 0.f));
    std::copy(pkt.wave.begin(), pkt.wave.end(), y.begin() + q);

    auto run_bench = [&](SicPolicy policy, const char* name) {
        SicUnifiedConfig cfg;
        cfg.policy = policy;
        cfg.backend = SicEstimatorBackend::Ipatov;
        cfg.sparse_pulse_reconstruct = true;
        cfg.tap_count = 20;
        cfg.lag_lo = -1;
        cfg.comm_code = kCode;
        cfg.samples_per_symbol = 1016;
        cfg.sample_rate = kFs;
        cfg.roi_lo = q - 32;
        cfg.roi_hi = q + 32;
        cfg.sub_roi_lo = q;
        cfg.sub_roi_hi = q + (policy == SicPolicy::PreambleOnly ? (72 * 1016) : pkt.wave.size());
        cfg.max_window_samples = n;
        cfg.max_packet_samples = n;

        SicUnifiedScratch scratch;
        std::string err;
        if (!prepare_sic_unified(cfg, t, scratch, &err)) {
            std::cerr << "Prepare failed: " << err << std::endl;
            return;
        }

        std::vector<cf> out(n);
        std::vector<cf> i_hat(n);
        SicUnifiedResult res;
        res.out = out.data();
        res.i_hat = i_hat.data();

        // Warmup
        for (int i = 0; i < 10; ++i) {
            sic_process_unified(y.data(), n, t, cfg, scratch, res);
        }

        // Measure
        constexpr int kRounds = 50;
        std::vector<double> totals;
        std::vector<double> estimates;
        std::vector<double> comms;
        std::vector<double> recons;
        std::vector<double> subs;
        std::vector<double> rx_timings;
        std::vector<double> rx_cfos;
        std::vector<double> rx_sfds;
        std::vector<double> rx_cirs;
        std::vector<double> rx_phrs;
        std::vector<double> rx_pays;

        for (int i = 0; i < kRounds; ++i) {
            const bool ok = sic_process_unified(y.data(), n, t, cfg, scratch, res);
            if (!ok || !res.committed || !std::isfinite(res.suppression_db)) {
                std::cerr << "Fatal: benchmark round " << i << " failed closed\n";
                std::exit(1);
            }
            // Strict sample-by-sample finiteness check
            for (size_t s = 0; s < n; ++s) {
                if (!std::isfinite(out[s].real()) || !std::isfinite(out[s].imag())) {
                    std::cerr << "Fatal: nonfinite sample in out at " << s << " in round " << i << "\n";
                    std::exit(1);
                }
            }
            totals.push_back(res.timings.core_total_us);
            estimates.push_back(res.timings.estimate_us);
            comms.push_back(res.timings.comm_rx_us);
            recons.push_back(res.timings.reconstruct_us);
            subs.push_back(res.timings.subtract_us);
            rx_timings.push_back(res.timings.rx_timing_us);
            rx_cfos.push_back(res.timings.rx_cfo_us);
            rx_sfds.push_back(res.timings.rx_sfd_us);
            rx_cirs.push_back(res.timings.rx_cir_us);
            rx_phrs.push_back(res.timings.rx_phr_us);
            rx_pays.push_back(res.timings.rx_payload_us);
        }

        double out_energy = 0.0;
        for (size_t s = 0; s < n; ++s) {
            out_energy += double(std::norm(out[s]));
        }
        const double out_l2 = std::sqrt(out_energy);

        const char* dump_dir = std::getenv("SIC_DUMP_OUTPUT_DIR");
        if (dump_dir) {
            std::string pol_name = (policy == SicPolicy::PreambleOnly) ? "preamble" : "fullpacket";
            std::string path = std::string(dump_dir) + "/sic_out_" + pol_name + ".bin";
            std::ofstream ofs(path, std::ios::binary);
            if (ofs) {
                ofs.write(reinterpret_cast<const char*>(out.data()), sizeof(cf) * n);
            }
        }

        auto stats = [](std::vector<double>& v) {
            std::sort(v.begin(), v.end());
            double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            double p50 = v[v.size() / 2];
            double p95 = v[static_cast<size_t>(v.size() * 0.95)];
            return std::make_tuple(mean, p50, p95);
        };

        auto [tot_m, tot_50, tot_95] = stats(totals);
        auto [est_m, est_50, est_95] = stats(estimates);
        auto [com_m, com_50, com_95] = stats(comms);
        auto [rec_m, rec_50, rec_95] = stats(recons);
        auto [sub_m, sub_50, sub_95] = stats(subs);
        auto [t_m, _, __] = stats(rx_timings);
        auto [c_m, ___, ____] = stats(rx_cfos);
        auto [s_m, _____, ______] = stats(rx_sfds);
        auto [cir_m, _______, ________] = stats(rx_cirs);
        auto [phr_m, _________, __________] = stats(rx_phrs);
        auto [pay_m, ___________, ____________] = stats(rx_pays);

        std::cout << "\n=======================================================\n";
        std::cout << "  Benchmark: " << name << "\n";
        std::cout << "  Committed: " << (res.committed ? "YES" : "NO") 
                  << " | Suppression: " << std::fixed << std::setprecision(2) << res.suppression_db << " dB\n";
        std::cout << "  Level: " << static_cast<int>(res.level) << " | Cancelled Mask: " << static_cast<uint32_t>(res.cancelled_fields) << "\n";
        std::cout << "  OUTPUT_FINITE: ALL_FINITE (n=" << n << ")\n";
        std::cout << "  OUTPUT_L2: " << std::fixed << std::setprecision(8) << out_l2 << "\n";
        std::cout << "  RAW_TOTALS:";
        for (double v : totals) std::cout << " " << v;
        std::cout << "\n";
        std::cout << "-------------------------------------------------------\n";
        std::cout << "  STAGE                    MEAN (us)    P50 (us)    P95 (us)\n";
        std::cout << "  1. estimate_channel      " << std::setw(9) << est_m << "   " << std::setw(9) << est_50 << "   " << std::setw(9) << est_95 << "\n";
        std::cout << "  2. comm_rx (demod/sfd)   " << std::setw(9) << com_m << "   " << std::setw(9) << com_50 << "   " << std::setw(9) << com_95 << "\n";
        std::cout << "     - timing              " << std::setw(9) << t_m << "\n";
        std::cout << "     - cfo                 " << std::setw(9) << c_m << "\n";
        std::cout << "     - sfd (tail refit)    " << std::setw(9) << s_m << "\n";
        std::cout << "     - cir (equalizer)     " << std::setw(9) << cir_m << "\n";
        std::cout << "     - phr                 " << std::setw(9) << phr_m << "\n";
        std::cout << "     - payload             " << std::setw(9) << pay_m << "\n";
        std::cout << "  3. reconstruct (FIR)     " << std::setw(9) << rec_m << "   " << std::setw(9) << rec_50 << "   " << std::setw(9) << rec_95 << "\n";
        std::cout << "  4. subtract & validate   " << std::setw(9) << sub_m << "   " << std::setw(9) << sub_50 << "   " << std::setw(9) << sub_95 << "\n";
        std::cout << "-------------------------------------------------------\n";
        std::cout << "  TOTAL CORE TIME          " << std::setw(9) << tot_m << "   " << std::setw(9) << tot_50 << "   " << std::setw(9) << tot_95 << "\n";
        std::cout << "=======================================================\n";
    };

    run_bench(SicPolicy::PreambleOnly, "Preamble-Only (SYNC + SFD Aligned Flow)");
    run_bench(SicPolicy::FullPacket,   "Full-Packet (Complete Frame Flow)");

    return 0;
}
