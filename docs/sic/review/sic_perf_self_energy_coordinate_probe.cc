// Review-only probe for PERF-R01; not a production block or a replacement QA.
// Compile from the worktree root as documented in the rectification report.
// Including the implementation exposes its anonymous-namespace helpers here.
#include "../../../gr-uwb/lib/uwb_preamble_sic_core.cc"
#include <cstdio>
#include <random>

int main()
{
    using namespace gr::uwb::sic;
    const size_t L = 1016, M = 8 * L, N = 64 * L + 100;
    std::vector<std::complex<float>> sync(L), pre(N), a(M), b(M);
    std::mt19937 gen(42);
    std::uniform_real_distribution<float> rnd(-1, 1);
    for (auto& x : sync)
        x = {rnd(gen), rnd(gen)};
    for (size_t i = 0; i < N; ++i)
        pre[i] = sync[i % L];
    SicConfig cfg;
    cfg.samples_per_symbol = L;
    cfg.sync_reps = 64;
    cfg.fit_reps = 8;
    SicScratch scratch;
    std::string err;
    if (!prepare_sic_core(cfg, sync.data(), L, pre.data(), N, scratch, &err)) {
        std::puts(err.c_str());
        return 1;
    }
    for (int64_t origin : {int64_t(1997), int64_t(122000),
                           int64_t(2000000), int64_t(16777216)}) {
        size_t byte_diff = 0, energy_diff = 0;
        double worst = 0;
        for (int rel = -4; rel < 104; ++rel) {
            for (int oi = 0; oi <= 20; ++oi) {
                const double delta = -0.5 + 0.05 * oi;
                build_shifted_f(a.data(), M, scratch.pre_up.data(), N,
                                0, rel + delta);
                build_shifted_f(b.data(), M, scratch.pre_up.data(), N,
                                origin, double(origin + rel) + delta);
                byte_diff += std::memcmp(a.data(), b.data(), M * sizeof(a[0])) != 0;
                std::complex<float> ea, eb;
                volk_32fc_x2_conjugate_dot_prod_32fc_u(
                    &ea, a.data(), a.data(), M);
                volk_32fc_x2_conjugate_dot_prod_32fc_u(
                    &eb, b.data(), b.data(), M);
                energy_diff += ea.real() != eb.real();
                worst = std::max(worst, std::abs(double(ea.real()) - eb.real()));
            }
        }
        std::printf("origin=%lld candidates=2268 template_byte_diff=%zu "
                    "energy_diff=%zu max_energy_diff=%.9g\n",
                    static_cast<long long>(origin), byte_diff, energy_diff, worst);
    }
    // This characterizes coordinate rounding, not the repaired cache's gate.
    // A repaired implementation may still print differences but correctly
    // reject those cache hits; formal QA must verify that gate independently.
    return 0;
}
