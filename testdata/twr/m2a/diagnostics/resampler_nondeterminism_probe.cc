// M2-A diagnostic: is RationalResamplerLmCore bitwise reproducible?
//
// WHAT THIS MEASURES
//   Two FRESH instances of the same core, given the SAME taps and the SAME
//   input, and asked to produce the full upfirdn output.  It reports how many
//   output samples differ and by how much, per iteration.
//
// WHAT THIS DOES *NOT* MEASURE
//   It does NOT localise WHERE an uninitialised read (if any) happens.  There
//   is no read-address or source-line evidence here, only the observable
//   symptom "same input, different output, run to run".  Do not read this as
//   a root-cause proof.
//
// Build (no source-tree path baked in; pass everything on the command line):
//   c++ -std=c++17 -O2 -I gr-uwb/include \
//       testdata/twr/m2a/diagnostics/resampler_nondeterminism_probe.cc \
//       -o /tmp/rsnd -L/usr/local/lib -lvolk -lpthread -ldl -lm
// Run:
//   env -u LD_LIBRARY_PATH /tmp/rsnd <taps.f32> <L> <M> <N> <out_extra> <kernel|-> <iters>
//   e.g. /tmp/rsnd testdata/resampler_65_48/taps_quality_minorder.txt 65 48 4096 256 - 10
//
// Notes:
//   * taps are RAW little-endian float32, no header.
//   * <kernel> is one of scalar_macroblock | volk_macroblock |
//     avx2_fma_macroblock, or "-" to keep the core's default.
//   * Set MALLOC_PERTURB_ to perturb the allocator between runs; the symptom
//     changes with it, which is consistent with reading uninitialised memory
//     but is not by itself proof of where.

#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <cstdint>
#include <string>
#include <vector>

// FNV-1a over the raw output bytes: a stable digest so the caller can compare
// runs FROM DIFFERENT PROCESSES.  The symptom observed here is that two fresh
// cores agree WITHIN one process (they reuse the same heap block) but the value
// differs BETWEEN processes, which is what an uninitialised-memory read looks
// like.  Comparing within a process alone therefore under-reports it.
static uint64_t fnv1a(const std::vector<std::complex<float>>& v)
{
    uint64_t h = 1469598103934665603ull;
    const unsigned char* p = reinterpret_cast<const unsigned char*>(v.data());
    for (size_t i = 0; i < v.size() * sizeof(std::complex<float>); ++i) {
        h ^= p[i];
        h *= 1099511628211ull;
    }
    return h;
}

using gr_complex = std::complex<float>;

static std::vector<float> load_f32(const char* path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return {};
    f.seekg(0, std::ios::end);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<float> t(bytes / 4u);
    f.read(reinterpret_cast<char*>(t.data()), static_cast<std::streamsize>(bytes));
    return t;
}

template <uint32_t L, uint32_t M>
static void run_case(const std::vector<float>& taps, size_t N, size_t extra,
                     const std::string& kernel, int iters)
{
    std::vector<gr_complex> in(N);
    std::mt19937 rng(0x5EEDu);
    std::uniform_real_distribution<float> d(-0.5f, 0.5f);
    for (gr_complex& c : in)
        c = gr_complex(d(rng), d(rng));

    auto one = [&](std::vector<gr_complex>& y) {
        gr::uwb::core::RationalResamplerLmCore<L, M> core(taps);
        if (!kernel.empty())
            core.set_kernel(kernel);
        const size_t lout = decltype(core)::expected_output_length(N, core.tap_count());
        y.assign(lout + extra, gr_complex(0.0f, 0.0f));
        const auto r = core.process(in.data(), in.size(), y.data(), y.size());
        size_t produced = r.produced;
        for (;;) {
            const size_t n = core.flush(y.data() + produced, y.size() - produced);
            if (n == 0)
                break;
            produced += n;
        }
        y.resize(produced);
    };

    int worst_diff = 0;
    float worst_abs = 0.0f;
    int nonfinite = 0;
    uint64_t last_hash = 0;
    for (int it = 0; it < iters; ++it) {
        std::vector<gr_complex> a;
        std::vector<gr_complex> b;
        one(a);
        one(b);
        last_hash = fnv1a(a);
        int nd = 0;
        float mx = 0.0f;
        for (size_t i = 0; i < a.size(); ++i) {
            if (!std::isfinite(a[i].real()) || !std::isfinite(a[i].imag()))
                ++nonfinite;
            const float df = std::abs(a[i] - b[i]);
            if (df != 0.0f) {
                ++nd;
                if (df > mx)
                    mx = df;
            }
        }
        if (nd > worst_diff)
            worst_diff = nd;
        if (mx > worst_abs)
            worst_abs = mx;
    }
    std::printf("L/M=%u/%u N=%zu extra=%zu kernel=%s iters=%d "
                "worst_ndiff=%d worst_max_abs=%.3e nonfinite=%d out_hash=0x%016llx\n",
                L, M, N, extra, kernel.empty() ? "(default)" : kernel.c_str(), iters,
                worst_diff, static_cast<double>(worst_abs), nonfinite,
                static_cast<unsigned long long>(last_hash));
}

int main(int argc, char** argv)
{
    if (argc < 8) {
        std::printf("usage: %s <taps.f32> <L> <M> <N> <out_extra> <kernel|-> <iters>\n",
                    argv[0]);
        return 2;
    }
    const std::vector<float> taps = load_f32(argv[1]);
    if (taps.size() < 2u) {
        std::printf("FAIL: cannot read taps from %s\n", argv[1]);
        return 2;
    }
    const int l = std::atoi(argv[2]);
    const int m = std::atoi(argv[3]);
    const size_t N = static_cast<size_t>(std::atoll(argv[4]));
    const size_t extra = static_cast<size_t>(std::atoll(argv[5]));
    const std::string kernel = (std::string(argv[6]) == "-") ? "" : argv[6];
    const int iters = std::atoi(argv[7]);
    std::printf("taps=%s T=%zu\n", argv[1], taps.size());
    if (l == 65 && m == 48)
        run_case<65, 48>(taps, N, extra, kernel, iters);
    else if (l == 65 && m == 32)
        run_case<65, 32>(taps, N, extra, kernel, iters);
    else if (l == 48 && m == 65)
        run_case<48, 65>(taps, N, extra, kernel, iters);
    else if (l == 32 && m == 65)
        run_case<32, 65>(taps, N, extra, kernel, iters);
    else {
        std::printf("FAIL: unsupported L/M %d/%d\n", l, m);
        return 2;
    }
    return 0;
}
