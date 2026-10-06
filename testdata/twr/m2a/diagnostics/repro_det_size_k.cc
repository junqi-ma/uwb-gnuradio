// M2-A diagnostic (REPRODUCING): the resampler core's SIMD path is not
// bitwise reproducible, and the symptom is sensitive to the program's heap
// allocation pattern.
//
// This is the archived form of the probe that first reproduced it.  It builds
// TWO fresh cores with the SAME taps and the SAME input and compares their full
// upfirdn outputs; run it several times and the `ndiff` alternates.
//
// It does NOT localise the read (no read address / source line).  See README.md
// for the exact observations, environment and commands, and for the third probe
// that did NOT reproduce under a different allocation pattern.
//
// Build:
//   c++ -std=c++17 -O2 -I gr-uwb/include \
//       testdata/twr/m2a/diagnostics/repro_det_size_k.cc \
//       -o /tmp/repro_size -L/usr/local/lib -lvolk -lpthread -ldl -lm
// Run (repeat ~10x; scalar must stay 0):
//   env -u LD_LIBRARY_PATH /tmp/repro_size <taps.f32> <out_extra> <kernel|->

#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

using gr_complex = std::complex<float>;

static std::vector<float> load_f32(const char* p)
{
    std::ifstream f(p, std::ios::binary);
    if (!f)
        return {};
    f.seekg(0, std::ios::end);
    const size_t n = static_cast<size_t>(f.tellg()) / 4u;
    f.seekg(0);
    std::vector<float> t(n);
    f.read(reinterpret_cast<char*>(t.data()), static_cast<std::streamsize>(n * 4u));
    return t;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: %s <taps.f32> [out_extra=256] [kernel|-]\n", argv[0]);
        return 2;
    }
    const std::vector<float> taps = load_f32(argv[1]);
    if (taps.size() < 2u) {
        std::printf("FAIL: cannot read taps from %s\n", argv[1]);
        return 2;
    }
    const size_t extra = argc > 2 ? static_cast<size_t>(std::atoi(argv[2])) : 256u;
    const std::string kernel = argc > 3 ? argv[3] : "";
    using Core = gr::uwb::core::RationalResampler65_48Core;

    const size_t N = 4096;
    std::vector<gr_complex> in(N);
    std::mt19937 rng(0x5EED);
    std::uniform_real_distribution<float> d(-0.5f, 0.5f);
    for (gr_complex& c : in)
        c = gr_complex(d(rng), d(rng));

    auto one = [&](std::vector<gr_complex>& y) {
        Core c(taps);
        if (!kernel.empty() && kernel != "-")
            c.set_kernel(kernel);
        const size_t lout = Core::expected_output_length(N, c.tap_count());
        y.assign(lout + extra, gr_complex(0.0f, 0.0f));
        const auto r = c.process(in.data(), in.size(), y.data(), y.size());
        size_t p = r.produced;
        for (;;) {
            const size_t n = c.flush(y.data() + p, y.size() - p);
            if (n == 0)
                break;
            p += n;
        }
        y.resize(p);
    };

    std::vector<gr_complex> a;
    std::vector<gr_complex> b;
    one(a);
    one(b);
    int nd = 0;
    float mx = 0.0f;
    for (size_t i = 0; i < a.size(); ++i) {
        const float df = std::abs(a[i] - b[i]);
        if (df != 0.0f) {
            ++nd;
            if (df > mx)
                mx = df;
        }
    }
    std::printf("extra=%zu kernel=%s ndiff=%d max_abs=%.3e\n", extra,
                kernel.empty() ? "(default)" : kernel.c_str(), nd,
                static_cast<double>(mx));
    return 0;
}
