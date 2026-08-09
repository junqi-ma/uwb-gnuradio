#include <gnuradio/blocks/head.h>
#include <gnuradio/blocks/message_debug.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_detector_sc16.h>

#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
constexpr size_t kPacketStart = 4992000;
constexpr size_t kSymbolLen = 1016;

std::vector<gr_complex> load(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open " + path);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<gr_complex> x(bytes / sizeof(gr_complex));
    f.read(reinterpret_cast<char*>(x.data()), bytes);
    return x;
}

template <typename Detector, typename Source>
double run(const Source& src, const Detector& det, size_t item_size,
           uint64_t target, size_t* messages)
{
    auto head = gr::blocks::head::make(item_size, target);
    auto dbg = gr::blocks::message_debug::make();
    auto tb = gr::make_top_block("detector_format_benchmark");
    tb->connect(src, 0, head, 0);
    tb->connect(head, 0, det, 0);
    tb->msg_connect(det, "packet", dbg, "store");
    const auto t0 = std::chrono::steady_clock::now();
    tb->run();
    const double sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    *messages = dbg->num_messages();
    return sec;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: benchmark_detector_sc16 CFILE [target_samples]\n";
        return 2;
    }
    const uint64_t target = argc > 2 ? std::stoull(argv[2]) : 100000000ULL;
    auto x = load(argv[1]);
    if (x.size() <= kPacketStart + kSymbolLen)
        throw std::runtime_error("reference cfile is too short");
    std::vector<gr_complex> tmpl(x.begin() + kPacketStart,
                                 x.begin() + kPacketStart + kSymbolLen);
    gr::uwb::core::uwb_l2_normalize(tmpl);

    std::vector<int16_t> sc16(x.size() * 2);
    for (size_t i = 0; i < x.size(); ++i) {
        const float re = std::max(-32768.0f, std::min(32767.0f,
            std::round(x[i].real() * 32768.0f)));
        const float im = std::max(-32768.0f, std::min(32767.0f,
            std::round(x[i].imag() * 32768.0f)));
        sc16[2 * i] = static_cast<int16_t>(re);
        sc16[2 * i + 1] = static_cast<int16_t>(im);
    }

    auto src_f = gr::blocks::vector_source_c::make(x, true);
    auto det_f = gr::uwb::UwbDetector::make(tmpl);
    size_t n_f = 0;
    const double sec_f = run(src_f, det_f, sizeof(gr_complex), target, &n_f);

    auto src_s = gr::blocks::vector_source_s::make(sc16, true, 2);
    auto det_s = gr::uwb::UwbDetectorSc16::make(tmpl);
    size_t n_s = 0;
    const double sec_s = run(src_s, det_s, sizeof(std::complex<int16_t>),
                             target, &n_s);
    const double rate_f = target / sec_f / 1e6;
    const double rate_s = target / sec_s / 1e6;
    std::cout << "target_samples=" << target << "\n"
              << "cf32_seconds=" << sec_f << " cf32_MSps=" << rate_f
              << " packets=" << n_f << "\n"
              << "sc16_seconds=" << sec_s << " sc16_MSps=" << rate_s
              << " packets=" << n_s << "\n"
              << "speedup_sc16_over_cf32=" << rate_s / rate_f << "\n"
              << "input_bytes_ratio=0.5\n";
    return (n_f == n_s && n_f > 0) ? 0 : 3;
}
