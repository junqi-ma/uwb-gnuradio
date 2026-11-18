// Offline DW3000 SC16 windows -> existing 65/48 resampler -> full C++ demod.
// No UHD device, GNU Radio scheduler, or CFO skip workaround.
#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gr::uwb;

template <class T> std::vector<T> load(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("Cannot open " + path);
    const auto bytes = f.tellg();
    if (bytes <= 0 || static_cast<size_t>(bytes) % sizeof(T))
        throw std::runtime_error("Invalid data length: " + path);
    std::vector<T> v(static_cast<size_t>(bytes) / sizeof(T));
    f.seekg(0);
    if (!f.read(reinterpret_cast<char*>(v.data()), bytes))
        throw std::runtime_error("Cannot read " + path);
    return v;
}

int main(int argc, char** argv)
try {
    if (argc != 9) {
        std::cerr << "Usage: decoder IQ STARTS TEMPLATE TAPS PREAMBLE SFD PRE_GUARD WINDOW\n";
        return 2;
    }
    const auto taps = load<float>(argv[4]);
    const auto tmpl = load<std::complex<float>>(argv[3]);
    core::RationalResampler65_48Core rs(taps);
    rs.set_fixed_phase_schedule(true);
    std::ifstream iq(argv[1], std::ios::binary), starts(argv[2]);
    if (!iq || !starts) throw std::runtime_error("Cannot open IQ or starts file");
    demod::Qm35825Profile p;
    p.code_index = 10;
    p.preamble_repetitions = std::stoul(argv[5]);
    p.cir_repetitions = p.preamble_repetitions > p.cir_skip_initial_repetitions
        ? p.preamble_repetitions - p.cir_skip_initial_repetitions : p.preamble_repetitions;
    p.sfd_mode = argv[6];
    p.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
    p.rx_fast_sfd = true;
    p.rx_fast_cfo_rotation = true;
    p.rx_seed_refine_tail = true;
    p.rx_softchip_on_demand = true;
    const int64_t pre = std::stoll(argv[7]);
    const size_t n = std::stoull(argv[8]);
    if (pre < 0 || n == 0 || n > 10000000 || p.preamble_repetitions == 0)
        throw std::runtime_error("Invalid window or preamble size");
    std::vector<std::complex<int16_t>> x(n);
    std::vector<std::complex<float>> y(rs.expected_output_length(n));
    demod::core::DemodScratch scratch;
    int64_t seed;
    size_t id = 0;
    while (starts >> seed) {
        const int64_t a = seed - pre;
        if (a < 0) {
            std::cout << "{\"id\":" << id++ << ",\"seed\":" << seed
                      << ",\"input_error\":\"window_before_file\"}" << std::endl;
            continue;
        }
        iq.clear();
        iq.seekg(a * 4);
        if (!iq.read(reinterpret_cast<char*>(x.data()), n * 4)) {
            std::cout << "{\"id\":" << id++ << ",\"seed\":" << seed
                      << ",\"input_error\":\"incomplete_window\"}" << std::endl;
            continue;
        }
        rs.reset();
        size_t made = rs.process_sc16(x.data(), n, 1.f / 32768,
                                      y.data(), y.size()).produced;
        while (made < y.size()) {
            const auto count = rs.flush(y.data() + made, y.size() - made);
            if (!count) break;
            made += count;
        }
        const auto r = demod::core::demodulate_one(
            y.data(), made, p, id, std::llround(pre * 65.0 / 48.0),
            0, tmpl, scratch);
        std::cout << std::setprecision(12)
                  << "{\"id\":" << id++ << ",\"seed\":" << seed
                  << ",\"status\":" << int(r.status)
                  << ",\"timing_ok\":" << r.timing.ok
                  << ",\"timing_metric\":" << r.timing.metric
                  << ",\"peaks\":" << r.timing.detected_peaks
                  << ",\"peaks_folded\":" << r.timing.sync_peaks_folded
                  << ",\"cir_grid_repetitions\":" << r.timing.cir_grid_repetitions
                  << ",\"cfo_hz\":" << r.cfo.cfo_hz
                  << ",\"cfo_points\":" << r.cfo.peaks_used
                  << ",\"sfd_ok\":" << r.sfd.ok
                  << ",\"phr_ok\":" << r.phr.ok
                  << ",\"length\":" << r.phr.psdu_length
                  << ",\"fcs\":" << r.payload.fcs_pass
                  << ",\"demod_us\":" << r.stage_total_us << ",\"hex\":\"";
        for (const auto b : r.payload.bytes)
            std::cout << std::hex << std::setw(2) << std::setfill('0') << int(b);
        std::cout << std::dec << "\"}" << std::endl;
    }
    if (!starts.eof()) throw std::runtime_error("Starts file must contain integer sample positions");
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
}
