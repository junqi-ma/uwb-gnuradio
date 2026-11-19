// Offline DW3000 SC16 windows -> existing 65/48 resampler -> full C++ demod.
// No UHD device, GNU Radio scheduler, or CFO skip workaround.
#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_demod_sfd.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_resample_window.h>
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
    const bool par_experiment = argc == 11;
    if (argc != 9 && !par_experiment) {
        std::cerr << "Usage: decoder IQ STARTS TEMPLATE TAPS PREAMBLE SFD PRE_GUARD WINDOW"
                     " [off|sync|sfd|both|guarded|window3|window3-sfd acquire|locked|sfd-probe]\n";
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
    bool sfd_probe = false;
    if (par_experiment) {
        const std::string par = argv[9];
        const std::string strategy = argv[10];
        if ((par != "off" && par != "sync" && par != "sfd" &&
             par != "both" && par != "guarded" && par != "window3" && par != "window3-sfd") ||
            (strategy != "acquire" && strategy != "locked" && strategy != "sfd-probe"))
            throw std::runtime_error("Invalid PAR mode or synchronization strategy");
        // Use the same detection and decode settings as the live replay.
        p.rx_timing_sparse = true;
        p.rx_fused_payload_cmf = p.rx_fused_phr_cmf = true;
        p.rx_sync_par_three_samples = par == "window3";
        p.rx_sfd_par_three_samples = p.rx_sync_par_three_samples || par == "window3-sfd";
        p.rx_par_confirm = par == "guarded" || p.rx_sfd_par_three_samples;
        p.rx_sync_par_only = par == "sync" || par == "both" || p.rx_par_confirm;
        p.rx_sfd_par_only = par == "sfd" || par == "both" || p.rx_par_confirm;
        sfd_probe = strategy == "sfd-probe";
        if (strategy == "locked") {
            p.sync_strategy = demod::SyncStrategy::LockedBounded;
            p.timing_search_margin = 0;
            p.timing_max_backtrack_symbols = 0;
        } else {
            p.timing_search_margin = 131072;
            p.timing_max_backtrack_symbols = 128;
        }
    }
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
        const size_t made = core::resample_sc16_window(rs, x.data(), n, 1.f / 32768, y);
        const int64_t work_seed = std::llround(pre * 65.0 / 48.0);
        if (sfd_probe) {
            // Inject a timing hypothesis to measure SFD false alarms explicitly.
            demod::TimingResult timing;
            const double delay = 0.5 * (static_cast<double>(taps.size()) - 1);
            timing.preamble_start_sample = std::llround((65.0 * pre + delay) / 48.0);
            timing.peak_samples.push_back(
                timing.preamble_start_sample + demod::kQm35SamplesPerSymbol - 1);
            timing.measured_period = demod::kQm35SamplesPerSymbol;
            demod::SfdResult sfd;
            demod::core::stage_sfd(y.data(), made, p, timing,
                                  demod::GetSfdSequence(p.sfd_mode), tmpl, sfd, scratch);
            std::cout << "{\"id\":" << id++ << ",\"seed\":" << seed
                      << ",\"sfd_ok\":" << sfd.ok
                      << ",\"sfd_start\":" << sfd.sfd_start_sample
                      << ",\"sfd_metric\":" << sfd.metric << "}\n";
            continue;
        }
        const auto r = demod::core::demodulate_one(
            y.data(), made, p, id, work_seed, 0, tmpl, scratch);
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
                  << ",\"demod_us\":" << r.stage_total_us;
        if (par_experiment)
            std::cout << ",\"preamble_start\":" << r.timing.preamble_start_sample
                      << ",\"sfd_start\":" << r.sfd.sfd_start_sample
                      << ",\"timing_us\":" << r.stage_timing_us
                      << ",\"sfd_us\":" << r.stage_sfd_us
                      << ",\"sfd_metric\":" << r.sfd.metric;
        std::cout << ",\"hex\":\"";
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
