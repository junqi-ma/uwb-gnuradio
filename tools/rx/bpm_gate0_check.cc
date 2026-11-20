// Gate-0 standard-consistency check for the BPM-BPSK optimisations.
//
// On a decoded frame this tool measures two standard invariants that the
// Known-Hop / Terminated-Viterbi switches rely on:
//
//   1) the winning BPM hop of every payload symbol equals
//      `(pn[8*s] < 0)` from the BFRP scrambler, and
//   2) the last two CL-3 encoder input bits of the decoded codeword are 0
//      (the IEEE 802.15.4 zero-tail termination).
//
// It deliberately runs the four-candidate fused CMF with write_complex=true so
// the hop test is an independent argmax over the measured candidates, not the
// hypothesis that the switch assumes.
//
// Usage:
//   bpm_gate0_check FILE FORMAT START COUNT CODE PREAMBLE SFD [PREDICTED]
//     FORMAT     cf32 (complex<float>) or sc16 (complex<int16>, /32768)
//     START      first sample of the window inside FILE
//     COUNT      window length in samples
//     PREDICTED  optional window-relative seed (-1 = blind)
//
// Output: one JSON object on stdout.  Exit 0 when the frame decodes with FCS,
// 2 on input errors, 1 when the frame does not decode (still prints JSON).
#include <gnuradio/uwb/uwb_demod_payload.h>
#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_demod_sfd.h>

#include <complex>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace gr::uwb;

namespace {

std::vector<std::complex<float>> load_window(const std::string& path,
                                             const std::string& format,
                                             size_t start, size_t count)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("cannot open " + path);
    std::vector<std::complex<float>> out(count);
    if (format == "cf32") {
        in.seekg(static_cast<std::streamoff>(start) * 8);
        if (!in.read(reinterpret_cast<char*>(out.data()),
                     static_cast<std::streamsize>(count * 8)))
            throw std::runtime_error("short cf32 read");
        return out;
    }
    if (format == "sc16") {
        std::vector<std::complex<int16_t>> raw(count);
        in.seekg(static_cast<std::streamoff>(start) * 4);
        if (!in.read(reinterpret_cast<char*>(raw.data()),
                     static_cast<std::streamsize>(count * 4)))
            throw std::runtime_error("short sc16 read");
        for (size_t i = 0; i < count; ++i)
            out[i] = std::complex<float>(raw[i].real() / 32768.0f,
                                         raw[i].imag() / 32768.0f);
        return out;
    }
    throw std::runtime_error("FORMAT must be cf32 or sc16");
}

std::vector<std::complex<float>> load_cf32_vec(const std::string& path)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in)
        throw std::runtime_error("cannot open " + path);
    const auto bytes = in.tellg();
    if (bytes <= 0 || (bytes % 8) != 0)
        throw std::runtime_error("invalid cf32 length: " + path);
    std::vector<std::complex<float>> v(static_cast<size_t>(bytes) / 8);
    in.seekg(0);
    if (!in.read(reinterpret_cast<char*>(v.data()), bytes))
        throw std::runtime_error("cannot read " + path);
    return v;
}

} // namespace

int main(int argc, char** argv)
try {
    if (argc != 8 && argc != 9) {
        std::cerr << "Usage: bpm_gate0_check FILE FORMAT START COUNT CODE "
                     "PREAMBLE SFD [PREDICTED]\n";
        return 2;
    }
    const std::string path = argv[1];
    const std::string format = argv[2];
    const size_t start = std::stoull(argv[3]);
    const size_t count = std::stoull(argv[4]);
    const size_t code = std::stoull(argv[5]);
    const size_t preamble = std::stoull(argv[6]);
    const std::string sfd_mode = argv[7];
    const int64_t predicted = (argc == 9) ? std::stoll(argv[8]) : -1;

    auto rx = load_window(path, format, start, count);
    std::string tmpl_path = (code == 9)
        ? "testdata/reference_preamble.bin"
        : "testdata/reference_preamble_code10_998p4.cf32";
    if (code != 9 && code != 10)
        throw std::runtime_error("only code 9 (bin) and 10 (cf32) templates exist");
    const auto tmpl = load_cf32_vec(tmpl_path);

    demod::Qm35825Profile p = demod::Qm35825Profile::Default();
    p.code_index = code;
    p.preamble_repetitions = preamble;
    p.cir_repetitions = preamble > p.cir_skip_initial_repetitions
        ? preamble - p.cir_skip_initial_repetitions
        : preamble;
    static std::string sfd_storage;
    sfd_storage = sfd_mode;
    p.sfd_mode = sfd_storage.c_str();
    p.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
    p.rx_fast_sfd = true;
    p.rx_fast_cfo_rotation = true;
    p.rx_seed_refine_tail = true;
    p.rx_softchip_on_demand = true;
    p.rx_fused_payload_cmf = true;
    p.rx_fused_phr_cmf = true;

    demod::core::DemodScratch scratch;
    const auto res = demod::core::demodulate_one(
        rx.data(), rx.size(), p, 0, predicted, 0, tmpl, scratch);

    const bool phr_ok = res.phr.ok && scratch.fused_phr.decisions_valid &&
                        scratch.fused_phr.g0.size() == 21;

    size_t symbols = 0;
    size_t hop_checked = 0, hop_match = 0;
    size_t tail_checked = 0, tail_zero = 0;
    bool metrics_ok = false;

    if (phr_ok && res.ns_sfd.ok && res.payload.fcs_pass) {
        const size_t psdu_bits = static_cast<size_t>(res.phr.psdu_length) * 8;
        const size_t blocks = (psdu_bits + 329) / 330;
        symbols = psdu_bits + 48 * blocks;
        const size_t tap_count = res.cir.cir_filter_taps.size();
        const int64_t chip_start =
            res.timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
            static_cast<int64_t>(p.cir_pre_samples) - 1;
        const int64_t payload_start =
            res.ns_sfd.sfd_end_chip + 1 +
            static_cast<int64_t>(demod::rxfusion::kPhrSymbols) *
                static_cast<int64_t>(demod::rxfusion::kPhrChipsPerSymbol);
        const int8_t pol = (res.ns_sfd.polarity < 0) ? -1 : 1;
        // payload PN (scrambler offset 1344) reuses the fused cache.
        metrics_ok = demod::core::prepare_fused_payload_pn(
            scratch, p.code_index, symbols);
        if (metrics_ok) {
            demod::rxfusion::Metrics metrics;
            demod::rxfusion::Scratch kernel;
            const double cfo_w = 2.0 * M_PI * res.cfo.cfo_hz / p.sample_rate;
            metrics_ok = demod::rxfusion::correlate_payload_fused_cfo(
                rx.data(), rx.size(), res.cir.cir_filter_taps.data(), tap_count,
                chip_start, demod::rxfusion::kSamplesPerChip, payload_start,
                scratch.fused.pn.data(), symbols, pol, scratch.softchip.rot,
                cfo_w, metrics, kernel, nullptr, /*write_complex=*/true);
            if (metrics_ok) {
                const int8_t* pn = scratch.fused.pn.data();
                for (size_t s = 0; s < symbols; ++s) {
                    float best[2] = { 0.0f, 0.0f };
                    for (size_t h = 0; h < 2; ++h) {
                        const float a = std::abs(
                            metrics.complex_value[4 * s + 2 * h]);
                        const float b = std::abs(
                            metrics.complex_value[4 * s + 2 * h + 1]);
                        best[h] = std::max(a, b);
                    }
                    const int hop = best[0] >= best[1] ? 0 : 1;
                    const int expected = (pn[8 * s] < 0) ? 1 : 0;
                    ++hop_checked;
                    if (hop == expected)
                        ++hop_match;
                }

                // Full hard codeword [PHR | payload] and the zero tail.
                std::vector<int8_t> bits(2 * (21 + symbols));
                for (size_t s = 0; s < 21; ++s) {
                    bits[2 * s] = scratch.fused_phr.g0[s];
                    bits[2 * s + 1] = scratch.fused_phr.g1[s];
                }
                std::vector<int8_t> g0, g1;
                demod::rxfusion::decide_bpm(metrics, g0, g1);
                for (size_t s = 0; s < symbols; ++s) {
                    bits[2 * (s + 21)] = g0[s];
                    bits[2 * (s + 21) + 1] = g1[s];
                }
                std::vector<int8_t> decoded;
                demod::core::detail::vitdec_cl3(bits.data(), 21 + symbols,
                                                decoded);
                if (decoded.size() == 21 + symbols) {
                    tail_checked = 2;
                    tail_zero = (decoded[decoded.size() - 1] == 0 ? 1 : 0) +
                                (decoded[decoded.size() - 2] == 0 ? 1 : 0);
                }
            }
        }
    }

    std::cout << "{\"status\":" << int(res.status)
              << ",\"sfd_ok\":" << res.sfd.ok
              << ",\"phr_ok\":" << res.phr.ok
              << ",\"fcs\":" << res.payload.fcs_pass
              << ",\"length\":" << res.phr.psdu_length
              << ",\"cfo_hz\":" << res.cfo.cfo_hz
              << ",\"symbols\":" << symbols
              << ",\"hop_checked\":" << hop_checked
              << ",\"hop_match\":" << hop_match
              << ",\"metrics_ok\":" << metrics_ok
              << ",\"tail_checked\":" << tail_checked
              << ",\"tail_zero\":" << tail_zero << "}\n";
    return res.payload.fcs_pass ? 0 : 1;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
}
