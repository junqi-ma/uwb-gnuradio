// Full-frame BPRF generator for the low-SNR evaluation matrix.
// Writes a CF32 stream at the 998.4 MS/s WORK grid with one frame per 1 ms
// slot (998400 samples), using the production modulator mod::modulate_one.
#include <gnuradio/uwb/uwb_hrp_mod_core.h>

#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using cf = std::complex<float>;

int main(int argc, char** argv)
{
    size_t packets = 20;
    size_t psdu_bytes = 127;
    size_t code = 10;
    size_t sync_reps = 256;
    std::string sfd = "4z2";
    uint32_t seed = 1;
    std::string out = "frames.cf32";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--packets") packets = std::stoul(next());
        else if (a == "--psdu-bytes") psdu_bytes = std::stoul(next());
        else if (a == "--code") code = std::stoul(next());
        else if (a == "--sync") sync_reps = std::stoul(next());
        else if (a == "--sfd") sfd = next();
        else if (a == "--seed") seed = std::stoul(next());
        else if (a == "--out") out = next();
        else {
            std::fprintf(stderr, "unknown arg %s\n", a.c_str());
            return 2;
        }
    }

    if (packets == 0 || psdu_bytes != 127 || code != 10 ||
        sync_reps != 256 || sfd != "4z2") {
        std::fprintf(stderr, "calibrated fixture requires packets>0, 127 B, code10, 256 SYNC, 4z2\n");
        return 2;
    }
    constexpr size_t kSlot = 998400; // 1 ms at 998.4 MS/s
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> byte_dist(0, 255);

    gr::uwb::mod::HrpModConfig cfg;
    cfg.code_index = code;
    cfg.sync_repetitions = sync_reps;
    cfg.sfd_mode = sfd.c_str();
    cfg.peak_amplitude = 0.8f;

    const size_t count = gr::uwb::mod::packet_samples_998p4(
        sync_reps, 8, psdu_bytes, cfg.insert_sts, &cfg.pulse);
    gr::uwb::mod::HrpModScratch scratch;
    scratch.reserve(psdu_bytes, count);

    std::vector<cf> wave(count);
    std::vector<cf> stream(kSlot * packets, cf(0.0f, 0.0f));

    for (size_t p = 0; p < packets; ++p) {
        std::vector<uint8_t> wire(psdu_bytes - 2);
        for (auto& b : wire)
            b = static_cast<uint8_t>(byte_dist(rng));
        gr::uwb::mod::append_ieee_fcs(wire);
        size_t written = 0;
        if (!gr::uwb::mod::modulate_one(wire.data(), wire.size(), cfg,
                                       scratch, wave.data(), wave.size(),
                                       written)) {
            std::fprintf(stderr, "modulate_one failed at packet %zu\n", p);
            return 1;
        }
        if (written > kSlot) {
            std::fprintf(stderr, "packet %zu too long (%zu > %zu)\n", p,
                         written, kSlot);
            return 1;
        }
        std::memcpy(&stream[p * kSlot], wave.data(),
                    written * sizeof(cf));
    }

    FILE* f = std::fopen(out.c_str(), "wb");
    if (!f) {
        std::perror(out.c_str());
        return 1;
    }
    const bool complete = std::fwrite(stream.data(), sizeof(cf), stream.size(), f) == stream.size();
    if (std::fclose(f) != 0 || !complete) return 1;
    std::printf("wrote %s: packets=%zu slot=%zu total=%zu packet_samples=%zu\n",
                out.c_str(), packets, kSlot, stream.size(), count);
    return 0;
}
