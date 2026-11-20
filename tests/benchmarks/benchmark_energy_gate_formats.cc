#include <gnuradio/uwb/uwb_detector_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Result {
    double seconds = 0.0;
    size_t regions = 0;
    uint64_t region_samples = 0;
};

std::vector<std::complex<float>> load(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("cannot open " + path);
    const size_t bytes = static_cast<size_t>(f.tellg());
    f.seekg(0);
    std::vector<std::complex<float>> samples(bytes / sizeof(std::complex<float>));
    f.read(reinterpret_cast<char*>(samples.data()), bytes);
    return samples;
}

template <typename StateMachine, typename Sample>
Result run_gate(const std::vector<Sample>& input,
                uint64_t target,
                size_t chunk_items)
{
    StateMachine sm(/*pre_trigger=*/4216,
                    /*energy_threshold=*/1e-3f,
                    /*gate_decimation=*/100,
                    /*gate_window=*/32,
                    /*holdoff_decimated=*/8);
    Result result;
    uint64_t processed = 0;
    size_t source_offset = 0;
    const auto drain = [&]() {
        while (sm.region_ready()) {
            const auto handle = sm.take_region();
            ++result.regions;
            result.region_samples += sm.region(handle).samples.size();
            sm.release_region(handle);
        }
    };

    const auto start = std::chrono::steady_clock::now();
    while (processed < target) {
        const size_t available = input.size() - source_offset;
        const uint64_t remaining = target - processed;
        const size_t count = static_cast<size_t>(std::min<uint64_t>(
            std::min(chunk_items, available), remaining));
        sm.process(input.data() + source_offset, count, processed);
        processed += count;
        source_offset += count;
        if (source_offset == input.size())
            source_offset = 0;
        drain();
    }
    sm.flush_region();
    drain();
    result.seconds = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start)
                         .count();
    return result;
}

void print_result(const char* format,
                  const Result& result,
                  uint64_t target,
                  size_t bytes_per_sample)
{
    const double rate = static_cast<double>(target) / result.seconds / 1e6;
    const double gbps = static_cast<double>(target) * bytes_per_sample /
                        result.seconds / 1e9;
    std::cout << format << "_seconds=" << result.seconds << " " << format
              << "_MSps=" << rate << " input_GBps=" << gbps
              << " regions=" << result.regions
              << " region_samples=" << result.region_samples << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: benchmark_energy_gate_formats CFILE "
                     "[target_samples] [chunk_items] [cf32-first|sc16-first]\n";
        return 2;
    }
    const uint64_t target = argc > 2 ? std::stoull(argv[2]) : 1000000000ULL;
    const size_t chunk = argc > 3 ? std::stoull(argv[3]) : 1048576ULL;
    const bool sc16_first = argc > 4 && std::string(argv[4]) == "sc16-first";
    if (chunk == 0)
        throw std::runtime_error("chunk_items must be greater than zero");

    // Quantize once outside the timed section, then regenerate FC32 from SC16
    // so both gates see exactly the same samples and threshold scale.
    const auto file_samples = load(argv[1]);
    std::vector<std::complex<int16_t>> sc16(file_samples.size());
    std::vector<std::complex<float>> fc32(file_samples.size());
    constexpr float inv_full_scale = 1.0f / 32768.0f;
    for (size_t i = 0; i < file_samples.size(); ++i) {
        const auto quantize = [](float value) {
            return static_cast<int16_t>(std::max(
                -32768.0f, std::min(32767.0f, std::round(value * 32768.0f))));
        };
        sc16[i] = { quantize(file_samples[i].real()),
                    quantize(file_samples[i].imag()) };
        fc32[i] = { static_cast<float>(sc16[i].real()) * inv_full_scale,
                    static_cast<float>(sc16[i].imag()) * inv_full_scale };
    }

    Result result_f;
    Result result_s;
    if (sc16_first) {
        result_s = run_gate<gr::uwb::UwbDetectorStateMachineSc16>(sc16, target, chunk);
        result_f = run_gate<gr::uwb::UwbDetectorStateMachine>(fc32, target, chunk);
    } else {
        result_f = run_gate<gr::uwb::UwbDetectorStateMachine>(fc32, target, chunk);
        result_s = run_gate<gr::uwb::UwbDetectorStateMachineSc16>(sc16, target, chunk);
    }

    std::cout << "target_samples=" << target << " chunk_items=" << chunk
              << " order=" << (sc16_first ? "sc16-first" : "cf32-first")
              << " gate_only=1 detector=0\n";
    print_result("cf32", result_f, target, sizeof(std::complex<float>));
    print_result("sc16", result_s, target, sizeof(std::complex<int16_t>));
    const double rate_f = static_cast<double>(target) / result_f.seconds;
    const double rate_s = static_cast<double>(target) / result_s.seconds;
    std::cout << "speedup_sc16_over_cf32=" << rate_s / rate_f << "\n";
    const bool equal = result_f.regions == result_s.regions &&
                       result_f.region_samples == result_s.region_samples;
    std::cout << "region_results_equal=" << (equal ? 1 : 0) << "\n";
    return equal && result_f.regions > 0 ? 0 : 3;
}
