/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Dual-mode RX + SIC full-window C++ QA suite:
 *   Mode A: Sensing RX (radar_cir_one / SicPlusSensing) + L2 Comm SIC
 *   Mode B: Comm RX (demodulate_one / SicPlusComm) + L1 Sensing SIC
 *
 * Covers selectable HRP codes (9..12), preamble lengths (16/64/256),
 * SFD modes ("4z2"/"ieee"), pulse shapes (Legacy/Gaussian/Blackman),
 * untriggered full-window searches, L0 bypass when interferer is absent,
 * pure-C++ 737.28 MS/s -> 998.4 MS/s SC16 pipeline execution, and
 * multi-worker bit-exact determinism.
 */

#define BOOST_TEST_MODULE qa_uwb_dual_rx_modes
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_discrete_cir_sic.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_sic_unified.h>
#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

using cf = std::complex<float>;
using sc16 = std::complex<int16_t>;

namespace mod = gr::uwb::mod;
namespace demod = gr::uwb::demod;
namespace radar = gr::uwb::radar;
namespace sic = gr::uwb::sic;
namespace pipeline = gr::uwb::pipeline;

constexpr double kWorkFs = 998400000.0;
constexpr double kNativeFs = 737280000.0;
constexpr size_t kSps = demod::kQm35SamplesPerSymbol; // 1016
constexpr size_t kNativeWinSamples = 336000;          // 336000 * 65 / 48 = 455000
constexpr size_t kWorkWinSamples = 455000;            // ~0.456 ms @ 998.4 MHz
constexpr size_t kPayloadDataBytes = 16;              // +2 FCS = 18 wire bytes

struct RxProfileSpec {
    size_t code_index;
    size_t sync_reps;
    const char* sfd_mode;
    mod::PulseSpec pulse;
};

struct ProfilePair {
    const char* name;
    RxProfileSpec sensing;
    RxProfileSpec comm;
};

mod::PulseSpec make_legacy_pulse()
{
    return mod::PulseSpec{};
}

mod::PulseSpec make_gaussian_pulse()
{
    mod::PulseSpec p;
    p.shape = mod::PulseShape::Gaussian;
    p.gaussian_sigma_ns = 2.5f;
    return p;
}

mod::PulseSpec make_blackman_pulse()
{
    mod::PulseSpec p;
    p.shape = mod::PulseShape::Blackman;
    p.blackman_bw_mhz = 200.0f;
    return p;
}

ProfilePair make_pair_1()
{
    ProfilePair pair;
    pair.name = "Pair1(Sensing:c9/64/4z2/Legacy vs Comm:c10/16/ieee/Gaussian)";
    pair.sensing = RxProfileSpec{ 9, 64, "4z2", make_legacy_pulse() };
    pair.comm = RxProfileSpec{ 10, 16, "ieee", make_gaussian_pulse() };
    return pair;
}

ProfilePair make_pair_2()
{
    ProfilePair pair;
    pair.name = "Pair2(Sensing:c11/256/ieee/Blackman vs Comm:c12/64/4z2/Blackman)";
    pair.sensing = RxProfileSpec{ 11, 256, "ieee", make_blackman_pulse() };
    pair.comm = RxProfileSpec{ 12, 64, "4z2", make_blackman_pulse() };
    return pair;
}

uint64_t fnv1a_64(const uint8_t* data, size_t len)
{
    uint64_t h = 14695981039346656037ULL;
    for (size_t i = 0; i < len; ++i) {
        h ^= static_cast<uint64_t>(data[i]);
        h *= 1099511628211ULL;
    }
    return h;
}

double compute_rms(const cf* data, size_t n)
{
    if (!data || n == 0)
        return 0.0;
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double re = static_cast<double>(data[i].real());
        const double im = static_cast<double>(data[i].imag());
        acc += re * re + im * im;
    }
    return std::sqrt(acc / static_cast<double>(n));
}

void scale_waveform(std::vector<cf>& wf, float gain)
{
    for (auto& s : wf) {
        s *= gain;
    }
}

void normalize_peak(std::vector<cf>& wf, float target_peak = 1.0f)
{
    float peak = 0.0f;
    for (const auto& s : wf) {
        peak = std::max(peak, std::abs(s));
    }
    if (peak > 0.0f) {
        scale_waveform(wf, target_peak / peak);
    }
}

std::vector<cf> l2_normalize_sync(const cf* p, size_t n)
{
    std::vector<cf> out(p, p + n);
    double e = 0.0;
    for (const auto& s : out) {
        e += static_cast<double>(s.real()) * s.real() +
             static_cast<double>(s.imag()) * s.imag();
    }
    const float inv = (e > 0.0) ? static_cast<float>(1.0 / std::sqrt(e)) : 1.0f;
    for (auto& s : out) {
        s *= inv;
    }
    return out;
}

bool modulate_prefix_iq_998p4(const RxProfileSpec& spec, std::vector<cf>& out)
{
    mod::HrpModConfig cfg;
    cfg.code_index = spec.code_index;
    cfg.sync_repetitions = spec.sync_reps;
    cfg.sfd_mode = spec.sfd_mode;
    cfg.pulse = spec.pulse;

    const size_t n_sfd = demod::GetSfdSequence(spec.sfd_mode).size();
    const size_t cap = (spec.sync_reps + n_sfd) * kSps + 1024;
    out.assign(cap, cf(0.f, 0.f));

    mod::HrpModScratch scratch;
    size_t out_n = 0;
    if (!mod::modulate_prefix_waveform(cfg, scratch, out.data(), out.size(), out_n)) {
        return false;
    }
    out.resize(out_n);
    normalize_peak(out, 1.0f);
    return out_n >= (spec.sync_reps + n_sfd) * kSps;
}

bool modulate_packet_iq_998p4(const RxProfileSpec& spec,
                              const std::vector<uint8_t>& wire_psdu,
                              std::vector<cf>& out)
{
    mod::HrpModConfig cfg;
    cfg.code_index = spec.code_index;
    cfg.sync_repetitions = spec.sync_reps;
    cfg.sfd_mode = spec.sfd_mode;
    cfg.pulse = spec.pulse;

    const size_t n_sfd = demod::GetSfdSequence(spec.sfd_mode).size();
    const size_t cap = mod::packet_samples_998p4(
        spec.sync_reps, n_sfd, wire_psdu.size(), false, &spec.pulse) + 256;
    out.assign(cap, cf(0.f, 0.f));

    mod::HrpModScratch scratch;
    size_t out_n = 0;
    if (!mod::modulate_one(wire_psdu.data(), wire_psdu.size(),
                           cfg, scratch, out.data(), out.size(), out_n)) {
        return false;
    }
    out.resize(out_n);
    return out_n > 0;
}

void add_with_cfo(std::vector<cf>& dst,
                  const std::vector<cf>& src,
                  size_t offset,
                  float rms_target,
                  size_t active_sync_samples,
                  double cfo_hz,
                  double fs)
{
    const size_t n_rms = std::min(active_sync_samples, src.size());
    const double cur_rms = compute_rms(src.data(), n_rms);
    const float gain =
        (cur_rms > 0.0) ? static_cast<float>(rms_target / cur_rms) : 1.0f;

    constexpr double kPi = 3.14159265358979323846;
    const double omega = 2.0 * kPi * cfo_hz / fs;
    const size_t end = std::min(dst.size(), offset + src.size());
    for (size_t i = offset; i < end; ++i) {
        const size_t k = i - offset;
        const double phase = omega * static_cast<double>(i);
        const cf rot(static_cast<float>(std::cos(phase)),
                     static_cast<float>(std::sin(phase)));
        dst[i] += (src[k] * gain) * rot;
    }
}

demod::Qm35825Profile make_comm_profile(const RxProfileSpec& spec,
                                        size_t timing_search_margin,
                                        size_t timing_coarse_stride)
{
    demod::Qm35825Profile prof = demod::Qm35825Profile::Default();
    prof.code_index = spec.code_index;
    prof.preamble_repetitions = spec.sync_reps;
    prof.sfd_mode = spec.sfd_mode;
    prof.pulse_center_sample = mod::pulse_center_tap(spec.pulse);
    prof.cir_skip_initial_repetitions = (spec.sync_reps <= 16) ? 2 : 10;
    prof.cir_repetitions = spec.sync_reps - prof.cir_skip_initial_repetitions;
    prof.cfo_skip_initial_repetitions = (spec.sync_reps <= 16) ? 2 : 24;
    prof.cfo_min_fit_repetitions = (spec.sync_reps <= 16) ? 8 : 32;
    prof.min_valid_peaks = 8;
    prof.timing_search_margin = timing_search_margin;
    prof.timing_coarse_stride = timing_coarse_stride;
    prof.rx_fast_sfd = true;
    prof.rx_fast_cfo_rotation = true;
    return prof;
}

radar::RadarCirConfig make_radar_profile(const RxProfileSpec& spec,
                                         size_t timing_coarse_stride,
                                         bool full_window_search)
{
    radar::RadarCirConfig cfg;
    cfg.code_index = spec.code_index;
    cfg.sync_repetitions = spec.sync_reps;
    cfg.samples_per_symbol = kSps;
    cfg.sfd_mode = spec.sfd_mode;
    cfg.cir_pre = 16;
    cfg.cir_post = 100;
    cfg.cir_skip_initial = 10;
    cfg.cir_repetitions = spec.sync_reps - cfg.cir_skip_initial;
    cfg.timing_coarse_stride = timing_coarse_stride;
    cfg.full_window_search = full_window_search;
    return cfg;
}

std::string find_testdata_file(const std::string& rel)
{
    std::vector<std::string> candidates;
#ifdef UWB_TESTDATA_DIR
    candidates.push_back(std::string(UWB_TESTDATA_DIR) + "/" + rel);
#endif
    candidates.push_back(rel);
    candidates.push_back("testdata/" + rel);
    candidates.push_back("../testdata/" + rel);
    candidates.push_back("../../testdata/" + rel);
    candidates.push_back("../../../testdata/" + rel);
    candidates.push_back("../../../../testdata/" + rel);
    for (const auto& c : candidates) {
        std::ifstream f(c, std::ios::binary);
        if (f.good())
            return c;
    }
    return {};
}

std::vector<float> load_tx_48_65_taps()
{
    const std::string path = find_testdata_file("twr/m2a/taps/tx_48_65.f32");
    BOOST_REQUIRE_MESSAGE(!path.empty(),
                          "Cannot locate twr/m2a/taps/tx_48_65.f32 in testdata");
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE(f.good());
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE(bytes > 0 && bytes % sizeof(float) == 0);
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()),
           static_cast<std::streamsize>(bytes));
    BOOST_REQUIRE(f.good());
    return taps;
}

std::vector<sc16> resample_998p4_to_737p28_sc16(const std::vector<cf>& work_iq,
                                                size_t target_sc16_len,
                                                const std::vector<float>& tx_taps)
{
    std::vector<cf> padded = work_iq;
    padded.resize(work_iq.size() + 256, cf(0.f, 0.f));

    gr::uwb::core::RationalResampler48_65Core resampler(tx_taps);
    const size_t cap = resampler.expected_output_length(padded.size()) + 64;
    std::vector<cf> native_cf(cap, cf(0.f, 0.f));
    const size_t produced =
        resampler.process(padded.data(), padded.size(), native_cf.data());
    BOOST_REQUIRE_GE(produced, target_sc16_len);

    std::vector<sc16> out(target_sc16_len);
    for (size_t i = 0; i < target_sc16_len; ++i) {
        const long re = std::lround(static_cast<double>(native_cf[i].real()) * 32767.0);
        const long im = std::lround(static_cast<double>(native_cf[i].imag()) * 32767.0);
        const int16_t ire = static_cast<int16_t>(
            std::max(-32767L, std::min(32767L, re)));
        const int16_t iim = static_cast<int16_t>(
            std::max(-32767L, std::min(32767L, im)));
        out[i] = sc16(ire, iim);
    }
    return out;
}

class TempCf32File {
public:
    explicit TempCf32File(const std::vector<cf>& data)
    {
        static std::atomic<uint64_t> seq{ 0 };
        const uint64_t id = seq.fetch_add(1, std::memory_order_relaxed);
        path_ = "/tmp/qa_uwb_dual_rx_" + std::to_string(getpid()) + "_" +
                std::to_string(id) + ".cf32";
        std::ofstream f(path_, std::ios::binary | std::ios::trunc);
        if (f) {
            f.write(reinterpret_cast<const char*>(data.data()),
                    static_cast<std::streamsize>(data.size() * sizeof(cf)));
        }
    }
    ~TempCf32File()
    {
        if (!path_.empty()) {
            std::remove(path_.c_str());
        }
    }
    TempCf32File(const TempCf32File&) = delete;
    TempCf32File& operator=(const TempCf32File&) = delete;

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

class MemorySc16Source final : public pipeline::ISc16StreamSource {
public:
    MemorySc16Source(std::vector<sc16> samples, double sample_rate)
        : samples_(std::move(samples)), sample_rate_(sample_rate)
    {
    }

    bool open() override
    {
        pos_ = 0;
        open_ = true;
        return true;
    }

    void close() override { open_ = false; }

    size_t read_samples(sc16* dest,
                        size_t max_samples,
                        pipeline::StreamReadMeta& meta) override
    {
        meta = pipeline::StreamReadMeta{};
        if (!open_ || !dest || max_samples == 0 || pos_ >= samples_.size()) {
            meta.is_eof = true;
            return 0;
        }
        const size_t n = std::min(max_samples, samples_.size() - pos_);
        std::memcpy(dest, samples_.data() + pos_, n * sizeof(sc16));
        meta.start_sample_abs = static_cast<int64_t>(pos_);
        meta.has_time_spec = true;
        meta.time_spec_s = static_cast<double>(pos_) / sample_rate_;
        pos_ += n;
        meta.is_eof = false;
        return n;
    }

    double actual_rx_rate() const override { return sample_rate_; }
    bool is_eof() const override { return pos_ >= samples_.size(); }

private:
    std::vector<sc16> samples_;
    double sample_rate_ = kNativeFs;
    size_t pos_ = 0;
    bool open_ = false;
};

struct DualWindowFixtures {
    RxProfileSpec sensing_spec;
    RxProfileSpec comm_spec;
    std::vector<cf> sensing_sync_l2;
    std::vector<cf> comm_sync_l2;
    std::vector<uint8_t> wire_psdu;
    std::vector<sc16> mode_a_stream; // [win0: Sensing+Comm, win1: SensingOnly]
    std::vector<sc16> mode_b_stream; // [win0: Comm+Sensing, win1: CommOnly]
};

DualWindowFixtures build_pair1_sc16_fixtures()
{
    const ProfilePair pair = make_pair_1();
    const auto tx_taps = load_tx_48_65_taps();

    std::vector<cf> sensing_prefix;
    BOOST_REQUIRE(modulate_prefix_iq_998p4(pair.sensing, sensing_prefix));

    const auto wire_psdu = mod::make_random_psdu(kPayloadDataBytes, 20261113u, true);
    std::vector<cf> comm_pkt;
    BOOST_REQUIRE(modulate_packet_iq_998p4(pair.comm, wire_psdu, comm_pkt));

    const float target_rms = 0.02f;
    const float sir_neg14_factor = static_cast<float>(std::pow(10.0, 14.0 / 20.0));
    const float interferer_rms = target_rms * sir_neg14_factor;

    // Mode A Windows (Target: Sensing @ 65000 +15kHz; Interferer: Comm @ 60000 -22kHz)
    std::vector<cf> a_win0(kWorkWinSamples, cf(0.f, 0.f));
    add_with_cfo(a_win0, sensing_prefix, 65000, target_rms,
                 pair.sensing.sync_reps * kSps, 15000.0, kWorkFs);
    add_with_cfo(a_win0, comm_pkt, 60000, interferer_rms,
                 pair.comm.sync_reps * kSps, -22000.0, kWorkFs);

    std::vector<cf> a_win1(kWorkWinSamples, cf(0.f, 0.f));
    add_with_cfo(a_win1, sensing_prefix, 65000, target_rms,
                 pair.sensing.sync_reps * kSps, 15000.0, kWorkFs);

    auto a_sc16_0 = resample_998p4_to_737p28_sc16(a_win0, kNativeWinSamples, tx_taps);
    auto a_sc16_1 = resample_998p4_to_737p28_sc16(a_win1, kNativeWinSamples, tx_taps);

    // Mode B Windows (Target: Comm @ 105000 +15kHz; Interferer: Sensing @ 100000 -22kHz)
    std::vector<cf> b_win0(kWorkWinSamples, cf(0.f, 0.f));
    add_with_cfo(b_win0, comm_pkt, 105000, target_rms,
                 pair.comm.sync_reps * kSps, 15000.0, kWorkFs);
    add_with_cfo(b_win0, sensing_prefix, 100000, interferer_rms,
                 pair.sensing.sync_reps * kSps, -22000.0, kWorkFs);

    std::vector<cf> b_win1(kWorkWinSamples, cf(0.f, 0.f));
    add_with_cfo(b_win1, comm_pkt, 105000, target_rms,
                 pair.comm.sync_reps * kSps, 15000.0, kWorkFs);

    auto b_sc16_0 = resample_998p4_to_737p28_sc16(b_win0, kNativeWinSamples, tx_taps);
    auto b_sc16_1 = resample_998p4_to_737p28_sc16(b_win1, kNativeWinSamples, tx_taps);

    DualWindowFixtures fix;
    fix.sensing_spec = pair.sensing;
    fix.comm_spec = pair.comm;
    fix.sensing_sync_l2 = l2_normalize_sync(sensing_prefix.data(), kSps);
    fix.comm_sync_l2 = l2_normalize_sync(comm_pkt.data(), kSps);
    fix.wire_psdu = wire_psdu;

    fix.mode_a_stream.reserve(2 * kNativeWinSamples);
    fix.mode_a_stream.insert(fix.mode_a_stream.end(), a_sc16_0.begin(), a_sc16_0.end());
    fix.mode_a_stream.insert(fix.mode_a_stream.end(), a_sc16_1.begin(), a_sc16_1.end());

    fix.mode_b_stream.reserve(2 * kNativeWinSamples);
    fix.mode_b_stream.insert(fix.mode_b_stream.end(), b_sc16_0.begin(), b_sc16_0.end());
    fix.mode_b_stream.insert(fix.mode_b_stream.end(), b_sc16_1.begin(), b_sc16_1.end());

    return fix;
}

pipeline::PipelineConfig make_mode_a_pipeline_cfg(const DualWindowFixtures& fix,
                                                  const std::string& comm_sync_path,
                                                  const std::string& radar_sync_path,
                                                  uint64_t num_windows,
                                                  int dsp_workers)
{
    pipeline::PipelineConfig cfg;
    cfg.source_type = pipeline::StreamSourceType::Replay;
    cfg.native_rate = kNativeFs;
    cfg.work_rate = kWorkFs;
    cfg.schedule_t0 = 0.0;
    cfg.period_s = static_cast<double>(kNativeWinSamples) / kNativeFs;
    cfg.pre_guard_samples = 0;
    cfg.capture_samples = kNativeWinSamples;
    cfg.post_guard_samples = 0;
    cfg.pool_capacity = 8;
    cfg.consumer_mode = pipeline::ConsumerMode::SicPlusSensing;
    cfg.resampler_profile = "realtime_minorder";
    cfg.scale_mode = pipeline::Sc16ScaleMode::UnitRange;
    cfg.dsp_workers = dsp_workers;
    cfg.max_windows = num_windows;

    cfg.sic_cfg.policy = sic::SicPolicy::AutoFallback;
    cfg.sic_cfg.backend = sic::SicEstimatorBackend::Ipatov;
    cfg.sic_cfg.comm_code = fix.comm_spec.code_index;
    cfg.sic_cfg.sync_reps = fix.comm_spec.sync_reps;
    cfg.sic_cfg.sfd_mode = fix.comm_spec.sfd_mode;
    cfg.sic_cfg.pulse = fix.comm_spec.pulse;
    cfg.sic_cfg.roi_lo = 0;
    cfg.sic_cfg.roi_hi = 0;
    cfg.sic_cfg.sub_roi_lo = 0;
    cfg.sic_cfg.sub_roi_hi = 0;
    cfg.sic_cfg.full_window_search = true;
    cfg.sic_cfg.coarse_stride = 8;
    cfg.sic_cfg.comm_timing_coarse_stride = 8;
    cfg.sic_cfg.comm_timing_search_margin = kWorkWinSamples;
    cfg.sic_cfg.max_window_samples = kWorkWinSamples + 4096;
    cfg.sic_cfg.max_packet_samples = kWorkWinSamples + 4096;
    cfg.sic_sync_template_file = comm_sync_path;

    cfg.radar_cfg = make_radar_profile(fix.sensing_spec, 8, true);
    cfg.radar_pulse = fix.sensing_spec.pulse;
    cfg.radar_full_window_search = true;
    cfg.radar_sync_template_file = radar_sync_path;

    return cfg;
}

pipeline::PipelineConfig make_mode_b_pipeline_cfg(const DualWindowFixtures& fix,
                                                  const std::string& sensing_sync_path,
                                                  const std::string& comm_sync_path,
                                                  uint64_t num_windows,
                                                  int dsp_workers)
{
    pipeline::PipelineConfig cfg;
    cfg.source_type = pipeline::StreamSourceType::Replay;
    cfg.native_rate = kNativeFs;
    cfg.work_rate = kWorkFs;
    cfg.schedule_t0 = 0.0;
    cfg.period_s = static_cast<double>(kNativeWinSamples) / kNativeFs;
    cfg.pre_guard_samples = 0;
    cfg.capture_samples = kNativeWinSamples;
    cfg.post_guard_samples = 0;
    cfg.pool_capacity = 8;
    cfg.consumer_mode = pipeline::ConsumerMode::SicPlusComm;
    cfg.resampler_profile = "realtime_minorder";
    cfg.scale_mode = pipeline::Sc16ScaleMode::UnitRange;
    cfg.dsp_workers = dsp_workers;
    cfg.max_windows = num_windows;

    cfg.sic_cfg.policy = sic::SicPolicy::PreambleOnly;
    cfg.sic_cfg.backend = sic::SicEstimatorBackend::Ipatov;
    cfg.sic_cfg.comm_code = fix.sensing_spec.code_index;
    cfg.sic_cfg.sync_reps = fix.sensing_spec.sync_reps;
    cfg.sic_cfg.sfd_mode = fix.sensing_spec.sfd_mode;
    cfg.sic_cfg.pulse = fix.sensing_spec.pulse;
    cfg.sic_cfg.roi_lo = 0;
    cfg.sic_cfg.roi_hi = 0;
    cfg.sic_cfg.sub_roi_lo = 0;
    cfg.sic_cfg.sub_roi_hi = 0;
    cfg.sic_cfg.full_window_search = true;
    cfg.sic_cfg.coarse_stride = 8;
    cfg.sic_cfg.max_window_samples = kWorkWinSamples + 4096;
    cfg.sic_cfg.max_packet_samples = kWorkWinSamples + 4096;
    cfg.sic_sync_template_file = sensing_sync_path;

    cfg.comm_cfg = make_comm_profile(fix.comm_spec, kWorkWinSamples, 8);
    cfg.comm_pulse = fix.comm_spec.pulse;
    cfg.comm_full_window_search = true;
    cfg.comm_timing_coarse_stride = 8;
    cfg.comm_sync_template_file = comm_sync_path;

    return cfg;
}

std::vector<pipeline::WindowResult> run_memory_pipeline(
    const pipeline::PipelineConfig& cfg,
    const std::vector<sc16>& stream_samples,
    pipeline::PipelineLedger* out_ledger = nullptr)
{
    pipeline::UhdSc16Pipeline pipe(cfg);
    auto src = std::make_unique<MemorySc16Source>(stream_samples, kNativeFs);
    BOOST_REQUIRE(pipe.initialize(std::move(src)));
    pipe.start();
    BOOST_REQUIRE(pipe.wait_ready(5000));
    BOOST_REQUIRE(pipe.ready_ok());
    pipe.wait();
    BOOST_REQUIRE(!pipe.failed());
    BOOST_REQUIRE(!pipe.source_failed());
    if (out_ledger) {
        *out_ledger = pipe.ledger();
    }
    return pipe.results();
}

} // namespace

BOOST_AUTO_TEST_SUITE(qa_uwb_dual_rx_modes)

// 1. Mode A: Sensing RX + L2 Comm SIC across full untriggered window (Pair 1 & Pair 2)
BOOST_AUTO_TEST_CASE(test_mode_a_sensing_rx_with_comm_sic_full_window)
{
    const ProfilePair pairs[2] = { make_pair_1(), make_pair_2() };
    const size_t sensing_offsets[2] = { 65000, 65000 };
    const size_t comm_offsets[2] = { 60000, 195000 };

    for (size_t pi = 0; pi < 2; ++pi) {
        const ProfilePair& pair = pairs[pi];
        BOOST_TEST_CONTEXT(pair.name);

        const size_t sensing_off = sensing_offsets[pi];
        const size_t comm_off = comm_offsets[pi];
        const int64_t true_sensing_sfd =
            static_cast<int64_t>(sensing_off + pair.sensing.sync_reps * kSps);

        std::vector<cf> sensing_prefix;
        BOOST_REQUIRE(modulate_prefix_iq_998p4(pair.sensing, sensing_prefix));

        const auto wire_psdu =
            mod::make_random_psdu(kPayloadDataBytes, 100u + static_cast<uint32_t>(pi), true);
        std::vector<cf> comm_pkt;
        BOOST_REQUIRE(modulate_packet_iq_998p4(pair.comm, wire_psdu, comm_pkt));

        std::vector<cf> comm_prefix;
        BOOST_REQUIRE(modulate_prefix_iq_998p4(pair.comm, comm_prefix));
        const std::vector<cf> comm_sync(comm_prefix.begin(), comm_prefix.begin() + kSps);
        const std::vector<cf> sensing_sync_l2 =
            l2_normalize_sync(sensing_prefix.data(), kSps);

        const float target_rms = 0.02f;
        const float interferer_rms =
            target_rms * static_cast<float>(std::pow(10.0, 14.0 / 20.0)); // SIR = -14 dB

        std::vector<cf> y_mixed(kWorkWinSamples, cf(0.f, 0.f));
        add_with_cfo(y_mixed, sensing_prefix, sensing_off, target_rms,
                     pair.sensing.sync_reps * kSps, 15000.0, kWorkFs);
        add_with_cfo(y_mixed, comm_pkt, comm_off, interferer_rms,
                     pair.comm.sync_reps * kSps, -22000.0, kWorkFs);

        std::vector<cf> y_sensing_only(kWorkWinSamples, cf(0.f, 0.f));
        add_with_cfo(y_sensing_only, sensing_prefix, sensing_off, target_rms,
                     pair.sensing.sync_reps * kSps, 15000.0, kWorkFs);

        // Prepare Radar CIR (full window search, coarse stride 8)
        radar::RadarCirConfig rcfg = make_radar_profile(pair.sensing, 8, true);
        radar::RadarCirCoreScratch rscratch;
        BOOST_REQUIRE(radar::prepare_radar_cir_core(
            rcfg, sensing_sync_l2.data(), kSps, rcfg.cir_pre, rcfg.cir_post, rscratch));
        radar::reserve_radar_cir_core(kWorkWinSamples, rscratch);

        // Deliberately wrong untriggered SFD seed (sensing_off away from true SFD)
        const int64_t untriggered_sfd_seed =
            static_cast<int64_t>(pair.sensing.sync_reps * kSps);

        // (a) Without SIC, strong Comm interferer degrades/fails Sensing SFD/CIR
        radar::RadarCirResult no_sic_res;
        const bool no_sic_ok = radar::radar_cir_one(
            y_mixed.data(), y_mixed.size(), untriggered_sfd_seed, rcfg, rscratch, no_sic_res);
        BOOST_CHECK(!no_sic_ok || no_sic_res.status != radar::RadarCirStatus::Ok ||
                    std::llabs(no_sic_res.sfd_start_sample - true_sensing_sfd) > 8);

        // (b) Prepare Unified SIC for Comm (AutoFallback, L2 full-packet, roi=0..0)
        sic::SicUnifiedTemplates tmpl;
        tmpl.sync = comm_sync.data();
        tmpl.sync_len = comm_sync.size();
        tmpl.preamble = comm_prefix.data();
        tmpl.preamble_len = comm_prefix.size();

        sic::SicUnifiedConfig scfg;
        scfg.policy = sic::SicPolicy::AutoFallback;
        scfg.backend = sic::SicEstimatorBackend::Ipatov;
        scfg.comm_code = pair.comm.code_index;
        scfg.sync_reps = pair.comm.sync_reps;
        scfg.sfd_mode = pair.comm.sfd_mode;
        scfg.pulse = pair.comm.pulse;
        scfg.roi_lo = 0;
        scfg.roi_hi = 0;
        scfg.sub_roi_lo = 0;
        scfg.sub_roi_hi = 0;
        scfg.full_window_search = true;
        scfg.coarse_stride = 8;
        scfg.comm_timing_coarse_stride = 8;
        scfg.comm_timing_search_margin = kWorkWinSamples;
        scfg.max_window_samples = kWorkWinSamples;
        scfg.max_packet_samples = comm_pkt.size() + 2048;

        sic::SicUnifiedScratch sscratch;
        std::string prep_err;
        BOOST_REQUIRE_MESSAGE(sic::prepare_sic_unified(scfg, tmpl, sscratch, &prep_err),
                              prep_err);

        std::vector<cf> sic_out(kWorkWinSamples, cf(0.f, 0.f));
        sic::SicUnifiedResult sic_res;
        sic_res.out = sic_out.data();

        BOOST_REQUIRE(sic::sic_process_unified(
            y_mixed.data(), y_mixed.size(), tmpl, scfg, sscratch, sic_res));
        BOOST_CHECK_MESSAGE(sic_res.committed,
                            "SIC not committed: level=" << static_cast<int>(sic_res.level)
                            << " reason=" << (sic_res.reason ? sic_res.reason : "null")
                            << " ch_status=" << (sic_res.channel.status ? sic_res.channel.status : "null")
                            << " ch_reason=" << (sic_res.channel.reason ? sic_res.channel.reason : "null")
                            << " fb_reason=" << (sic_res.fallback_reason ? sic_res.fallback_reason : "null")
                            << " phr=" << sic_res.comm_phr_pass
                            << " fcs=" << sic_res.comm_fcs_pass
                            << " q=" << sic_res.channel.q
                            << " res=" << sic_res.channel.fit_residual);
        BOOST_CHECK_MESSAGE(sic_res.level == sic::SicCancellationLevel::L2_FullPacket,
                            "unexpected level=" << static_cast<int>(sic_res.level)
                            << " fb_reason=" << (sic_res.fallback_reason ? sic_res.fallback_reason : "null")
                            << " reason=" << (sic_res.reason ? sic_res.reason : "null")
                            << " phr=" << sic_res.comm_phr_pass
                            << " fcs=" << sic_res.comm_fcs_pass
                            << " q=" << sic_res.channel.q
                            << " res=" << sic_res.channel.fit_residual);
        BOOST_CHECK(sic_res.fallback_reason == nullptr);
        BOOST_CHECK(sic_res.comm_phr_pass);
        BOOST_CHECK(sic_res.comm_fcs_pass);
        // SIR = -14 dB caps full-window cancellation at ~17.1 dB (Pair 1) and ~9.1 dB (Pair 2, 256 SYNC).
        BOOST_CHECK_GT(sic_res.suppression_db, (pair.sensing.sync_reps >= 256) ? 8.0 : 12.0);

        // (c) Absent interferer check first so clean vs post-SIC failure is clear
        std::vector<cf> bypass_out(kWorkWinSamples, cf(0.f, 0.f));
        sic::SicUnifiedResult bypass_res;
        bypass_res.out = bypass_out.data();
        (void)sic::sic_process_unified(
            y_sensing_only.data(), y_sensing_only.size(), tmpl, scfg, sscratch, bypass_res);
        BOOST_CHECK(!bypass_res.committed);
        BOOST_CHECK(bypass_res.level == sic::SicCancellationLevel::L0_Bypass);
        BOOST_CHECK_EQUAL(bypass_res.suppression_db, 0.0);
        BOOST_CHECK_EQUAL(std::memcmp(bypass_out.data(),
                                      y_sensing_only.data(),
                                      kWorkWinSamples * sizeof(cf)),
                          0);

        radar::RadarCirResult clean_res;
        const bool clean_ok = radar::radar_cir_one(
            bypass_out.data(), bypass_out.size(), untriggered_sfd_seed, rcfg, rscratch, clean_res);
        BOOST_REQUIRE_MESSAGE(clean_ok,
                              "Clean radar_cir_one failed: status="
                              << static_cast<int>(clean_res.status)
                              << " reason=" << (clean_res.status_reason ? clean_res.status_reason : "null")
                              << " sfd=" << clean_res.sfd_start_sample
                              << " sync=" << clean_res.detected_sync_origin
                              << " t_us=" << clean_res.frontend_timing_us
                              << " cfo_us=" << clean_res.frontend_cfo_us
                              << " sfd_us=" << clean_res.frontend_sfd_us
                              << " cov_us=" << clean_res.coverage_us);
        BOOST_CHECK(clean_res.status == radar::RadarCirStatus::Ok);
        BOOST_CHECK_LE(std::llabs(clean_res.sfd_start_sample - true_sensing_sfd), 2);

        // (d) Post-SIC Sensing RX recovers SFD, SYNC origin, CFO, and CIR peak
        radar::RadarCirResult post_res;
        const bool post_ok = radar::radar_cir_one(
            sic_out.data(), sic_out.size(), untriggered_sfd_seed, rcfg, rscratch, post_res);
        BOOST_REQUIRE_MESSAGE(post_ok,
                              "Post-SIC radar_cir_one failed: status="
                              << static_cast<int>(post_res.status)
                              << " reason=" << (post_res.status_reason ? post_res.status_reason : "null")
                              << " sfd=" << post_res.sfd_start_sample
                              << " sync=" << post_res.detected_sync_origin
                              << " t_us=" << post_res.frontend_timing_us
                              << " cfo_us=" << post_res.frontend_cfo_us
                              << " sfd_us=" << post_res.frontend_sfd_us
                              << " cov_us=" << post_res.coverage_us);
        BOOST_CHECK(post_res.status == radar::RadarCirStatus::Ok);
        BOOST_CHECK_LE(std::llabs(post_res.sfd_start_sample - true_sensing_sfd), 2);
        BOOST_CHECK_LE(std::llabs(post_res.detected_sync_origin -
                                  static_cast<int64_t>(sensing_off)), 2);
        BOOST_CHECK_GT(post_res.peak_abs, 0.0f);
        BOOST_CHECK_LT(std::abs(post_res.final_cfo_hz - 15000.0), 500.0);
    }
}

// 2. Mode B: Comm RX + L1 Sensing PreambleOnly SIC across full untriggered window (Pair 1 & Pair 2)
BOOST_AUTO_TEST_CASE(test_mode_b_comm_rx_with_sensing_sic_full_window)
{
    const ProfilePair pairs[2] = { make_pair_1(), make_pair_2() };
    const size_t comm_offsets[2] = { 105000, 100000 };
    const size_t sensing_offsets[2] = { 100000, 65000 };

    for (size_t pi = 0; pi < 2; ++pi) {
        const ProfilePair& pair = pairs[pi];
        BOOST_TEST_CONTEXT(pair.name);

        const size_t comm_off = comm_offsets[pi];
        const size_t sensing_off = sensing_offsets[pi];

        const auto wire_psdu =
            mod::make_random_psdu(kPayloadDataBytes, 300u + static_cast<uint32_t>(pi), true);
        std::vector<cf> comm_pkt;
        BOOST_REQUIRE(modulate_packet_iq_998p4(pair.comm, wire_psdu, comm_pkt));

        std::vector<cf> sensing_prefix;
        BOOST_REQUIRE(modulate_prefix_iq_998p4(pair.sensing, sensing_prefix));
        const std::vector<cf> sensing_sync(sensing_prefix.begin(), sensing_prefix.begin() + kSps);
        const std::vector<cf> comm_sync_l2 = l2_normalize_sync(comm_pkt.data(), kSps);

        const float target_rms = 0.02f;
        const float interferer_rms =
            target_rms * static_cast<float>(std::pow(10.0, 14.0 / 20.0)); // SIR = -14 dB

        std::vector<cf> y_mixed(kWorkWinSamples, cf(0.f, 0.f));
        add_with_cfo(y_mixed, comm_pkt, comm_off, target_rms,
                     pair.comm.sync_reps * kSps, 15000.0, kWorkFs);
        add_with_cfo(y_mixed, sensing_prefix, sensing_off, interferer_rms,
                     pair.sensing.sync_reps * kSps, -22000.0, kWorkFs);

        std::vector<cf> y_comm_only(kWorkWinSamples, cf(0.f, 0.f));
        add_with_cfo(y_comm_only, comm_pkt, comm_off, target_rms,
                     pair.comm.sync_reps * kSps, 15000.0, kWorkFs);

        // Comm demod profile: full-window untriggered seed = n/2, margin = n, stride = 8
        const demod::Qm35825Profile cprof =
            make_comm_profile(pair.comm, kWorkWinSamples, 8);
        demod::core::DemodScratch dscratch;
        dscratch.reserve(kWorkWinSamples);
        const int64_t untriggered_comm_seed = static_cast<int64_t>(kWorkWinSamples / 2);

        // (a) Without SIC, strong Sensing interferer breaks Comm demod/FCS
        const demod::DemodResult no_sic_demod = demod::core::demodulate_one(
            y_mixed.data(), y_mixed.size(), cprof, 1,
            untriggered_comm_seed, 0, comm_sync_l2, dscratch);
        BOOST_CHECK(no_sic_demod.status != demod::DemodStatus::Success ||
                    !no_sic_demod.payload.fcs_pass);

        // (b) Prepare Unified SIC for Sensing interferer (PreambleOnly, roi=0..0, coarse_stride=8)
        sic::SicUnifiedTemplates tmpl;
        tmpl.sync = sensing_sync.data();
        tmpl.sync_len = sensing_sync.size();
        tmpl.preamble = sensing_prefix.data();
        tmpl.preamble_len = sensing_prefix.size();

        sic::SicUnifiedConfig scfg;
        scfg.policy = sic::SicPolicy::PreambleOnly;
        scfg.backend = sic::SicEstimatorBackend::Ipatov;
        scfg.comm_code = pair.sensing.code_index;
        scfg.sync_reps = pair.sensing.sync_reps;
        scfg.sfd_mode = pair.sensing.sfd_mode;
        scfg.pulse = pair.sensing.pulse;
        scfg.roi_lo = 0;
        scfg.roi_hi = 0;
        scfg.sub_roi_lo = 0;
        scfg.sub_roi_hi = 0;
        scfg.full_window_search = true;
        scfg.coarse_stride = 8;
        scfg.max_window_samples = kWorkWinSamples;
        scfg.max_packet_samples = sensing_prefix.size() + 2048;

        sic::SicUnifiedScratch sscratch;
        std::string prep_err;
        BOOST_REQUIRE_MESSAGE(sic::prepare_sic_unified(scfg, tmpl, sscratch, &prep_err),
                              prep_err);

        std::vector<cf> sic_out(kWorkWinSamples, cf(0.f, 0.f));
        sic::SicUnifiedResult sic_res;
        sic_res.out = sic_out.data();

        BOOST_REQUIRE(sic::sic_process_unified(
            y_mixed.data(), y_mixed.size(), tmpl, scfg, sscratch, sic_res));
        BOOST_CHECK_MESSAGE(sic_res.committed,
                            "SIC not committed: level=" << static_cast<int>(sic_res.level)
                            << " reason=" << (sic_res.reason ? sic_res.reason : "null")
                            << " ch_status=" << (sic_res.channel.status ? sic_res.channel.status : "null")
                            << " ch_reason=" << (sic_res.channel.reason ? sic_res.channel.reason : "null")
                            << " fb_reason=" << (sic_res.fallback_reason ? sic_res.fallback_reason : "null")
                            << " phr=" << sic_res.comm_phr_pass
                            << " fcs=" << sic_res.comm_fcs_pass
                            << " q=" << sic_res.channel.q
                            << " res=" << sic_res.channel.fit_residual);
        BOOST_CHECK(sic_res.level == sic::SicCancellationLevel::L1_Preamble);
        // SIR = -14 dB caps full-window cancellation at 10*log10(1 + 10^1.4) ~= 14.34 dB.
        BOOST_CHECK_GT(sic_res.suppression_db, 12.0);

        // (c) Post-SIC Comm demod recovers exact PHR length, FCS, and PSDU bytes
        const demod::DemodResult post_demod = demod::core::demodulate_one(
            sic_out.data(), sic_out.size(), cprof, 2,
            untriggered_comm_seed, 0, comm_sync_l2, dscratch);
        BOOST_REQUIRE_MESSAGE(post_demod.status == demod::DemodStatus::Success,
                              "Post-SIC demod failed: status=" << static_cast<int>(post_demod.status)
                              << " sync=" << post_demod.timing.preamble_start_sample
                              << " sfd=" << post_demod.sfd.sfd_start_sample
                              << " phr_ok=" << post_demod.phr.ok
                              << " fcs=" << post_demod.payload.fcs_pass);
        BOOST_CHECK(post_demod.phr.ok);
        BOOST_CHECK_EQUAL(post_demod.phr.psdu_length, wire_psdu.size());
        BOOST_CHECK(post_demod.payload.fcs_pass);
        BOOST_REQUIRE_EQUAL(post_demod.payload.bytes.size(), wire_psdu.size());
        BOOST_CHECK_EQUAL_COLLECTIONS(
            post_demod.payload.bytes.begin(), post_demod.payload.bytes.end(),
            wire_psdu.begin(), wire_psdu.end());

        // (d) Absent Sensing interferer -> clean L0 bypass (bit-identical out == y)
        std::vector<cf> bypass_out(kWorkWinSamples, cf(0.f, 0.f));
        sic::SicUnifiedResult bypass_res;
        bypass_res.out = bypass_out.data();
        (void)sic::sic_process_unified(
            y_comm_only.data(), y_comm_only.size(), tmpl, scfg, sscratch, bypass_res);
        BOOST_CHECK(!bypass_res.committed);
        BOOST_CHECK(bypass_res.level == sic::SicCancellationLevel::L0_Bypass);
        BOOST_CHECK_EQUAL(bypass_res.suppression_db, 0.0);
        BOOST_CHECK_EQUAL(std::memcmp(bypass_out.data(),
                                      y_comm_only.data(),
                                      kWorkWinSamples * sizeof(cf)),
                          0);

        const demod::DemodResult clean_demod = demod::core::demodulate_one(
            bypass_out.data(), bypass_out.size(), cprof, 3,
            untriggered_comm_seed, 0, comm_sync_l2, dscratch);
        BOOST_REQUIRE(clean_demod.status == demod::DemodStatus::Success);
        BOOST_CHECK(clean_demod.payload.fcs_pass);
        BOOST_REQUIRE_EQUAL(clean_demod.payload.bytes.size(), wire_psdu.size());
        BOOST_CHECK_EQUAL_COLLECTIONS(
            clean_demod.payload.bytes.begin(), clean_demod.payload.bytes.end(),
            wire_psdu.begin(), wire_psdu.end());
    }
}

// 3. End-to-end pure C++ UhdSc16Pipeline test for Mode A (SicPlusSensing) & Mode B (SicPlusComm)
BOOST_AUTO_TEST_CASE(test_sc16_pipeline_mode_a_and_b_pure_cpp)
{
    const DualWindowFixtures fix = build_pair1_sc16_fixtures();
    const TempCf32File sensing_sync_file(fix.sensing_sync_l2);
    const TempCf32File comm_sync_file(fix.comm_sync_l2);

    const uint64_t expected_fnv_wire =
        fnv1a_64(fix.wire_psdu.data(), fix.wire_psdu.size());
    const uint64_t expected_fnv_data =
        fnv1a_64(fix.wire_psdu.data(), kPayloadDataBytes);

    // ---- Mode A: SicPlusSensing (win0 = Sensing+Comm, win1 = SensingOnly) ----
    {
        const auto cfg_a = make_mode_a_pipeline_cfg(
            fix, comm_sync_file.path(), sensing_sync_file.path(), 2, 1);
        pipeline::PipelineLedger ledger_a;
        const auto res_a = run_memory_pipeline(cfg_a, fix.mode_a_stream, &ledger_a);

        std::string inv_err;
        BOOST_CHECK_MESSAGE(ledger_a.verify_invariants(&inv_err), inv_err);
        BOOST_CHECK_EQUAL(ledger_a.offered_windows, 2u);
        BOOST_CHECK_EQUAL(ledger_a.admitted_windows, 2u);
        BOOST_CHECK_EQUAL(ledger_a.completed_windows, 2u);
        BOOST_CHECK_EQUAL(ledger_a.dropped_admission, 0u);
        BOOST_CHECK_EQUAL(ledger_a.offered_windows,
                          ledger_a.admitted_windows + ledger_a.dropped_admission +
                              ledger_a.invalid_gap_before_admission +
                              ledger_a.cancelled_windows);

        BOOST_REQUIRE_EQUAL(res_a.size(), 2u);

        // Window 0: Comm interferer present -> L2 SIC commits + Sensing succeeds
        BOOST_CHECK(res_a[0].sic_executed);
        BOOST_CHECK(res_a[0].sic_committed);
        BOOST_CHECK_GT(res_a[0].actual_suppression_db, 12.0f);
        BOOST_CHECK(res_a[0].radar_executed);
        BOOST_CHECK(res_a[0].radar_sfd_found);
        BOOST_CHECK_GT(res_a[0].radar_peak_metric, 0.0f);
        BOOST_CHECK_GT(res_a[0].radar_valid_reps, 0u);

        // Window 1: No Comm interferer -> L0 bypass + Sensing succeeds
        BOOST_CHECK(res_a[1].sic_executed);
        BOOST_CHECK(!res_a[1].sic_committed);
        BOOST_CHECK(res_a[1].radar_executed);
        BOOST_CHECK(res_a[1].radar_sfd_found);
        BOOST_CHECK_GT(res_a[1].radar_peak_metric, 0.0f);
        BOOST_CHECK_GT(res_a[1].radar_valid_reps, 0u);
    }

    // ---- Mode B: SicPlusComm (win0 = Comm+Sensing, win1 = CommOnly) ----
    {
        const auto cfg_b = make_mode_b_pipeline_cfg(
            fix, sensing_sync_file.path(), comm_sync_file.path(), 2, 1);
        pipeline::PipelineLedger ledger_b;
        const auto res_b = run_memory_pipeline(cfg_b, fix.mode_b_stream, &ledger_b);

        std::string inv_err;
        BOOST_CHECK_MESSAGE(ledger_b.verify_invariants(&inv_err), inv_err);
        BOOST_CHECK_EQUAL(ledger_b.offered_windows, 2u);
        BOOST_CHECK_EQUAL(ledger_b.admitted_windows, 2u);
        BOOST_CHECK_EQUAL(ledger_b.completed_windows, 2u);
        BOOST_CHECK_EQUAL(ledger_b.dropped_admission, 0u);
        BOOST_CHECK_EQUAL(ledger_b.offered_windows,
                          ledger_b.admitted_windows + ledger_b.dropped_admission +
                              ledger_b.invalid_gap_before_admission +
                              ledger_b.cancelled_windows);

        BOOST_REQUIRE_EQUAL(res_b.size(), 2u);

        // Window 0: Sensing interferer present -> L1 SIC commits + Comm RX decodes
        BOOST_CHECK(res_b[0].sic_executed);
        BOOST_CHECK_MESSAGE(res_b[0].sic_committed,
                            "res_b[0] sic_committed=false status="
                            << res_b[0].sic_status
                            << " comm_reason="
                            << (res_b[0].comm_status_reason ? res_b[0].comm_status_reason : "null"));
        BOOST_CHECK_GT(res_b[0].actual_suppression_db, 12.0f);
        BOOST_CHECK(res_b[0].comm_attempted);
        BOOST_CHECK(res_b[0].comm_ok);
        BOOST_CHECK(res_b[0].comm_phr_ok);
        BOOST_CHECK_EQUAL(res_b[0].comm_phr_length, fix.wire_psdu.size());
        BOOST_CHECK(res_b[0].comm_fcs_ok);
        BOOST_CHECK_GT(res_b[0].comm_payload_bytes, 0u);
        BOOST_CHECK(res_b[0].comm_psdu_fnv1a == expected_fnv_wire ||
                    res_b[0].comm_psdu_fnv1a == expected_fnv_data);

        // Window 1: No Sensing interferer -> L0 bypass + Comm RX decodes identically
        BOOST_CHECK(res_b[1].sic_executed);
        BOOST_CHECK(!res_b[1].sic_committed);
        BOOST_CHECK(res_b[1].comm_attempted);
        BOOST_CHECK_MESSAGE(res_b[1].comm_ok,
                            "res_b[1] comm_ok=false reason="
                            << (res_b[1].comm_status_reason ? res_b[1].comm_status_reason : "null")
                            << " sync=" << res_b[1].comm_sync_origin
                            << " sfd=" << res_b[1].comm_sfd_start
                            << " phr=" << res_b[1].comm_phr_ok
                            << " fcs=" << res_b[1].comm_fcs_ok);
        BOOST_CHECK(res_b[1].comm_phr_ok);
        BOOST_CHECK_EQUAL(res_b[1].comm_phr_length, fix.wire_psdu.size());
        BOOST_CHECK(res_b[1].comm_fcs_ok);
        BOOST_CHECK_EQUAL(res_b[1].comm_payload_bytes, res_b[0].comm_payload_bytes);
        BOOST_CHECK_EQUAL(res_b[1].comm_psdu_fnv1a, res_b[0].comm_psdu_fnv1a);
    }
}

// 4. Dual-mode dsp_workers=1 vs dsp_workers=2 bit-exact WindowResult determinism
BOOST_AUTO_TEST_CASE(test_sc16_pipeline_dual_mode_worker_determinism)
{
    const DualWindowFixtures fix = build_pair1_sc16_fixtures();
    const TempCf32File sensing_sync_file(fix.sensing_sync_l2);
    const TempCf32File comm_sync_file(fix.comm_sync_l2);

    // Repeat the 2-window stream twice (4 windows total) so 2 workers process in parallel
    std::vector<sc16> stream_a_4;
    stream_a_4.reserve(fix.mode_a_stream.size() * 2);
    stream_a_4.insert(stream_a_4.end(), fix.mode_a_stream.begin(), fix.mode_a_stream.end());
    stream_a_4.insert(stream_a_4.end(), fix.mode_a_stream.begin(), fix.mode_a_stream.end());

    std::vector<sc16> stream_b_4;
    stream_b_4.reserve(fix.mode_b_stream.size() * 2);
    stream_b_4.insert(stream_b_4.end(), fix.mode_b_stream.begin(), fix.mode_b_stream.end());
    stream_b_4.insert(stream_b_4.end(), fix.mode_b_stream.begin(), fix.mode_b_stream.end());

    // ---- Mode A: SicPlusSensing (N=1 vs N=2) ----
    {
        const auto res1 = run_memory_pipeline(
            make_mode_a_pipeline_cfg(fix, comm_sync_file.path(), sensing_sync_file.path(), 4, 1),
            stream_a_4);
        pipeline::PipelineLedger ledger2;
        const auto res2 = run_memory_pipeline(
            make_mode_a_pipeline_cfg(fix, comm_sync_file.path(), sensing_sync_file.path(), 4, 2),
            stream_a_4, &ledger2);

        std::string inv_err;
        BOOST_CHECK_MESSAGE(ledger2.verify_invariants(&inv_err), inv_err);
        BOOST_CHECK_EQUAL(ledger2.completed_windows, 4u);
        BOOST_REQUIRE_EQUAL(res1.size(), 4u);
        BOOST_REQUIRE_EQUAL(res2.size(), 4u);

        for (size_t k = 0; k < 4; ++k) {
            BOOST_CHECK_EQUAL(res1[k].window_id, res2[k].window_id);
            BOOST_CHECK(res1[k].state == res2[k].state);
            BOOST_CHECK_EQUAL(res1[k].native_start_abs, res2[k].native_start_abs);
            BOOST_CHECK_EQUAL(res1[k].native_sample_count, res2[k].native_sample_count);

            BOOST_CHECK_EQUAL(res1[k].sic_executed, res2[k].sic_executed);
            BOOST_CHECK_EQUAL(res1[k].sic_committed, res2[k].sic_committed);
            BOOST_CHECK_EQUAL(res1[k].actual_suppression_db, res2[k].actual_suppression_db);

            BOOST_CHECK_EQUAL(res1[k].radar_executed, res2[k].radar_executed);
            BOOST_CHECK_EQUAL(res1[k].radar_sfd_found, res2[k].radar_sfd_found);
            BOOST_CHECK_EQUAL(res1[k].radar_peak_metric, res2[k].radar_peak_metric);
            BOOST_CHECK_EQUAL(res1[k].radar_detected_start, res2[k].radar_detected_start);
            BOOST_CHECK_EQUAL(res1[k].radar_cfo_hz, res2[k].radar_cfo_hz);
            BOOST_CHECK_EQUAL(res1[k].radar_cir_origin, res2[k].radar_cir_origin);
            BOOST_CHECK_EQUAL(res1[k].radar_detected_origin, res2[k].radar_detected_origin);
            BOOST_CHECK_EQUAL(res1[k].radar_valid_reps, res2[k].radar_valid_reps);
        }
    }

    // ---- Mode B: SicPlusComm (N=1 vs N=2) ----
    {
        const auto res1 = run_memory_pipeline(
            make_mode_b_pipeline_cfg(fix, sensing_sync_file.path(), comm_sync_file.path(), 4, 1),
            stream_b_4);
        pipeline::PipelineLedger ledger2;
        const auto res2 = run_memory_pipeline(
            make_mode_b_pipeline_cfg(fix, sensing_sync_file.path(), comm_sync_file.path(), 4, 2),
            stream_b_4, &ledger2);

        std::string inv_err;
        BOOST_CHECK_MESSAGE(ledger2.verify_invariants(&inv_err), inv_err);
        BOOST_CHECK_EQUAL(ledger2.completed_windows, 4u);
        BOOST_REQUIRE_EQUAL(res1.size(), 4u);
        BOOST_REQUIRE_EQUAL(res2.size(), 4u);

        for (size_t k = 0; k < 4; ++k) {
            BOOST_CHECK_EQUAL(res1[k].window_id, res2[k].window_id);
            BOOST_CHECK(res1[k].state == res2[k].state);
            BOOST_CHECK_EQUAL(res1[k].native_start_abs, res2[k].native_start_abs);
            BOOST_CHECK_EQUAL(res1[k].native_sample_count, res2[k].native_sample_count);

            BOOST_CHECK_EQUAL(res1[k].sic_executed, res2[k].sic_executed);
            BOOST_CHECK_EQUAL(res1[k].sic_committed, res2[k].sic_committed);
            BOOST_CHECK_EQUAL(res1[k].actual_suppression_db, res2[k].actual_suppression_db);

            BOOST_CHECK_EQUAL(res1[k].comm_attempted, res2[k].comm_attempted);
            BOOST_CHECK_EQUAL(res1[k].comm_ok, res2[k].comm_ok);
            BOOST_CHECK_EQUAL(
                std::string(res1[k].comm_status_reason ? res1[k].comm_status_reason : ""),
                std::string(res2[k].comm_status_reason ? res2[k].comm_status_reason : ""));
            BOOST_CHECK_EQUAL(res1[k].comm_sync_origin, res2[k].comm_sync_origin);
            BOOST_CHECK_EQUAL(res1[k].comm_sfd_start, res2[k].comm_sfd_start);
            BOOST_CHECK_EQUAL(res1[k].comm_cfo_hz, res2[k].comm_cfo_hz);
            BOOST_CHECK_EQUAL(res1[k].comm_phr_ok, res2[k].comm_phr_ok);
            BOOST_CHECK_EQUAL(res1[k].comm_phr_length, res2[k].comm_phr_length);
            BOOST_CHECK_EQUAL(res1[k].comm_fcs_ok, res2[k].comm_fcs_ok);
            BOOST_CHECK_EQUAL(res1[k].comm_payload_bytes, res2[k].comm_payload_bytes);
            BOOST_CHECK_EQUAL(res1[k].comm_psdu_fnv1a, res2[k].comm_psdu_fnv1a);
        }
    }
}

// 5. SIC bypass transparency (Track A contract).
//
// The pipeline runs SIC with `copy_input_on_bypass = false`: on a non-committed
// result the consumer must read the ORIGINAL input, not a copy buffer.  The
// observable invariant is that a clean window produces bit-identical sensing
// results whether SIC ran and bypassed, or did not run at all.
BOOST_AUTO_TEST_CASE(test_sic_bypass_is_transparent_to_sensing)
{
    const DualWindowFixtures fix = build_pair1_sc16_fixtures();
    const TempCf32File sensing_sync_file(fix.sensing_sync_l2);
    const TempCf32File comm_sync_file(fix.comm_sync_l2);

    auto cfg_sic = make_mode_a_pipeline_cfg(fix, comm_sync_file.path(),
                                            sensing_sync_file.path(), 2, 1);
    auto cfg_raw = cfg_sic;
    cfg_raw.consumer_mode = pipeline::ConsumerMode::SensingCir;

    const auto res_sic = run_memory_pipeline(cfg_sic, fix.mode_a_stream);
    const auto res_raw = run_memory_pipeline(cfg_raw, fix.mode_a_stream);
    BOOST_REQUIRE_EQUAL(res_sic.size(), 2u);
    BOOST_REQUIRE_EQUAL(res_raw.size(), 2u);

    // Window 1 carries no interferer, so SIC must bypass (committed == false).
    BOOST_CHECK(res_sic[1].sic_executed);
    BOOST_CHECK(!res_sic[1].sic_committed);
    BOOST_CHECK(!res_raw[1].sic_executed);

    BOOST_CHECK(res_raw[1].radar_sfd_found);
    BOOST_CHECK_EQUAL(res_sic[1].radar_sfd_found, res_raw[1].radar_sfd_found);
    BOOST_CHECK_EQUAL(res_sic[1].radar_peak_metric, res_raw[1].radar_peak_metric);
    BOOST_CHECK_EQUAL(res_sic[1].radar_detected_start,
                      res_raw[1].radar_detected_start);
    BOOST_CHECK_EQUAL(res_sic[1].radar_cfo_hz, res_raw[1].radar_cfo_hz);
    BOOST_CHECK_EQUAL(res_sic[1].radar_cir_origin, res_raw[1].radar_cir_origin);
    BOOST_CHECK_EQUAL(res_sic[1].radar_detected_origin,
                      res_raw[1].radar_detected_origin);
    BOOST_CHECK_EQUAL(res_sic[1].radar_valid_reps, res_raw[1].radar_valid_reps);
}

BOOST_AUTO_TEST_SUITE_END()
