/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * End-to-end pure C++ UhdSc16Pipeline QA under ScheduleMode::AutoAcquire:
 *   - Mode A (SicPlusSensing): 737.28M -> 998.4M, +120 ppm SFO drift,
 *     interference-aware asymmetric pre/post guards, early Comm interferer
 *     (SFD + pre-SFD preamble in pre_guard), late Comm interferer (SFD in
 *     post_guard), dsp_workers=1 vs dsp_workers=2 determinism.
 *   - Mode B (SicPlusComm): 737.28M -> 998.4M, -100 ppm SFO drift, early &
 *     late Sensing interferers cancelled cleanly with 100% PSDU/FCS recovery.
 *   - Stream discontinuity / overflow gap -> re-acquire -> re-lock + ledger
 *     conservation invariants.
 */

#define BOOST_TEST_MODULE qa_uwb_uhd_sc16_pipeline_auto
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_auto_schedule_controller.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
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
constexpr size_t kPayloadDataBytes = 16;

struct RxProfileSpec {
    size_t code_index;
    size_t sync_reps;
    const char* sfd_mode;
    mod::PulseSpec pulse;
};

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
                  int64_t offset,
                  float rms_target,
                  size_t active_sync_samples,
                  double cfo_hz,
                  double fs)
{
    if (offset < 0 || static_cast<size_t>(offset) >= dst.size())
        return;
    const size_t off = static_cast<size_t>(offset);
    const size_t n_rms = std::min(active_sync_samples, src.size());
    const double cur_rms = compute_rms(src.data(), n_rms);
    const float gain =
        (cur_rms > 0.0) ? static_cast<float>(rms_target / cur_rms) : 1.0f;

    constexpr double kPi = 3.14159265358979323846;
    const double omega = 2.0 * kPi * cfo_hz / fs;
    const size_t end = std::min(dst.size(), off + src.size());
    for (size_t i = off; i < end; ++i) {
        const size_t k = i - off;
        const double phase = omega * static_cast<double>(i);
        const cf rot(static_cast<float>(std::cos(phase)),
                     static_cast<float>(std::sin(phase)));
        dst[i] += (src[k] * gain) * rot;
    }
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

class TempCf32File {
public:
    explicit TempCf32File(const std::vector<cf>& data)
    {
        static std::atomic<uint64_t> seq{ 0 };
        const uint64_t id = seq.fetch_add(1, std::memory_order_relaxed);
        path_ = "/tmp/qa_uwb_auto_pipe_" + std::to_string(getpid()) + "_" +
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

/**
 * Memory source that injects one hardware-style overflow gap at `gap_after_samples`
 * (skipping `gap_samples` from the RX timeline before continuing on post_samples).
 */
class GapMemorySc16Source final : public pipeline::ISc16StreamSource {
public:
    GapMemorySc16Source(std::vector<sc16> pre_samples,
                        size_t gap_samples,
                        std::vector<sc16> post_samples,
                        double sample_rate)
        : pre_(std::move(pre_samples)),
          gap_samples_(gap_samples),
          post_(std::move(post_samples)),
          sample_rate_(sample_rate)
    {
    }

    bool open() override
    {
        phase_ = 0;
        pos_ = 0;
        abs_cursor_ = 0;
        open_ = true;
        return true;
    }

    void close() override { open_ = false; }

    size_t read_samples(sc16* dest,
                        size_t max_samples,
                        pipeline::StreamReadMeta& meta) override
    {
        meta = pipeline::StreamReadMeta{};
        if (!open_ || !dest || max_samples == 0) {
            meta.is_eof = true;
            return 0;
        }

        if (phase_ == 0) {
            if (pos_ < pre_.size()) {
                const size_t n = std::min(max_samples, pre_.size() - pos_);
                std::memcpy(dest, pre_.data() + pos_, n * sizeof(sc16));
                meta.start_sample_abs = abs_cursor_;
                meta.has_time_spec = true;
                meta.time_spec_s = static_cast<double>(abs_cursor_) / sample_rate_;
                pos_ += n;
                abs_cursor_ += static_cast<int64_t>(n);
                return n;
            }
            phase_ = 1;
            pos_ = 0;
        }

        if (phase_ == 1) {
            // Deliver first post-gap chunk with is_gap=true + gap_samples_.
            if (pos_ >= post_.size()) {
                meta.is_eof = true;
                return 0;
            }
            abs_cursor_ += static_cast<int64_t>(gap_samples_);
            const size_t n = std::min(max_samples, post_.size() - pos_);
            std::memcpy(dest, post_.data() + pos_, n * sizeof(sc16));
            meta.is_gap = true;
            meta.gap_samples = gap_samples_;
            meta.start_sample_abs = abs_cursor_;
            meta.has_time_spec = true;
            meta.time_spec_s = static_cast<double>(abs_cursor_) / sample_rate_;
            pos_ += n;
            abs_cursor_ += static_cast<int64_t>(n);
            phase_ = 2;
            return n;
        }

        if (pos_ < post_.size()) {
            const size_t n = std::min(max_samples, post_.size() - pos_);
            std::memcpy(dest, post_.data() + pos_, n * sizeof(sc16));
            meta.start_sample_abs = abs_cursor_;
            meta.has_time_spec = true;
            meta.time_spec_s = static_cast<double>(abs_cursor_) / sample_rate_;
            pos_ += n;
            abs_cursor_ += static_cast<int64_t>(n);
            return n;
        }

        meta.is_eof = true;
        return 0;
    }

    double actual_rx_rate() const override { return sample_rate_; }
    bool is_eof() const override { return phase_ >= 1 && pos_ >= post_.size(); }

private:
    std::vector<sc16> pre_;
    size_t gap_samples_ = 0;
    std::vector<sc16> post_;
    double sample_rate_ = kNativeFs;
    int phase_ = 0;
    size_t pos_ = 0;
    int64_t abs_cursor_ = 0;
    bool open_ = false;
};

struct InterfererBurstSpec {
    bool present = false;
    int64_t offset_work_rel_target = 0; // negative = arrives before target
    float rms = 0.0f;
    double cfo_hz = 0.0;
};

/**
 * Render one target + optional interferer burst into a native 737.28 MS/s
 * stream with exact 48/65 phase alignment.
 */
void stamp_burst_737_sc16(std::vector<sc16>& native_stream,
                          double target_native_exact,
                          const std::vector<cf>& target_wf,
                          size_t target_sync_work,
                          float target_rms,
                          double target_cfo_hz,
                          const InterfererBurstSpec& int_spec,
                          const std::vector<cf>& int_wf,
                          size_t int_sync_work,
                          const std::vector<float>& tx_taps)
{
    constexpr int64_t kLeadNative48 = 48 * 700; // 33,600 native = 45,500 work lead
    const int64_t t_floor_48 =
        (static_cast<int64_t>(std::floor(target_native_exact)) / 48) * 48;
    const int64_t base_native = std::max<int64_t>(0, t_floor_48 - kLeadNative48);
    const int64_t target_off_work = static_cast<int64_t>(std::llround(
        (target_native_exact - static_cast<double>(base_native)) *
        (kWorkFs / kNativeFs)));

    int64_t max_end_work =
        target_off_work + static_cast<int64_t>(target_wf.size()) + 1024;
    if (int_spec.present) {
        max_end_work = std::max<int64_t>(
            max_end_work,
            target_off_work + int_spec.offset_work_rel_target +
                static_cast<int64_t>(int_wf.size()) + 1024);
    }
    const size_t work_len = static_cast<size_t>(std::max<int64_t>(4096, max_end_work));
    std::vector<cf> work_buf(work_len, cf(0.f, 0.f));

    add_with_cfo(work_buf, target_wf, target_off_work, target_rms,
                 target_sync_work, target_cfo_hz, kWorkFs);
    if (int_spec.present) {
        add_with_cfo(work_buf, int_wf,
                     target_off_work + int_spec.offset_work_rel_target,
                     int_spec.rms, int_sync_work, int_spec.cfo_hz, kWorkFs);
    }

    gr::uwb::core::RationalResampler48_65Core resampler(tx_taps);
    std::vector<cf> native_cf(resampler.expected_output_length(work_buf.size()) + 64,
                              cf(0.f, 0.f));
    const size_t produced =
        resampler.process(work_buf.data(), work_buf.size(), native_cf.data());

    for (size_t i = 0; i < produced; ++i) {
        const size_t dst_idx = static_cast<size_t>(base_native) + i;
        if (dst_idx >= native_stream.size())
            break;
        const long re = std::lround(static_cast<double>(native_cf[i].real()) * 32767.0);
        const long im = std::lround(static_cast<double>(native_cf[i].imag()) * 32767.0);
        const int32_t sum_re = static_cast<int32_t>(native_stream[dst_idx].real()) +
                               static_cast<int32_t>(re);
        const int32_t sum_im = static_cast<int32_t>(native_stream[dst_idx].imag()) +
                               static_cast<int32_t>(im);
        native_stream[dst_idx] = sc16(
            static_cast<int16_t>(std::max(-32767, std::min(32767, sum_re))),
            static_cast<int16_t>(std::max(-32767, std::min(32767, sum_im))));
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

std::vector<pipeline::WindowResult> run_memory_pipeline(
    const pipeline::PipelineConfig& cfg,
    std::unique_ptr<pipeline::ISc16StreamSource> src,
    pipeline::PipelineLedger* out_ledger = nullptr,
    pipeline::AutoScheduleStats* out_sched_stats = nullptr)
{
    pipeline::UhdSc16Pipeline pipe(cfg);
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
    if (out_sched_stats) {
        *out_sched_stats = pipe.auto_schedule_stats();
    }
    return pipe.results();
}

} // namespace

BOOST_AUTO_TEST_SUITE(qa_uwb_uhd_sc16_pipeline_auto)

BOOST_AUTO_TEST_CASE(test_auto_schedule_mode_a_early_and_late_comm_interference)
{
    const RxProfileSpec sensing_spec{ 9, 64, "4z2", mod::PulseSpec{} };
    mod::PulseSpec comm_pulse;
    comm_pulse.shape = mod::PulseShape::Gaussian;
    comm_pulse.gaussian_sigma_ns = 2.5f;
    const RxProfileSpec comm_spec{ 10, 16, "4z2", comm_pulse };

    const auto tx_taps = load_tx_48_65_taps();
    std::vector<cf> sensing_prefix;
    BOOST_REQUIRE(modulate_prefix_iq_998p4(sensing_spec, sensing_prefix));
    const auto wire_psdu = mod::make_random_psdu(kPayloadDataBytes, 202611131u, true);
    std::vector<cf> comm_pkt;
    BOOST_REQUIRE(modulate_packet_iq_998p4(comm_spec, wire_psdu, comm_pkt));

    const TempCf32File sensing_sync_file(l2_normalize_sync(sensing_prefix.data(), kSps));
    const TempCf32File comm_sync_file(l2_normalize_sync(comm_pkt.data(), kSps));

    constexpr size_t kNumPkts = 16;
    constexpr double kNominalPeriodS = 0.0008; // 800 us -> 589,824 native samples
    constexpr double kSfoFactor = 1.0 + 120e-6; // +120 ppm crystal drift
    constexpr double kTruePeriodNative = kNominalPeriodS * kNativeFs * kSfoFactor;
    constexpr double kT0Native = 85000.0;

    // Signal level in SC16 full-scale: target RMS ~0.075 (SNR >> 25 dB, energy gate
    // crosses default 0.004f cleanly), interferer +12 dB stronger (SIR = -12 dB).
    const float target_rms = 0.075f;
    const float int_rms = target_rms * static_cast<float>(std::pow(10.0, 12.0 / 20.0));

    const size_t total_native =
        static_cast<size_t>(std::ceil(kT0Native + kNumPkts * kTruePeriodNative)) +
        200000;
    std::vector<sc16> stream(total_native, sc16(0, 0));

    // Interference placement on post-acquisition scheduled windows (k >= 1):
    //   k = 1..5, 11..15: clean (establishes OLS/PLL lock)
    //   k = 6, 7: early Comm interferer starting -20,000 work samples before
    //             target -> its SFD & 16 pre-SFD SYNC sit inside pre_guard
    //   k = 8, 9: late Comm interferer starting near target tail (+62,000 work
    //             samples into target) -> its SFD lands inside post_guard
    //   k = 10: clean
    for (size_t k = 0; k < kNumPkts; ++k) {
        InterfererBurstSpec ib;
        if (k == 6 || k == 7) {
            ib.present = true;
            ib.offset_work_rel_target = -20000;
            ib.rms = int_rms;
            ib.cfo_hz = -21000.0;
        } else if (k == 8 || k == 9) {
            ib.present = true;
            ib.offset_work_rel_target = 62000; // near end of 73k target -> SFD in post_guard
            ib.rms = int_rms;
            ib.cfo_hz = -19000.0;
        }
        stamp_burst_737_sc16(
            stream,
            kT0Native + static_cast<double>(k) * kTruePeriodNative,
            sensing_prefix,
            sensing_spec.sync_reps * kSps,
            target_rms,
            14000.0,
            ib,
            comm_pkt,
            comm_spec.sync_reps * kSps,
            tx_taps);
    }

    auto make_cfg = [&](int workers) {
        pipeline::PipelineConfig cfg;
        cfg.source_type = pipeline::StreamSourceType::Replay;
        cfg.native_rate = kNativeFs;
        cfg.work_rate = kWorkFs;
        cfg.period_s = kNominalPeriodS;
        cfg.pool_capacity = 8;
        cfg.consumer_mode = pipeline::ConsumerMode::SicPlusSensing;
        cfg.resampler_profile = "realtime_minorder";
        cfg.scale_mode = pipeline::Sc16ScaleMode::UnitRange;
        cfg.dsp_workers = workers;
        cfg.max_windows = kNumPkts;

        cfg.schedule_mode = pipeline::ScheduleMode::AutoAcquire;
        cfg.auto_schedule.period_s = kNominalPeriodS;
        cfg.auto_schedule.max_period_rel_err = 0.02;
        cfg.auto_schedule.energy_threshold = 0.002f;
        cfg.auto_schedule.verifier_coarse_stride = 1;
        cfg.auto_schedule.lock_observations = 6;
        cfg.auto_schedule.provisional_guard_us = 25.0;
        cfg.auto_schedule.auto_interference_guards = true;
        cfg.auto_schedule.early_int_pre_sfd_sync_reps = 16;

        cfg.sic_cfg.policy = sic::SicPolicy::AutoFallback;
        cfg.sic_cfg.backend = sic::SicEstimatorBackend::Ipatov;
        cfg.sic_cfg.comm_code = comm_spec.code_index;
        cfg.sic_cfg.sync_reps = comm_spec.sync_reps;
        cfg.sic_cfg.sfd_mode = comm_spec.sfd_mode;
        cfg.sic_cfg.pulse = comm_spec.pulse;
        cfg.sic_cfg.full_window_search = true;
        cfg.sic_cfg.coarse_stride = 8;
        cfg.sic_cfg.comm_timing_coarse_stride = 8;
        cfg.sic_sync_template_file = comm_sync_file.path();

        cfg.radar_cfg = make_radar_profile(sensing_spec, 1, false);
        cfg.radar_pulse = sensing_spec.pulse;
        cfg.radar_sync_template_file = sensing_sync_file.path();
        return cfg;
    };

    pipeline::PipelineLedger ledger1, ledger2;
    pipeline::AutoScheduleStats stats1, stats2;
    const auto res1 = run_memory_pipeline(
        make_cfg(1),
        std::make_unique<MemorySc16Source>(stream, kNativeFs),
        &ledger1, &stats1);
    const auto res2 = run_memory_pipeline(
        make_cfg(2),
        std::make_unique<MemorySc16Source>(stream, kNativeFs),
        &ledger2, &stats2);

    std::string inv_err;
    BOOST_CHECK_MESSAGE(ledger1.verify_invariants(&inv_err), inv_err);
    BOOST_CHECK_MESSAGE(ledger2.verify_invariants(&inv_err), inv_err);
    BOOST_CHECK_EQUAL(ledger1.completed_windows, kNumPkts);
    BOOST_CHECK_EQUAL(ledger2.completed_windows, kNumPkts);
    BOOST_REQUIRE_EQUAL(res1.size(), kNumPkts);
    BOOST_REQUIRE_EQUAL(res2.size(), kNumPkts);

    BOOST_CHECK(stats1.enabled);
    BOOST_CHECK_EQUAL(stats1.acq_windows, 1u);
    BOOST_CHECK_EQUAL(stats1.scheduled_windows, kNumPkts - 1);
    BOOST_CHECK_EQUAL(stats1.final_state, "locked");
    BOOST_CHECK_EQUAL(stats1.discontinuities, 0u);
    BOOST_CHECK_EQUAL(stats1.stale_feedback, 0u);
    BOOST_CHECK_CLOSE(stats1.period_samples, kTruePeriodNative, 0.005); // < 50 ppm

    // All windows must succeed for Sensing CIR.
    size_t committed_count = 0;
    for (size_t k = 0; k < kNumPkts; ++k) {
        BOOST_CHECK_MESSAGE(res1[k].radar_executed && res1[k].radar_sfd_found,
                            "Mode A window " << k << " sensing failed (kind="
                                             << static_cast<int>(res1[k].window_kind)
                                             << ", sic=" << res1[k].sic_status << ")");
        BOOST_CHECK_GT(res1[k].radar_valid_reps, 0u);
        if (k == 6 || k == 7 || k == 8 || k == 9) {
            BOOST_CHECK_MESSAGE(res1[k].sic_committed,
                                "Expected Mode A SIC commit on interfered window " << k
                                << " reason=" << res1[k].sic_status);
            BOOST_CHECK_GT(res1[k].actual_suppression_db, 9.5f);
            ++committed_count;
        }
        // Determinism between dsp_workers=1 and dsp_workers=2 on dynamic slots.
        BOOST_CHECK_EQUAL(res1[k].native_start_abs, res2[k].native_start_abs);
        BOOST_CHECK_EQUAL(res1[k].native_sample_count, res2[k].native_sample_count);
        BOOST_CHECK_EQUAL(res1[k].sic_committed, res2[k].sic_committed);
        BOOST_CHECK_EQUAL(res1[k].radar_sfd_found, res2[k].radar_sfd_found);
        BOOST_CHECK_EQUAL(res1[k].radar_detected_origin, res2[k].radar_detected_origin);
        BOOST_CHECK_EQUAL(res1[k].radar_valid_reps, res2[k].radar_valid_reps);
    }
    BOOST_CHECK_EQUAL(committed_count, 4u);
}

BOOST_AUTO_TEST_CASE(test_auto_schedule_mode_b_early_and_late_sensing_interference)
{
    const RxProfileSpec sensing_int_spec{ 9, 16, "4z2", mod::PulseSpec{} };
    mod::PulseSpec comm_pulse;
    comm_pulse.shape = mod::PulseShape::Gaussian;
    comm_pulse.gaussian_sigma_ns = 2.5f;
    const RxProfileSpec comm_spec{ 10, 64, "4z2", comm_pulse };

    const auto tx_taps = load_tx_48_65_taps();
    std::vector<cf> sensing_prefix;
    BOOST_REQUIRE(modulate_prefix_iq_998p4(sensing_int_spec, sensing_prefix));
    const auto wire_psdu = mod::make_random_psdu(kPayloadDataBytes, 202611132u, true);
    const uint64_t expected_fnv_wire = fnv1a_64(wire_psdu.data(), wire_psdu.size());
    const uint64_t expected_fnv_data = fnv1a_64(wire_psdu.data(), kPayloadDataBytes);
    std::vector<cf> comm_pkt;
    BOOST_REQUIRE(modulate_packet_iq_998p4(comm_spec, wire_psdu, comm_pkt));

    const TempCf32File sensing_sync_file(l2_normalize_sync(sensing_prefix.data(), kSps));
    const TempCf32File comm_sync_file(l2_normalize_sync(comm_pkt.data(), kSps));

    constexpr size_t kNumPkts = 14;
    constexpr double kNominalPeriodS = 0.0008;
    constexpr double kSfoFactor = 1.0 - 100e-6; // -100 ppm crystal drift
    constexpr double kTruePeriodNative = kNominalPeriodS * kNativeFs * kSfoFactor;
    constexpr double kT0Native = 92000.0;

    const float target_rms = 0.075f;
    const float int_rms = target_rms * static_cast<float>(std::pow(10.0, 12.0 / 20.0));

    const size_t total_native =
        static_cast<size_t>(std::ceil(kT0Native + kNumPkts * kTruePeriodNative)) +
        200000;
    std::vector<sc16> stream(total_native, sc16(0, 0));

    for (size_t k = 0; k < kNumPkts; ++k) {
        InterfererBurstSpec ib;
        if (k == 5 || k == 6) {
            ib.present = true;
            ib.offset_work_rel_target = -20000; // early sensing SFD + pre-SFD in pre_guard
            ib.rms = int_rms;
            ib.cfo_hz = -21000.0;
        } else if (k == 7 || k == 8) {
            ib.present = true;
            // late sensing arrives inside Comm target, SFD lands in post_guard
            ib.offset_work_rel_target = static_cast<int64_t>(comm_pkt.size()) - 10000;
            ib.rms = int_rms;
            ib.cfo_hz = 19000.0;
        }
        stamp_burst_737_sc16(
            stream,
            kT0Native + static_cast<double>(k) * kTruePeriodNative,
            comm_pkt,
            comm_spec.sync_reps * kSps,
            target_rms,
            13000.0,
            ib,
            sensing_prefix,
            sensing_int_spec.sync_reps * kSps,
            tx_taps);
    }

    pipeline::PipelineConfig cfg;
    cfg.source_type = pipeline::StreamSourceType::Replay;
    cfg.native_rate = kNativeFs;
    cfg.work_rate = kWorkFs;
    cfg.period_s = kNominalPeriodS;
    cfg.pool_capacity = 8;
    cfg.consumer_mode = pipeline::ConsumerMode::SicPlusComm;
    cfg.resampler_profile = "realtime_minorder";
    cfg.scale_mode = pipeline::Sc16ScaleMode::UnitRange;
    cfg.dsp_workers = 1;
    cfg.max_windows = kNumPkts;

    cfg.schedule_mode = pipeline::ScheduleMode::AutoAcquire;
    cfg.auto_schedule.period_s = kNominalPeriodS;
    cfg.auto_schedule.max_period_rel_err = 0.02;
    cfg.auto_schedule.energy_threshold = 0.002f;
    cfg.auto_schedule.verifier_coarse_stride = 1;
    cfg.auto_schedule.lock_observations = 6;
    cfg.auto_schedule.provisional_guard_us = 25.0;
    cfg.auto_schedule.auto_interference_guards = true;
    cfg.auto_schedule.target_max_psdu_bytes = 32;
    cfg.auto_schedule.early_int_pre_sfd_sync_reps = 16;

    cfg.sic_cfg.policy = sic::SicPolicy::PreambleOnly;
    cfg.sic_cfg.backend = sic::SicEstimatorBackend::Ipatov;
    cfg.sic_cfg.comm_code = sensing_int_spec.code_index;
    cfg.sic_cfg.sync_reps = sensing_int_spec.sync_reps;
    cfg.sic_cfg.sfd_mode = sensing_int_spec.sfd_mode;
    cfg.sic_cfg.pulse = sensing_int_spec.pulse;
    cfg.sic_cfg.full_window_search = true;
    cfg.sic_cfg.coarse_stride = 8;
    cfg.sic_sync_template_file = sensing_sync_file.path();

    cfg.comm_cfg = make_comm_profile(comm_spec, 4096, 1);
    cfg.comm_pulse = comm_spec.pulse;
    cfg.comm_sync_template_file = comm_sync_file.path();

    pipeline::PipelineLedger ledger;
    pipeline::AutoScheduleStats stats;
    const auto res = run_memory_pipeline(
        cfg,
        std::make_unique<MemorySc16Source>(stream, kNativeFs),
        &ledger, &stats);

    std::string inv_err;
    BOOST_CHECK_MESSAGE(ledger.verify_invariants(&inv_err), inv_err);
    BOOST_CHECK_EQUAL(ledger.completed_windows, kNumPkts);
    BOOST_REQUIRE_EQUAL(res.size(), kNumPkts);
    BOOST_CHECK_EQUAL(stats.final_state, "locked");
    BOOST_CHECK_EQUAL(stats.acq_windows, 1u);
    BOOST_CHECK_CLOSE(stats.period_samples, kTruePeriodNative, 0.005);

    size_t committed_count = 0;
    for (size_t k = 0; k < kNumPkts; ++k) {
        BOOST_CHECK_MESSAGE(res[k].comm_ok && res[k].comm_phr_ok && res[k].comm_fcs_ok,
                            "Mode B window " << k << " comm decode failed: reason="
                                             << (res[k].comm_status_reason ? res[k].comm_status_reason : "null")
                                             << " sic=" << res[k].sic_status);
        BOOST_CHECK(res[k].comm_psdu_fnv1a == expected_fnv_wire ||
                    res[k].comm_psdu_fnv1a == expected_fnv_data);
        if (k >= 5 && k <= 8) {
            BOOST_CHECK_MESSAGE(res[k].sic_committed,
                                "Expected Mode B SIC commit on interfered window " << k
                                << " reason=" << res[k].sic_status);
            BOOST_CHECK_GT(res[k].actual_suppression_db, 12.0f);
            ++committed_count;
        }
    }
    BOOST_CHECK_EQUAL(committed_count, 4u);
}

BOOST_AUTO_TEST_CASE(test_auto_schedule_stream_discontinuity_gap_relock)
{
    const RxProfileSpec sensing_spec{ 9, 64, "4z2", mod::PulseSpec{} };
    const RxProfileSpec comm_spec{ 10, 16, "4z2", mod::PulseSpec{} };

    const auto tx_taps = load_tx_48_65_taps();
    std::vector<cf> sensing_prefix;
    BOOST_REQUIRE(modulate_prefix_iq_998p4(sensing_spec, sensing_prefix));
    const TempCf32File sensing_sync_file(l2_normalize_sync(sensing_prefix.data(), kSps));

    constexpr double kNominalPeriodS = 0.0008;
    constexpr double kPeriodNative = kNominalPeriodS * kNativeFs;
    const float target_rms = 0.08f;

    constexpr size_t kPrePkts = 6;
    constexpr size_t kPostPkts = 7;
    constexpr size_t kGapSamples = 60000;

    std::vector<sc16> pre_stream(
        static_cast<size_t>(60000.0 + kPrePkts * kPeriodNative + 150000), sc16(0, 0));
    for (size_t k = 0; k < kPrePkts; ++k) {
        stamp_burst_737_sc16(
            pre_stream, 60000.0 + static_cast<double>(k) * kPeriodNative,
            sensing_prefix, sensing_spec.sync_reps * kSps,
            target_rms, 11000.0, InterfererBurstSpec{}, {}, 0, tx_taps);
    }

    // Post-gap train begins at a completely shifted local origin so old schedule
    // would miss without discontinuity reset + re-acquire.
    std::vector<sc16> post_stream(
        static_cast<size_t>(110000.0 + kPostPkts * kPeriodNative + 150000), sc16(0, 0));
    for (size_t k = 0; k < kPostPkts; ++k) {
        stamp_burst_737_sc16(
            post_stream, 110000.0 + static_cast<double>(k) * kPeriodNative,
            sensing_prefix, sensing_spec.sync_reps * kSps,
            target_rms, 11000.0, InterfererBurstSpec{}, {}, 0, tx_taps);
    }

    pipeline::PipelineConfig cfg;
    cfg.source_type = pipeline::StreamSourceType::Replay;
    cfg.native_rate = kNativeFs;
    cfg.work_rate = kWorkFs;
    cfg.period_s = kNominalPeriodS;
    cfg.pool_capacity = 8;
    cfg.consumer_mode = pipeline::ConsumerMode::SicPlusSensing;
    cfg.resampler_profile = "realtime_minorder";
    cfg.scale_mode = pipeline::Sc16ScaleMode::UnitRange;
    cfg.dsp_workers = 1;
    cfg.max_windows = kPrePkts + kPostPkts;

    cfg.schedule_mode = pipeline::ScheduleMode::AutoAcquire;
    cfg.auto_schedule.period_s = kNominalPeriodS;
    cfg.auto_schedule.energy_threshold = 0.002f;
    cfg.auto_schedule.verifier_coarse_stride = 1;
    cfg.auto_schedule.lock_observations = 4;
    cfg.auto_schedule.provisional_guard_us = 25.0;
    cfg.auto_schedule.auto_interference_guards = true;

    cfg.sic_cfg.policy = sic::SicPolicy::AutoFallback;
    cfg.sic_cfg.backend = sic::SicEstimatorBackend::Ipatov;
    cfg.sic_cfg.comm_code = comm_spec.code_index;
    cfg.sic_cfg.sync_reps = comm_spec.sync_reps;
    cfg.sic_cfg.sfd_mode = comm_spec.sfd_mode;
    cfg.sic_cfg.full_window_search = true;
    cfg.sic_cfg.coarse_stride = 8;
    cfg.sic_sync_template_file = ""; // let pipeline synthesize code 10 template

    cfg.radar_cfg = make_radar_profile(sensing_spec, 1, false);
    cfg.radar_pulse = sensing_spec.pulse;
    cfg.radar_sync_template_file = sensing_sync_file.path();

    pipeline::PipelineLedger ledger;
    pipeline::AutoScheduleStats stats;
    const auto res = run_memory_pipeline(
        cfg,
        std::make_unique<GapMemorySc16Source>(
            std::move(pre_stream), kGapSamples, std::move(post_stream), kNativeFs),
        &ledger, &stats);

    std::string inv_err;
    BOOST_CHECK_MESSAGE(ledger.verify_invariants(&inv_err), inv_err);
    BOOST_CHECK_GE(ledger.read_overflows, 1u);
    BOOST_CHECK_GE(stats.discontinuities, 1u);
    BOOST_CHECK_GE(stats.acq_windows, 2u); // pre-gap acq + post-gap re-acq
    BOOST_CHECK_EQUAL(stats.final_state, "locked");

    size_t ok_count = 0;
    for (const auto& w : res) {
        if (w.radar_executed && w.radar_sfd_found && w.radar_valid_reps > 0) {
            ++ok_count;
        }
    }
    BOOST_CHECK_GE(ok_count, kPrePkts + kPostPkts - 1);
}

BOOST_AUTO_TEST_SUITE_END()
