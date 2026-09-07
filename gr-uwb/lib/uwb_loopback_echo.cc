/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbLoopbackEcho — software delay/gain/AWGN channel (no UHD).
 *
 * Scheduler: tx message handler only.  RX scratch, TX scratch, and PCHIP
 * slope buffers are sized to max_rx_samples / max_tx_samples at make().
 * Overflow of those bounds publishes status=invalid_window and no RX PDU.
 *
 * Fractional delay is Fritsch-Carlson PCHIP on I and Q separately,
 * matching MATLAB interp1(..., 'pchip', 0) with extrapolation 0.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_loopback_echo.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace gr {
namespace uwb {

namespace {

constexpr double kIntegerDelayEps = 1e-9;

bool is_integer_delay(double d)
{
    if (!std::isfinite(d))
        return false;
    return std::abs(d - std::nearbyint(d)) <= kIntegerDelayEps;
}

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    return radar_meta::to_i64(
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def)), def);
}

double dict_f64(pmt::pmt_t dict, const char* key, double def)
{
    if (!pmt::is_dict(dict))
        return def;
    return radar_meta::to_f64(
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_double(def)), def);
}

std::string dict_str(pmt::pmt_t dict, const char* key, const std::string& def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    if (pmt::eq(v, pmt::PMT_NIL))
        return def;
    try {
        return pmt::write_string(v);
    } catch (...) {
        return def;
    }
}

void split_seconds(double t, int64_t& full, double& frac)
{
    if (!std::isfinite(t)) {
        full = 0;
        frac = 0.0;
        return;
    }
    full = static_cast<int64_t>(std::floor(t));
    frac = t - static_cast<double>(full);
}

// Fritsch-Carlson PCHIP slopes on a uniform unit grid (h=1).
void pchip_slopes_unit(const double* y, size_t n, double* d)
{
    if (n == 0)
        return;
    if (n == 1) {
        d[0] = 0.0;
        return;
    }
    if (n == 2) {
        d[0] = y[1] - y[0];
        d[1] = d[0];
        return;
    }

    auto edge = [](double h0, double h1, double m0, double m1) {
        const double den = h0 + h1;
        double s = 0.0;
        if (den != 0.0)
            s = ((2.0 * h0 + h1) * m0 - h0 * m1) / den;
        if (m0 == 0.0 || s * m0 < 0.0)
            return 0.0;
        if (m0 * m1 < 0.0 && std::abs(s) > 3.0 * std::abs(m0))
            return 3.0 * m0;
        return s;
    };

    d[0] = edge(1.0, 1.0, y[1] - y[0], y[2] - y[1]);
    d[n - 1] = edge(1.0, 1.0, y[n - 1] - y[n - 2], y[n - 2] - y[n - 3]);

    for (size_t i = 1; i + 1 < n; ++i) {
        const double m0 = y[i] - y[i - 1];
        const double m1 = y[i + 1] - y[i];
        if (m0 == 0.0 || m1 == 0.0 || (m0 > 0.0) != (m1 > 0.0)) {
            d[i] = 0.0;
        } else {
            d[i] = 2.0 / (1.0 / m0 + 1.0 / m1);
        }
    }
}

double hermite_unit(double y0, double y1, double d0, double d1, double t)
{
    const double t2 = t * t;
    const double t3 = t2 * t;
    return (2.0 * t3 - 3.0 * t2 + 1.0) * y0 + (t3 - 2.0 * t2 + t) * d0 +
           (-2.0 * t3 + 3.0 * t2) * y1 + (t3 - t2) * d1;
}

double pchip_eval(const double* y, const double* d, size_t n, double xq)
{
    if (n == 0 || !std::isfinite(xq) || xq < 0.0)
        return 0.0;
    const double xmax = static_cast<double>(n - 1);
    if (xq > xmax)
        return 0.0;
    if (xq == xmax)
        return y[n - 1];
    auto i = static_cast<size_t>(std::floor(xq));
    if (i >= n - 1)
        return y[n - 1];
    const double t = xq - static_cast<double>(i);
    return hermite_unit(y[i], y[i + 1], d[i], d[i + 1], t);
}

bool rx_len_fits(size_t pre, size_t tx_len, size_t tail, size_t max_rx,
                 size_t* rx_len)
{
    if (tx_len > max_rx)
        return false;
    if (pre > max_rx - tx_len)
        return false;
    const size_t body = pre + tx_len;
    if (tail > max_rx - body)
        return false;
    *rx_len = body + tail;
    return true;
}

} // namespace

UwbLoopbackEcho::sptr
UwbLoopbackEcho::make(size_t pre_guard_samples,
                      size_t tail_samples,
                      const std::vector<double>& delay_samples,
                      const std::vector<gr_complex>& gains,
                      float noise_std,
                      uint32_t rng_seed,
                      size_t max_tx_samples,
                      size_t max_rx_samples,
                      size_t num_delay_samps,
                      double t0_s,
                      double pri_s)
{
    return gnuradio::make_block_sptr<UwbLoopbackEcho>(pre_guard_samples,
                                                      tail_samples,
                                                      delay_samples,
                                                      gains,
                                                      noise_std,
                                                      rng_seed,
                                                      max_tx_samples,
                                                      max_rx_samples,
                                                      num_delay_samps,
                                                      t0_s,
                                                      pri_s);
}

UwbLoopbackEcho::UwbLoopbackEcho(size_t pre_guard_samples,
                                 size_t tail_samples,
                                 const std::vector<double>& delay_samples,
                                 const std::vector<gr_complex>& gains,
                                 float noise_std,
                                 uint32_t rng_seed,
                                 size_t max_tx_samples,
                                 size_t max_rx_samples,
                                 size_t num_delay_samps,
                                 double t0_s,
                                 double pri_s)
    : gr::block("uwb_loopback_echo",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_pre_guard_(pre_guard_samples),
      d_tail_(tail_samples),
      d_delay_(delay_samples),
      d_gains_(gains),
      d_noise_std_(noise_std),
      d_rng_seed_(rng_seed),
      d_max_tx_(max_tx_samples),
      d_max_rx_(max_rx_samples),
      d_num_delay_samps_(num_delay_samps),
      d_t0_s_(t0_s),
      d_pri_s_(pri_s),
      d_rng_(rng_seed)
{
    if (d_max_tx_ == 0 || d_max_rx_ == 0) {
        throw std::invalid_argument(
            "UwbLoopbackEcho: max_tx_samples and max_rx_samples must be > 0");
    }
    if (!(d_pri_s_ > 0.0)) {
        throw std::invalid_argument("UwbLoopbackEcho: pri_s must be > 0");
    }
    if (!(d_noise_std_ >= 0.0f) || !std::isfinite(d_noise_std_)) {
        throw std::invalid_argument("UwbLoopbackEcho: noise_std must be >= 0");
    }
    if (d_delay_.empty() && d_gains_.empty()) {
        d_delay_ = { 0.0 };
        d_gains_ = { gr_complex(1.0f, 0.0f) };
    } else if (d_delay_.empty() || d_gains_.empty() ||
               d_delay_.size() != d_gains_.size()) {
        throw std::invalid_argument(
            "UwbLoopbackEcho: delay_samples and gains must be the same "
            "non-empty size (or both empty)");
    }
    for (double d : d_delay_) {
        if (!std::isfinite(d)) {
            throw std::invalid_argument(
                "UwbLoopbackEcho: delay_samples must be finite");
        }
    }
    for (gr_complex g : d_gains_) {
        if (!std::isfinite(g.real()) || !std::isfinite(g.imag())) {
            throw std::invalid_argument(
                "UwbLoopbackEcho: gains must be finite");
        }
    }

    d_rx_.assign(d_max_rx_, gr_complex(0.0f, 0.0f));
    d_tx_scratch_.assign(d_max_tx_, gr_complex(0.0f, 0.0f));
    d_s16_out_.assign(d_max_rx_ * 2, 0);
    d_y_i_.assign(d_max_rx_, 0.0);
    d_y_q_.assign(d_max_rx_, 0.0);
    d_d_i_.assign(d_max_rx_, 0.0);
    d_d_q_.assign(d_max_rx_, 0.0);

    message_port_register_in(pmt::mp("tx"));
    message_port_register_out(pmt::mp("rx"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("tx"), [this](pmt::pmt_t msg) { handle_tx(msg); });
}

UwbLoopbackEcho::~UwbLoopbackEcho() = default;

void
UwbLoopbackEcho::publish_status(const std::string& event, pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::mp(event));
    meta = pmt::dict_add(meta, pmt::mp("pdus_received"),
                         pmt::from_uint64(pdus_received()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_emitted"),
                         pmt::from_uint64(pdus_emitted()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_dropped"),
                         pmt::from_uint64(pdus_dropped()));
    if (pmt::is_dict(extra)) {
        pmt::pmt_t items = pmt::dict_items(extra);
        for (size_t i = 0; i < pmt::length(items); ++i) {
            pmt::pmt_t kv = pmt::nth(i, items);
            meta = pmt::dict_add(meta, pmt::car(kv), pmt::cdr(kv));
        }
    }
    message_port_pub(pmt::mp("status"), meta);
}

void
UwbLoopbackEcho::drop_status(const std::string& event, pmt::pmt_t extra)
{
    d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
    publish_status(event, extra);
}

void
UwbLoopbackEcho::apply_integer_path(const gr_complex* tx,
                                    size_t tx_len,
                                    size_t rx_len,
                                    double delay,
                                    gr_complex gain)
{
    const int64_t start =
        static_cast<int64_t>(d_pre_guard_) + static_cast<int64_t>(std::llround(delay));
    for (size_t k = 0; k < tx_len; ++k) {
        const int64_t idx = start + static_cast<int64_t>(k);
        if (idx < 0 || static_cast<size_t>(idx) >= rx_len)
            continue;
        d_rx_[static_cast<size_t>(idx)] += gain * tx[k];
    }
}

void
UwbLoopbackEcho::prepare_pchip_base(const gr_complex* tx,
                                    size_t tx_len,
                                    size_t rx_len)
{
    std::fill(d_y_i_.begin(), d_y_i_.begin() + static_cast<std::ptrdiff_t>(rx_len),
              0.0);
    std::fill(d_y_q_.begin(), d_y_q_.begin() + static_cast<std::ptrdiff_t>(rx_len),
              0.0);
    if (d_pre_guard_ < rx_len) {
        const size_t ncopy = std::min(tx_len, rx_len - d_pre_guard_);
        for (size_t k = 0; k < ncopy; ++k) {
            d_y_i_[d_pre_guard_ + k] = static_cast<double>(tx[k].real());
            d_y_q_[d_pre_guard_ + k] = static_cast<double>(tx[k].imag());
        }
    }
    pchip_slopes_unit(d_y_i_.data(), rx_len, d_d_i_.data());
    pchip_slopes_unit(d_y_q_.data(), rx_len, d_d_q_.data());
}

void
UwbLoopbackEcho::apply_frac_path(size_t rx_len, double delay, gr_complex gain)
{
    const double gr = static_cast<double>(gain.real());
    const double gi = static_cast<double>(gain.imag());
    for (size_t i = 0; i < rx_len; ++i) {
        const double xq = static_cast<double>(i) - delay;
        const double yi = pchip_eval(d_y_i_.data(), d_d_i_.data(), rx_len, xq);
        const double yq = pchip_eval(d_y_q_.data(), d_d_q_.data(), rx_len, xq);
        const double re = gr * yi - gi * yq;
        const double im = gr * yq + gi * yi;
        d_rx_[i] += gr_complex(static_cast<float>(re), static_cast<float>(im));
    }
}

struct TxProfile {
    double sample_rate = 0.0;
    bool native_rate = false;
    std::string sample_format;
    size_t sync_reps = 0;
    std::string sfd_mode;
    int64_t code_index = 0;
    int64_t sync_samples = -1;
    int64_t sfd_samples = -1;
};

// Native 737.28 grid length of a work-domain sample count: the 48/65
// down-conversion convention used by the 65/48 contract (ceil, matching
// the canonical native SYNC lengths 24009/48018/96036 for 32/64/128 reps).
inline int64_t native_span(int64_t work_samples)
{
    const int64_t scaled = work_samples * 48;
    return (scaled + 64) / 65;
}

// Returns status event name on failure, empty string on success.
std::string validate_loopback_profile(pmt::pmt_t meta,
                                      bool input_sc16,
                                      size_t tx_len,
                                      TxProfile& out)
{
    if (!radar_meta::dict_has(meta, "sample_rate"))
        return "bad_input_rate";
    out.sample_rate = dict_f64(meta, "sample_rate", 0.0);
    // The loopback is the software stand-in for the EchoTimer RX window and
    // must accept both the native 737.28 MS/s capture grid (production
    // path: native packet -> loopback -> PDU 65/48) and the 998.4 MS/s
    // work grid (direct path).  Delays are sample-index domain in the
    // input grid either way.
    out.native_rate = radar_meta::is_native_rate(out.sample_rate);
    if (!out.native_rate && !radar_meta::is_work_rate(out.sample_rate))
        return "bad_input_rate";

    out.sample_format = dict_str(meta, "sample_format", "");
    if (out.sample_format.empty())
        out.sample_format = input_sc16 ? "sc16" : "fc32";
    const bool format_sc16 = (out.sample_format == "sc16");
    const bool format_fc32 = (out.sample_format == "fc32");
    if (!format_sc16 && !format_fc32)
        return "invalid_profile";
    if (format_sc16 != input_sc16)
        return "invalid_profile";

    if (!radar_meta::dict_has(meta, radar_meta::kSyncRepetitions) ||
        !radar_meta::dict_has(meta, radar_meta::kSfdMode) ||
        !radar_meta::dict_has(meta, radar_meta::kCodeIndex))
        return "invalid_profile";

    const int64_t sync_reps_i =
        dict_i64(meta, radar_meta::kSyncRepetitions, -1);
    if (sync_reps_i <= 0 ||
        !radar_meta::sync_reps_supported(static_cast<size_t>(sync_reps_i)))
        return "invalid_profile";
    out.sync_reps = static_cast<size_t>(sync_reps_i);

    out.sfd_mode = dict_str(meta, radar_meta::kSfdMode, "");
    const auto sfd = demod::GetSfdSequence(out.sfd_mode.c_str());
    if (sfd.empty())
        return "invalid_profile";

    out.code_index = dict_i64(meta, radar_meta::kCodeIndex, -1);
    if (out.code_index < 0 ||
        !radar_meta::code_index_supported(static_cast<size_t>(out.code_index)))
        return "invalid_profile";

    if (radar_meta::dict_has(meta, radar_meta::kTxPacketSamples)) {
        const int64_t n = dict_i64(meta, radar_meta::kTxPacketSamples, -1);
        if (n < 0 || static_cast<size_t>(n) != tx_len)
            return "invalid_profile";
    }
    if (radar_meta::dict_has(meta, "sample_count")) {
        const int64_t n = dict_i64(meta, "sample_count", -1);
        if (n < 0 || static_cast<size_t>(n) != tx_len)
            return "invalid_profile";
    }
    if (radar_meta::dict_has(meta, "pre_guard_samples") &&
        dict_i64(meta, "pre_guard_samples", 0) < 0)
        return "invalid_profile";
    if (radar_meta::dict_has(meta, radar_meta::kRangeGuardSamples) &&
        dict_i64(meta, radar_meta::kRangeGuardSamples, 0) < 0)
        return "invalid_profile";

    // Expected SYNC/SFD spans depend on the input grid: 1016 work samples
    // per symbol at 998.4 MS/s, ceil(work*48/65) native samples per symbol
    // at 737.28 MS/s.
    const int64_t expect_sync =
        out.native_rate
            ? native_span(static_cast<int64_t>(out.sync_reps) *
                          static_cast<int64_t>(demod::kQm35SamplesPerSymbol))
            : static_cast<int64_t>(out.sync_reps *
                                   demod::kQm35SamplesPerSymbol);
    const int64_t expect_sfd =
        out.native_rate
            ? native_span(static_cast<int64_t>(sfd.size()) *
                          static_cast<int64_t>(demod::kQm35SamplesPerSymbol))
            : static_cast<int64_t>(sfd.size() *
                                   demod::kQm35SamplesPerSymbol);
    out.sync_samples = expect_sync;
    out.sfd_samples = expect_sfd;
    if (radar_meta::dict_has(meta, radar_meta::kSyncSamples)) {
        const int64_t n = dict_i64(meta, radar_meta::kSyncSamples, -1);
        if (n < 0 || n != expect_sync)
            return "invalid_profile";
        out.sync_samples = n;
    }
    if (radar_meta::dict_has(meta, radar_meta::kSfdSamples)) {
        const int64_t n = dict_i64(meta, radar_meta::kSfdSamples, -1);
        if (n < 0 || n != expect_sfd)
            return "invalid_profile";
        out.sfd_samples = n;
    }
    return {};
}

void
UwbLoopbackEcho::handle_tx(pmt::pmt_t msg)
{
    d_pdus_received_.fetch_add(1, std::memory_order_relaxed);

    if (!pmt::is_pair(msg)) {
        drop_status("invalid_input");
        return;
    }

    pmt::pmt_t meta_in = pmt::car(msg);
    pmt::pmt_t data_in = pmt::cdr(msg);
    if (!pmt::is_dict(meta_in))
        meta_in = pmt::make_dict();
    if (!pmt::is_c32vector(data_in) && !pmt::is_s16vector(data_in)) {
        drop_status("invalid_input");
        return;
    }

    const bool input_sc16 = pmt::is_s16vector(data_in);
    const size_t payload_items = pmt::length(data_in);
    if (payload_items == 0 || (input_sc16 && (payload_items & 1u) != 0)) {
        drop_status("invalid_input");
        return;
    }

    const size_t tx_len = input_sc16 ? payload_items / 2 : payload_items;

    TxProfile prof;
    const std::string profile_event =
        validate_loopback_profile(meta_in, input_sc16, tx_len, prof);
    if (!profile_event.empty()) {
        drop_status(profile_event);
        return;
    }

    if (tx_len > d_max_tx_) {
        drop_status("invalid_window");
        return;
    }

    size_t rx_len = 0;
    if (!rx_len_fits(d_pre_guard_, tx_len, d_tail_, d_max_rx_, &rx_len)) {
        drop_status("invalid_window");
        return;
    }

    const gr_complex* tx_ptr = nullptr;
    size_t n_elem = 0;
    if (!input_sc16) {
        tx_ptr = pmt::c32vector_elements(data_in, n_elem);
        if (tx_ptr == nullptr || n_elem != tx_len) {
            drop_status("invalid_input");
            return;
        }
    } else {
        const int16_t* s16 = pmt::s16vector_elements(data_in, n_elem);
        if (s16 == nullptr || n_elem != payload_items) {
            drop_status("invalid_input");
            return;
        }
        for (size_t i = 0; i < tx_len; ++i) {
            d_tx_scratch_[i] = gr_complex(static_cast<float>(s16[2 * i]),
                                          static_cast<float>(s16[2 * i + 1]));
        }
        tx_ptr = d_tx_scratch_.data();
    }

    std::fill(d_rx_.begin(),
              d_rx_.begin() + static_cast<std::ptrdiff_t>(rx_len),
              gr_complex(0.0f, 0.0f));

    bool need_pchip = false;
    for (double d : d_delay_) {
        if (!is_integer_delay(d)) {
            need_pchip = true;
            break;
        }
    }
    if (need_pchip)
        prepare_pchip_base(tx_ptr, tx_len, rx_len);

    for (size_t p = 0; p < d_delay_.size(); ++p) {
        if (is_integer_delay(d_delay_[p]))
            apply_integer_path(tx_ptr, tx_len, rx_len, d_delay_[p], d_gains_[p]);
        else
            apply_frac_path(rx_len, d_delay_[p], d_gains_[p]);
    }

    if (d_noise_std_ > 0.0f) {
        std::normal_distribution<float> dist(0.0f, d_noise_std_);
        for (size_t i = 0; i < rx_len; ++i) {
            d_rx_[i] += gr_complex(dist(d_rng_), dist(d_rng_));
        }
    }

    if (d_num_delay_samps_ > 0) {
        if (d_num_delay_samps_ >= rx_len) {
            std::fill(d_rx_.begin(),
                      d_rx_.begin() + static_cast<std::ptrdiff_t>(rx_len),
                      gr_complex(0.0f, 0.0f));
        } else {
            std::memmove(d_rx_.data(),
                         d_rx_.data() + d_num_delay_samps_,
                         (rx_len - d_num_delay_samps_) * sizeof(gr_complex));
            std::fill(d_rx_.begin() +
                          static_cast<std::ptrdiff_t>(rx_len - d_num_delay_samps_),
                      d_rx_.begin() + static_cast<std::ptrdiff_t>(rx_len),
                      gr_complex(0.0f, 0.0f));
        }
    }

    const int64_t pulse_id = std::max<int64_t>(
        0, dict_i64(meta_in, radar_meta::kPulseId, 0));
    int64_t schedule_index =
        dict_i64(meta_in, radar_meta::kScheduleIndex, pulse_id);
    if (schedule_index < 0)
        schedule_index = pulse_id;

    const double fs = prof.sample_rate;
    const std::string sample_format = prof.sample_format;
    const size_t sync_reps = prof.sync_reps;
    const std::string sfd_mode = prof.sfd_mode;
    const int64_t code_index = prof.code_index;
    const int64_t sync_samples = prof.sync_samples;
    const int64_t sfd_samples = prof.sfd_samples;

    const double tx_time =
        d_t0_s_ + static_cast<double>(pulse_id) * d_pri_s_;
    const double pre_s =
        (fs > 0.0) ? static_cast<double>(d_pre_guard_) / fs : 0.0;
    const double rx_time = tx_time - pre_s;
    int64_t tx_full = 0, rx_full = 0;
    double tx_frac = 0.0, rx_frac = 0.0;
    split_seconds(tx_time, tx_full, tx_frac);
    split_seconds(rx_time, rx_full, rx_frac);

    double cal_delay = d_delay_.empty() ? 0.0 : d_delay_.front();

    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kPulseId),
                         pmt::from_uint64(static_cast<uint64_t>(pulse_id)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kScheduleIndex),
                         pmt::from_uint64(static_cast<uint64_t>(schedule_index)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kTxTimeFull),
                         pmt::from_long(tx_full));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kTxTimeFrac),
                         pmt::from_double(tx_frac));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kRxTimeFull),
                         pmt::from_long(rx_full));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kRxTimeFrac),
                         pmt::from_double(rx_frac));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kNumDelaySamps),
                         pmt::from_long(static_cast<long>(d_num_delay_samps_)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kCalDelayNative),
                         pmt::from_double(cal_delay));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kCalibrationId),
                         pmt::mp(""));
    if (fs > 0.0) {
        meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                             pmt::from_double(fs));
    }
    meta = pmt::dict_add(meta, pmt::mp("sample_format"),
                         pmt::mp(sample_format));
    meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"),
                         pmt::from_long(static_cast<long>(d_pre_guard_)));
    // Scheduled-capture geometry (PDU 65/48 contract): the RX PDU is the
    // window [0, rx_len) with the echoed packet as the capture body.
    // window_start_sample = 0 (no absolute device grid in loopback),
    // capture = TX length, post guard = tail.  The resampler maps these
    // onto the work grid; without capture_samples a non-zero pre_guard
    // cannot satisfy pre_guard + capture <= sample_count and would be
    // dropped as invalid_metadata.
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"),
                         pmt::from_long(0));
    meta = pmt::dict_add(meta, pmt::mp("capture_samples"),
                         pmt::from_long(static_cast<long>(tx_len)));
    meta = pmt::dict_add(meta, pmt::mp("post_guard_samples"),
                         pmt::from_long(static_cast<long>(d_tail_)));
    if (sync_samples >= 0) {
        meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSyncSamples),
                             pmt::from_long(sync_samples));
    }
    if (sfd_samples >= 0) {
        meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSfdSamples),
                             pmt::from_long(sfd_samples));
    }
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kRangeGuardSamples),
                         pmt::from_long(static_cast<long>(d_tail_)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kTxPacketSamples),
                         pmt::from_long(static_cast<long>(tx_len)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kRxCaptureSamples),
                         pmt::from_long(static_cast<long>(rx_len)));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(rx_len)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSyncRepetitions),
                         pmt::from_long(static_cast<long>(sync_reps)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSfdMode),
                         pmt::mp(sfd_mode));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kCodeIndex),
                         pmt::from_long(code_index));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kUhdError), pmt::mp("none"));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSource),
                         pmt::mp("loopback"));

    pmt::pmt_t data_out;
    if (!input_sc16) {
        data_out = pmt::init_c32vector(rx_len, d_rx_.data());
    } else {
        for (size_t i = 0; i < rx_len; ++i) {
            const long ir = std::lround(d_rx_[i].real());
            const long ii = std::lround(d_rx_[i].imag());
            d_s16_out_[2 * i] = static_cast<int16_t>(
                std::max(-32768L, std::min(32767L, ir)));
            d_s16_out_[2 * i + 1] = static_cast<int16_t>(
                std::max(-32768L, std::min(32767L, ii)));
        }
        data_out = pmt::init_s16vector(rx_len * 2, d_s16_out_.data());
    }

    message_port_pub(pmt::mp("rx"), pmt::cons(meta, data_out));
    d_pdus_emitted_.fetch_add(1, std::memory_order_relaxed);
}

} // namespace uwb
} // namespace gr
