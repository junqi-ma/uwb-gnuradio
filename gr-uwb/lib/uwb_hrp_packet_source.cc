/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_hrp_packet_source.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>

#include <stdexcept>

namespace gr {
namespace uwb {

namespace {

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    return radar_meta::to_i64(
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def)), def);
}

} // namespace

UwbHrpPacketSource::sptr
UwbHrpPacketSource::make(const std::vector<uint8_t>& psdu,
                         size_t sync_repetitions,
                         const std::string& sfd_mode,
                         size_t code_index,
                         float peak_amplitude,
                         double pri_s,
                         bool auto_emit,
                         bool insert_sts,
                         bool append_fcs)
{
    return gnuradio::make_block_sptr<UwbHrpPacketSource>(
        psdu, sync_repetitions, sfd_mode, code_index, peak_amplitude, pri_s,
        auto_emit, insert_sts, append_fcs);
}

UwbHrpPacketSource::UwbHrpPacketSource(const std::vector<uint8_t>& psdu,
                                       size_t sync_repetitions,
                                       const std::string& sfd_mode,
                                       size_t code_index,
                                       float peak_amplitude,
                                       double pri_s,
                                       bool auto_emit,
                                       bool insert_sts,
                                       bool append_fcs)
    : gr::block("uwb_hrp_packet_source",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_psdu_(psdu),
      d_sfd_mode_(sfd_mode),
      d_pri_s_(pri_s),
      d_auto_emit_(auto_emit),
      d_append_fcs_(append_fcs)
{
    d_cfg_.code_index = code_index;
    d_cfg_.sync_repetitions = sync_repetitions;
    d_cfg_.sfd_mode = d_sfd_mode_.c_str();
    d_cfg_.insert_sts = insert_sts;
    d_cfg_.peak_amplitude = peak_amplitude;
    d_cfg_.ranging = false;

    if (append_fcs) {
        if (d_psdu_.size() + 2 > radar_meta::kMaxPsduBytes) {
            throw std::invalid_argument(
                "UwbHrpPacketSource: PSDU+FCS exceeds 127");
        }
        if (!d_psdu_.empty())
            mod::append_ieee_fcs(d_psdu_);
    }
    if (d_psdu_.size() > radar_meta::kMaxPsduBytes) {
        throw std::invalid_argument(
            "UwbHrpPacketSource: PSDU length must be 0..127");
    }
    if (insert_sts && !mod::sfd_mode_is_4z(sfd_mode.c_str())) {
        throw std::invalid_argument(
            "UwbHrpPacketSource: insert_sts requires a 4z SFD");
    }
    if (!radar_meta::code_index_supported(code_index)) {
        throw std::invalid_argument(
            "UwbHrpPacketSource: code_index must be 9..12");
    }
    if (!radar_meta::sync_reps_supported(sync_repetitions)) {
        throw std::invalid_argument(
            std::string("UwbHrpPacketSource: sync_repetitions must be ") +
            radar_meta::sync_reps_supported_list());
    }
    if (!(pri_s > 0.0)) {
        throw std::invalid_argument("UwbHrpPacketSource: pri_s must be > 0");
    }
    const auto sfd = demod::GetSfdSequence(sfd_mode.c_str());
    if (sfd.empty()) {
        throw std::invalid_argument("UwbHrpPacketSource: unknown sfd_mode");
    }

    d_scratch_.reserve(radar_meta::kMaxPsduBytes, mod::kMaxHrpTxSamples);
    d_fc32_.reserve(mod::kMaxHrpTxSamples);
    if (!rebuild()) {
        throw std::runtime_error("UwbHrpPacketSource: modulate_one failed");
    }
    const double duration =
        static_cast<double>(d_num_samples_) / radar_meta::kWorkRateHz;
    if (!(duration < pri_s)) {
        throw std::invalid_argument(
            "UwbHrpPacketSource: packet duration must be < pri_s");
    }

    message_port_register_in(pmt::mp("emit"));
    message_port_register_out(pmt::mp("tx"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("emit"),
                    [this](pmt::pmt_t msg) { handle_emit(msg); });
}

UwbHrpPacketSource::~UwbHrpPacketSource() = default;

bool
UwbHrpPacketSource::rebuild()
{
    d_cfg_.sfd_mode = d_sfd_mode_.c_str();
    const auto sfd = demod::GetSfdSequence(d_sfd_mode_.c_str());
    if (sfd.empty())
        return false;
    const size_t n_want = mod::packet_samples_998p4(
        d_cfg_.sync_repetitions, sfd.size(), d_psdu_.size(),
        d_cfg_.insert_sts);
    d_fc32_.reserve(mod::kMaxHrpTxSamples);
    if (d_fc32_.size() < n_want)
        d_fc32_.resize(n_want);
    size_t n = 0;
    if (!mod::modulate_one(d_psdu_.data(), d_psdu_.size(), d_cfg_, d_scratch_,
                           d_fc32_.data(), d_fc32_.size(), n) ||
        n == 0) {
        return false;
    }
    d_fc32_.resize(n);
    d_num_samples_ = n;
    d_iq_pmt_ = pmt::init_c32vector(n, d_fc32_.data());
    d_sync_samples_ = static_cast<int64_t>(d_cfg_.sync_repetitions *
                                           demod::kQm35SamplesPerSymbol);
    d_sfd_samples_ =
        static_cast<int64_t>(sfd.size() * demod::kQm35SamplesPerSymbol);
    return true;
}

bool
UwbHrpPacketSource::start()
{
    if (d_auto_emit_)
        publish_tx(d_next_pulse_id_++);
    return true;
}

void
UwbHrpPacketSource::publish_status(const std::string& event, pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::mp(event));
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
UwbHrpPacketSource::publish_tx(uint64_t pulse_id)
{
    if (pmt::is_null(d_iq_pmt_)) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_waveform");
        return;
    }
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kPulseId),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kScheduleIndex),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(radar_meta::kWorkRateHz));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("fc32"));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSyncRepetitions),
                         pmt::from_long(static_cast<long>(d_cfg_.sync_repetitions)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSfdMode),
                         pmt::mp(d_sfd_mode_));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kCodeIndex),
                         pmt::from_long(static_cast<long>(d_cfg_.code_index)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kTxPacketSamples),
                         pmt::from_long(static_cast<long>(d_num_samples_)));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(d_num_samples_)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSource),
                         pmt::mp("hrp_packet_source"));
    if (d_sync_samples_ >= 0) {
        meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSyncSamples),
                             pmt::from_long(d_sync_samples_));
    }
    if (d_sfd_samples_ >= 0) {
        meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSfdSamples),
                             pmt::from_long(d_sfd_samples_));
    }
    message_port_pub(pmt::mp("tx"), pmt::cons(meta, d_iq_pmt_));
    d_pdus_emitted_.fetch_add(1, std::memory_order_relaxed);
}

void
UwbHrpPacketSource::handle_emit(pmt::pmt_t msg)
{
    d_emits_received_.fetch_add(1, std::memory_order_relaxed);

    pmt::pmt_t dict = msg;
    pmt::pmt_t payload = pmt::PMT_NIL;
    if (pmt::is_pair(msg)) {
        if (pmt::is_dict(pmt::car(msg)))
            dict = pmt::car(msg);
        payload = pmt::cdr(msg);
    }
    if (pmt::is_u8vector(payload)) {
        size_t n = 0;
        const uint8_t* p = pmt::u8vector_elements(payload, n);
        if (n > radar_meta::kMaxPsduBytes) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_psdu");
            return;
        }
        d_psdu_.assign(p, p + n);
        if (!rebuild()) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("modulate_failed");
            return;
        }
    }

    uint64_t pulse_id = d_next_pulse_id_;
    if (pmt::is_dict(dict) && radar_meta::dict_has(dict, radar_meta::kPulseId)) {
        const int64_t v = dict_i64(dict, radar_meta::kPulseId, 0);
        if (v < 0) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_emit");
            return;
        }
        pulse_id = static_cast<uint64_t>(v);
        d_next_pulse_id_ = pulse_id + 1;
    } else if (pmt::is_uint64(msg) || pmt::is_integer(msg)) {
        const int64_t v = radar_meta::to_i64(msg, 0);
        if (v < 0) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_emit");
            return;
        }
        pulse_id = static_cast<uint64_t>(v);
        d_next_pulse_id_ = pulse_id + 1;
    } else {
        d_next_pulse_id_ = pulse_id + 1;
    }
    publish_tx(pulse_id);
}

} // namespace uwb
} // namespace gr
