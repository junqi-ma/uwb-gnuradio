// SPDX-License-Identifier: GPL-3.0-or-later
// Run-time selection of the BPM-BPSK algorithm ablation levels (M0..M4).
//
// The switches live in Qm35825Profile so the same binary can produce every
// ablation point without recompiling:
//
//   M0  previous fused baseline (4 candidates, split LLR, truncated)
//   M1  + known-hop 2-candidate correlation
//   M2  + joint bi-orthogonal ML Viterbi (+ terminated tail)
//   M3  + PHR pilot phase refine + symbol DD-PLL
//   M4  + fused fractional-phase SFO tracking
//
// Set UWB_BPM_LEVEL=0..4 in the environment of the app.  Unset keeps the
// app's own defaults (i.e. production behaviour is unchanged).
#pragma once

#include <gnuradio/uwb/uwb_phy_profile.h>

#include <cstdlib>

namespace gr::uwb::apps {

inline int bpm_ablation_level()
{
    const char* v = std::getenv("UWB_BPM_LEVEL");
    if (!v || !*v)
        return -1;
    return std::atoi(v);
}

inline bool env_flag(const char* name, bool dflt)
{
    const char* v = std::getenv(name);
    if (!v || !*v)
        return dflt;
    return std::atoi(v) != 0;
}

// `fused_baseline` forces the previous-round fused path on, so M0 is the
// 2d8f49b fused behaviour rather than the app's own default.
inline void apply_bpm_ablation(demod::Qm35825Profile& p, bool fused_baseline)
{
    const int level = bpm_ablation_level();
    if (level < 0)
        return;
    if (fused_baseline) {
        p.rx_softchip_on_demand = true;
        p.rx_fused_payload_cmf = true;
        p.rx_fused_phr_cmf = true;
    }
    p.rx_bpm_known_hop = level >= 1;
    p.rx_bpm_joint_ml = level >= 2;
    p.rx_bpm_terminated_viterbi = level >= 2;
    // DD-PLL is implemented and QA-verified, but it costs ~40 us/packet in the
    // 1208-symbol kernel and gave no FER gain on the 6x40 fixtures or the
    // -13 dB sweep, so it stays opt-in (UWB_BPM_DDPLL=1).
    p.rx_bpm_ddpll = false;
    p.rx_phr_phase_refine = level >= 3;
    p.rx_fused_sfo_track = level >= 4;
    // Per-switch overrides for switch isolation during the ablation.
    p.rx_bpm_known_hop = env_flag("UWB_BPM_KNOWN_HOP", p.rx_bpm_known_hop);
    p.rx_bpm_joint_ml = env_flag("UWB_BPM_JOINT_ML", p.rx_bpm_joint_ml);
    p.rx_bpm_terminated_viterbi =
        env_flag("UWB_BPM_TERMINATED", p.rx_bpm_terminated_viterbi);
    p.rx_bpm_ddpll = env_flag("UWB_BPM_DDPLL", p.rx_bpm_ddpll);
    p.rx_phr_phase_refine =
        env_flag("UWB_BPM_PHR_REFINE", p.rx_phr_phase_refine);
    p.rx_fused_sfo_track = env_flag("UWB_BPM_SFO", p.rx_fused_sfo_track);
}

} // namespace gr::uwb::apps
