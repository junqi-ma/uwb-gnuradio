// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_grid.h>

namespace gr::uwb::demod::core {
// One admission decision shared by rotation, prefix generation and fused
// decoding. Fractional-grid packets keep the requested CIR mode.
struct PacketExecutionPlan {
    bool integer_grid_ok = true;
    Qm35825Profile effective_profile;
};
inline PacketExecutionPlan plan_packet_execution(
    const Qm35825Profile& profile, const TimingResult& timing)
{
    PacketExecutionPlan plan;
    plan.effective_profile = profile;
    if (profile.rx_softchip_on_demand || profile.rx_fused_payload_cmf ||
        profile.rx_fused_phr_cmf) {
        const double spc_hat = (timing.measured_period > 0.0)
            ? timing.measured_period / static_cast<double>(kQm35ChipsPerSymbol)
            : 0.0;
        plan.integer_grid_ok = integer_chip_grid(spc_hat, profile.fused_gate_ppm * 1e-6);
        if (!plan.integer_grid_ok) {
            plan.effective_profile.rx_softchip_on_demand = false;
            plan.effective_profile.rx_fused_payload_cmf = false;
            plan.effective_profile.rx_fused_phr_cmf = false;
        }
    }
    return plan;
}
} // namespace gr::uwb::demod::core
