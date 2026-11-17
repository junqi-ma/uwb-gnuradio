// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_grid.h>

namespace gr::uwb::demod::core {
// One admission decision shared by rotation, prefix generation and fused
// decoding. The effective profile changes only Full -> Bypass on rejection.
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
        if (!plan.integer_grid_ok &&
            plan.effective_profile.cir_soft_chip_mode == CirSoftChipMode::Full)
            plan.effective_profile.cir_soft_chip_mode = CirSoftChipMode::Bypass;
    }
    return plan;
}
} // namespace gr::uwb::demod::core
