/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Abstract radio burst backend for the UWB EchoTimer (Radar Step 10).
 *
 * The scheduler worker (inside the EchoTimer gr::block, never a GNU Radio
 * message handler) drives one timed burst per schedule index:
 *
 *   1. issue_rx()  — arm the timed RX stream command FIRST (UHD:
 *      issue_stream_cmd with STREAM_MODE_NUM_SAMPS_AND_DONE, stream_now =
 *      false, time_spec = rx_ticks).  The RX command must be committed
 *      earlier than the TX command.
 *   2. issue_tx()  — execute the TX burst (UHD: send() loop over the
 *      planned fragments; first fragment carries the time spec + SOB, the
 *      last fragment carries EOB, intermediate fragments neither).  The
 *      backend handles partial send()/recv() and re-issues internally,
 *      mirroring the UHD requirement that a single send()/recv() call may
 *      transfer fewer samples than requested.
 *   3. collect_result() — wait for the RX completion on the backend's
 *      completion event (condition variable; UHD: blocking recv()).  This
 *      is the asynchronous wait point: never a busy wait, never I/O in a
 *      GNU Radio handler thread.
 *
 * Semantics reference: gr-uhd usrp_sink (tx_time tag → has_time_spec;
 * tx_sob/tx_eob tags → start_of_burst/end_of_burst metadata) and
 * gr-uhd usrp_source (NUM_SAMPS_AND_DONE timed stream commands); the
 * gr-radar usrp_echotimer_cc timed burst geometry.  Backends must never
 * throw across this interface — every failure is a per-burst status.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_ECHO_BURST_BACKEND_H
#define INCLUDED_GNURADIO_UWB_UWB_ECHO_BURST_BACKEND_H

#include <cstdint>
#include <string>

namespace gr {
namespace uwb {
namespace echo {

// Per-burst I/O status.  Exactly one aggregated status per schedule index
// reaches the scheduler worker: either Ok / PartialHandled (burst
// completed; PartialHandled marks bursts that needed partial send/recv
// re-issues) or one of the failure statuses.  The worker converts every
// non-Ok status into a burst result message — backends never throw and
// never silently drop a burst.
enum class BurstStatus : uint8_t {
    Ok = 0,
    PartialHandled = 1, // completed with partial send/recv re-issues
    LateCommand = 2,    // command timestamp already in the device past
    Timeout = 3,        // RX stream did not complete in time
    Overflow = 4,       // device reported an RX overflow
    BrokenChain = 5,    // SOB/EOB/time-spec fragment chain corrupted
    StopDuringIo = 6,   // stop() while I/O was in flight
    BackendError = 7,   // any other backend/transport failure
};

// Flag bits carried on a burst fragment (UHD tx_metadata/rx_metadata
// analogues).  Contract, validated by backends (violations are reported
// as BrokenChain):
//   - the FIRST fragment of a burst carries kFlagTimeSpec + kFlagStartOfBurst
//   - the LAST fragment carries kFlagEndOfBurst
//   - intermediate fragments carry neither
inline constexpr unsigned kFlagStartOfBurst = 0x1;
inline constexpr unsigned kFlagEndOfBurst = 0x2;
inline constexpr unsigned kFlagTimeSpec = 0x4;

// One planned burst fragment.  Elements are SC16 sample pairs (two
// int16_t each: I then Q), the production native wire format.
struct BurstFragment {
    const int16_t* tx_data = nullptr; // TX payload slice (nullptr for RX)
    int16_t* rx_data = nullptr;       // RX buffer slice (nullptr for TX)
    uint64_t offset = 0;              // element offset inside the burst
    uint64_t count = 0;               // elements in this fragment
    unsigned flags = 0;               // kFlag* bits, see contract above
    int64_t device_ticks = 0;         // command time when kFlagTimeSpec
};

struct TxCommand {
    uint64_t schedule_index = 0;
    int64_t tx_ticks = 0;
    uint64_t total_samples = 0;
    const BurstFragment* fragments = nullptr;
    size_t fragment_count = 0;
};

struct RxCommand {
    uint64_t schedule_index = 0;
    int64_t rx_ticks = 0;
    uint64_t total_samples = 0;
    const BurstFragment* fragments = nullptr; // first: time spec + SOB;
    size_t fragment_count = 0;                // last: EOB
};

// Aggregated per-burst result collected by the scheduler worker.
struct BurstResult {
    uint64_t schedule_index = 0;
    BurstStatus status = BurstStatus::BackendError;
    int64_t tx_ticks = -1;
    int64_t rx_ticks = -1;
    int64_t rx_time_ticks = -1; // device time of the first received sample
    uint64_t skipped_slots = 0; // expired grid slots skipped for this burst
    uint64_t tx_samples_requested = 0;
    uint64_t tx_samples_sent = 0;
    uint64_t rx_samples_requested = 0;
    uint64_t rx_samples_received = 0;
    uint64_t tx_reissues = 0; // partial send() calls handled
    uint64_t rx_reissues = 0; // partial recv() calls handled
    std::string error;        // UHD-like strerror field
};

// Abstract radio burst backend (dependency-injected into the EchoTimer
// block; FakeBurstBackend for CI in Step 10, UhdBurstBackend in Step 11).
class IRadioBurstBackend
{
public:
    virtual ~IRadioBurstBackend() = default;

    // One-time preparation (may allocate).  Called from block start(),
    // before the worker thread is spawned.  Repeated prepare() calls
    // (restart) must reset all internal state.
    virtual bool prepare(std::string& error) = 0;

    // Arm the timed RX stream command.  MUST be called before issue_tx()
    // for the same schedule index (RX command earlier than TX) and with
    // rx_ticks in the device future.  Returns Ok when armed, else the
    // failure status with a UHD-like error string.
    virtual BurstStatus issue_rx(const RxCommand& cmd, std::string& error) = 0;

    // Execute the TX burst: all fragments, with the backend internally
    // looping over partial send()/recv() transfers until the burst is
    // complete (or a fault/stop aborts it).  Returns Ok when the burst
    // executed and its completion is pending collect_result(), else the
    // failure status with a UHD-like error string.
    virtual BurstStatus issue_tx(const TxCommand& cmd, std::string& error) = 0;

    // Wait (block on the backend completion event, up to wait_ms) for the
    // result of one burst index.  Returns true with `out` filled when the
    // burst completed (Ok, PartialHandled or a faulted status).  Returns
    // false when the burst did not complete within wait_ms; the scheduler
    // worker then synthesizes Timeout — or StopDuringIo when a stop was
    // requested in the meantime.
    virtual bool collect_result(uint64_t schedule_index,
                                uint64_t wait_ms,
                                BurstResult& out) = 0;

    // Abort an armed RX stream command after a failed burst (UHD:
    // STOP_CONTINUOUS stream command).  Must be safe to call at any time.
    virtual void abort_rx() = 0;

    // Lifecycle: abort any in-flight I/O so block stop() / destruction
    // can join their worker without deadlocking.  After request_stop(),
    // collect_result() for incomplete bursts reports StopDuringIo.
    virtual void request_stop() = 0;
    virtual bool stop_requested() const = 0;

    // Current device time in ticks (the grid `now` used for expiry).
    virtual int64_t device_time_ticks() const = 0;
};

inline const char* burst_status_to_string(BurstStatus s)
{
    switch (s) {
    case BurstStatus::Ok:
        return "ok";
    case BurstStatus::PartialHandled:
        return "partial_handled";
    case BurstStatus::LateCommand:
        return "late_command";
    case BurstStatus::Timeout:
        return "timeout";
    case BurstStatus::Overflow:
        return "overflow";
    case BurstStatus::BrokenChain:
        return "broken_chain";
    case BurstStatus::StopDuringIo:
        return "stop_during_io";
    case BurstStatus::BackendError:
        return "backend_error";
    }
    return "internal_error";
}

} // namespace echo
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_ECHO_BURST_BACKEND_H */
