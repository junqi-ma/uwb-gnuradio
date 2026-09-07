/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * FakeBurstBackend — deterministic, allocation-tolerant CI backend for the
 * UWB EchoTimer (Radar Step 10).  Header-only; implements the same
 * IRadioBurstBackend contract the Step-11 UhdBurstBackend will implement,
 * without any device or wall-clock dependence.
 *
 * Behaviour:
 *  - Deterministic device time: d_device_ticks starts at
 *    device_time_start, is advanced by device_time_auto_advance after
 *    each completed burst, and can be driven explicitly by QA through
 *    set_device_time() (used to create expired grid slots / late
 *    commands).
 *  - Partial TX AND RX: max_io_chunk caps the elements transferred per
 *    internal send()/recv() call.  Calls that return fewer samples than
 *    requested count as partial re-issues (mirroring the UHD partial
 *    send/recv loop the Step-11 backend must implement).
 *  - Deterministic RX data: recv() fills the caller's buffers with
 *    expected_rx_sample(schedule_index, element), so QA can check the
 *    stitched RX sequence point-for-point.
 *  - One-shot fault injection per schedule index: late command, timeout
 *    (RX never completes), overflow, broken chain (fragment flags
 *    corrupted), stop-during-I/O.  Faults are reported as per-schedule-
 *    index results through the normal result path — never exceptions and
 *    never silently dropped.
 *  - stop-during-I/O: request_stop() wakes collect_result() waiters;
 *    incomplete bursts then report StopDuringIo, so block stop() can
 *    join its worker with I/O "in flight" without deadlocking.
 *
 * This backend is a QA instrument, not part of the production hot path:
 * it may allocate inside issue/collect (stitched sample buffers, flag
 * sequences).
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_FAKE_BURST_BACKEND_H
#define INCLUDED_GNURADIO_UWB_UWB_FAKE_BURST_BACKEND_H

#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_scheduler_core.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace echo {

class FakeBurstBackend : public IRadioBurstBackend
{
public:
    struct Config {
        // Per-call transfer cap for send/recv (forces partial TX AND RX
        // with re-issues).  0 = no cap (one call per fragment).
        uint64_t max_io_chunk = 0;
        // Initial device time; advanced by device_time_auto_advance after
        // each completed burst.  0 advance = QA drives the clock.
        int64_t device_time_start = 0;
        int64_t device_time_auto_advance = 0;
        // One-shot fault injection keyed by schedule index.  The fault is
        // consumed the first time that index is armed.
        std::map<uint64_t, BurstStatus> faults;
    };

    // Completed-burst QA record: exact sent/received counts, per-call
    // flag sequences (backend-seen SOB/EOB/time-spec), partial re-issue
    // counts and stitched sample sequences (point-for-point).
    struct FakeBurstRecord {
        uint64_t schedule_index = 0;
        BurstStatus status = BurstStatus::BackendError;
        uint64_t tx_samples_requested = 0;
        uint64_t tx_samples_sent = 0;
        uint64_t rx_samples_requested = 0;
        uint64_t rx_samples_received = 0;
        uint64_t tx_calls = 0;
        uint64_t rx_calls = 0;
        uint64_t tx_reissues = 0; // calls that returned fewer than requested
        uint64_t rx_reissues = 0;
        std::vector<uint8_t> tx_flags; // per-call flag bytes, issue order
        std::vector<uint8_t> rx_flags;
        std::vector<int16_t> tx_stitched; // SC16 pairs, point-for-point
        std::vector<int16_t> rx_stitched;
        int64_t rx_time_ticks = -1;
        std::string error;
    };

    explicit FakeBurstBackend(const Config& cfg) : d_cfg(cfg) {}

    // --- IRadioBurstBackend -------------------------------------------------

    bool prepare(std::string& error) override
    {
        (void)error; // never fails; idempotent (restart-safe reset)
        std::lock_guard<std::mutex> lock(d_mutex);
        d_prepared = true;
        reset_locked();
        return true;
    }

    BurstStatus issue_rx(const RxCommand& cmd, std::string& error) override
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (!d_prepared) {
            error = "backend not prepared";
            return BurstStatus::BackendError;
        }
        if (d_stop) {
            error = "backend stopped";
            return BurstStatus::StopDuringIo;
        }
        if (!valid_fragment_list(cmd.fragments, cmd.fragment_count)) {
            error = "invalid rx fragment list";
            return BurstStatus::BackendError;
        }
        // Clear any stale result/record for this index (restart reuse).
        d_results.erase(cmd.schedule_index);
        d_records.erase(cmd.schedule_index);
        d_active_fault = take_fault_locked(cmd.schedule_index);
        if (d_active_fault == BurstStatus::LateCommand) {
            error = "late command (injected): rx_ticks in the device past";
            return BurstStatus::LateCommand;
        }
        if (cmd.rx_ticks <= d_device_ticks) {
            error = "late command: rx_ticks <= device time";
            return BurstStatus::LateCommand;
        }
        d_rx_armed = true;
        d_armed_index = cmd.schedule_index;
        d_pending_rx.schedule_index = cmd.schedule_index;
        d_pending_rx.rx_ticks = cmd.rx_ticks;
        d_pending_rx.total_samples = cmd.total_samples;
        d_pending_rx.fragment_count = cmd.fragment_count;
        for (size_t i = 0; i < cmd.fragment_count; ++i)
            d_pending_rx.fragments[i] = cmd.fragments[i];
        return BurstStatus::Ok;
    }

    BurstStatus issue_tx(const TxCommand& cmd, std::string& error) override
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (!d_prepared) {
            error = "backend not prepared";
            return BurstStatus::BackendError;
        }
        if (d_stop) {
            error = "backend stopped";
            return BurstStatus::StopDuringIo;
        }
        if (d_active_fault == BurstStatus::BrokenChain) {
            error = "broken chain (injected): fragment flags corrupted";
            return BurstStatus::BrokenChain;
        }
        if (!valid_fragment_list(cmd.fragments, cmd.fragment_count)) {
            error = "broken chain: invalid TX fragment flags";
            return BurstStatus::BrokenChain;
        }
        if (!d_rx_armed || d_armed_index != cmd.schedule_index) {
            error = "broken chain: TX issued without armed RX for the "
                    "same schedule index";
            return BurstStatus::BrokenChain;
        }
        if (cmd.tx_ticks <= d_device_ticks) {
            error = "late command: tx_ticks <= device time";
            return BurstStatus::LateCommand;
        }

        FakeBurstRecord rec;
        rec.schedule_index = cmd.schedule_index;
        rec.tx_samples_requested = cmd.total_samples;
        bool first_call = true;
        uint64_t sent_total = 0;
        for (size_t i = 0; i < cmd.fragment_count; ++i) {
            const BurstFragment& f = cmd.fragments[i];
            uint64_t done = 0;
            while (done < f.count) {
                const uint64_t remaining = f.count - done;
                uint64_t chunk = remaining;
                if (d_cfg.max_io_chunk != 0 && chunk > d_cfg.max_io_chunk)
                    chunk = d_cfg.max_io_chunk;
                unsigned call_flags = f.flags;
                if (!first_call)
                    call_flags &= ~(kFlagTimeSpec | kFlagStartOfBurst);
                rec.tx_flags.push_back(static_cast<uint8_t>(call_flags));
                ++rec.tx_calls;
                if (chunk < remaining)
                    ++rec.tx_reissues;
                if (f.tx_data != nullptr) {
                    rec.tx_stitched.insert(
                        rec.tx_stitched.end(), f.tx_data + done * 2,
                        f.tx_data + (done + chunk) * 2);
                }
                done += chunk;
                sent_total += chunk;
                first_call = false;
                if (d_stop) {
                    // Natural stop mid-send: partial record, worker
                    // synthesizes the StopDuringIo result.
                    rec.tx_samples_sent = sent_total;
                    rec.status = BurstStatus::StopDuringIo;
                    rec.error = "stop during TX send (in flight)";
                    d_records[rec.schedule_index] = std::move(rec);
                    return BurstStatus::StopDuringIo;
                }
            }
        }
        rec.tx_samples_sent = sent_total;

        BurstResult res;
        res.schedule_index = cmd.schedule_index;
        res.tx_ticks = cmd.tx_ticks;
        res.rx_ticks = d_pending_rx.rx_ticks;
        res.tx_samples_requested = cmd.total_samples;
        res.tx_samples_sent = sent_total;
        res.rx_samples_requested = d_pending_rx.total_samples;

        if (d_active_fault == BurstStatus::Overflow ||
            d_active_fault == BurstStatus::StopDuringIo) {
            // Deliver the faulted result now; no RX samples are produced.
            res.status = d_active_fault;
            res.error = fault_error_locked(d_active_fault);
            d_records[rec.schedule_index] = std::move(rec);
            d_results.emplace(cmd.schedule_index, res);
            d_rx_armed = false;
            d_cv.notify_all();
            return BurstStatus::Ok; // completion pending at collect
        }
        if (d_active_fault == BurstStatus::Timeout) {
            // The RX stream NEVER completes: no result is queued, so the
            // worker blocks in collect_result() for the full wait and
            // then synthesizes the Timeout result (or StopDuringIo if a
            // stop was requested first).
            res.status = BurstStatus::Timeout;
            res.error = fault_error_locked(BurstStatus::Timeout);
            d_records[rec.schedule_index] = std::move(rec);
            d_rx_armed = false;
            d_cv.notify_all();
            return BurstStatus::Ok; // completion never arrives
        }

        // Execute the timed RX stream (the backend-internal recv loop).
        bool rx_first_call = true;
        uint64_t received_total = 0;
        for (size_t i = 0; i < d_pending_rx.fragment_count; ++i) {
            const BurstFragment& f = d_pending_rx.fragments[i];
            uint64_t done = 0;
            while (done < f.count) {
                const uint64_t remaining = f.count - done;
                uint64_t chunk = remaining;
                if (d_cfg.max_io_chunk != 0 && chunk > d_cfg.max_io_chunk)
                    chunk = d_cfg.max_io_chunk;
                unsigned call_flags = f.flags;
                if (!rx_first_call)
                    call_flags &= ~(kFlagTimeSpec | kFlagStartOfBurst);
                rec.rx_flags.push_back(static_cast<uint8_t>(call_flags));
                ++rec.rx_calls;
                if (chunk < remaining)
                    ++rec.rx_reissues;
                if (f.rx_data != nullptr) {
                    for (uint64_t e = 0; e < chunk; ++e) {
                        const uint64_t pos = f.offset + done + e;
                        f.rx_data[(done + e) * 2] =
                            expected_rx_sample(cmd.schedule_index, pos * 2);
                        f.rx_data[(done + e) * 2 + 1] =
                            expected_rx_sample(cmd.schedule_index,
                                               pos * 2 + 1);
                    }
                    rec.rx_stitched.insert(
                        rec.rx_stitched.end(), f.rx_data + done * 2,
                        f.rx_data + (done + chunk) * 2);
                }
                done += chunk;
                received_total += chunk;
                rx_first_call = false;
                if (d_stop) {
                    rec.rx_samples_received = received_total;
                    rec.status = BurstStatus::StopDuringIo;
                    rec.error = "stop during RX recv (in flight)";
                    d_records[rec.schedule_index] = std::move(rec);
                    return BurstStatus::StopDuringIo;
                }
            }
        }
        rec.rx_samples_received = received_total;
        rec.rx_time_ticks = d_pending_rx.rx_ticks;
        rec.status = (rec.tx_reissues + rec.rx_reissues) > 0
                         ? BurstStatus::PartialHandled
                         : BurstStatus::Ok;
        res.rx_time_ticks = d_pending_rx.rx_ticks;
        res.rx_samples_received = received_total;
        res.tx_reissues = rec.tx_reissues;
        res.rx_reissues = rec.rx_reissues;
        res.status = rec.status;
        d_records[rec.schedule_index] = std::move(rec);
        d_results.emplace(cmd.schedule_index, res);
        d_rx_armed = false;
        d_device_ticks += d_cfg.device_time_auto_advance;
        d_cv.notify_all();
        return BurstStatus::Ok;
    }

    bool collect_result(uint64_t schedule_index,
                        uint64_t wait_ms,
                        BurstResult& out) override
    {
        std::unique_lock<std::mutex> lock(d_mutex);
        ++d_collect_waiters;
        const auto wait = std::chrono::milliseconds(wait_ms);
        d_cv.wait_for(lock, wait, [&] {
            return d_results.count(schedule_index) != 0 || d_stop;
        });
        --d_collect_waiters;
        auto it = d_results.find(schedule_index);
        if (it != d_results.end()) {
            out = it->second;
            d_results.erase(it);
            return true;
        }
        if (d_stop) {
            out = BurstResult{};
            out.schedule_index = schedule_index;
            out.status = BurstStatus::StopDuringIo;
            out.error = "backend stopped while RX I/O in flight";
            return true;
        }
        return false; // caller synthesizes Timeout
    }

    void abort_rx() override
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_rx_armed = false;
        ++d_abort_count;
    }

    void request_stop() override
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_stop = true;
        d_cv.notify_all();
    }

    bool stop_requested() const override
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        return d_stop;
    }

    int64_t device_time_ticks() const override
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        return d_device_ticks;
    }

    // --- QA introspection ---------------------------------------------------

    // Deterministic RX sample the fake device produces for one int16_t
    // element (`element` = 2 * sample_pos + {0,1}).
    static int16_t expected_rx_sample(uint64_t schedule_index,
                                      uint64_t element)
    {
        const uint64_t v =
            schedule_index * 40503ull + element * 7919ull + 12345ull;
        return static_cast<int16_t>((v & 0x7FFF) - 0x4000);
    }

    void set_device_time(int64_t ticks)
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        d_device_ticks = ticks;
    }

    // Number of threads currently blocked inside collect_result().
    uint64_t collect_waiters() const
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        return d_collect_waiters;
    }

    uint64_t abort_count() const
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        return d_abort_count;
    }

    size_t burst_count() const
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        return d_records.size();
    }

    // Copy-out a completed burst record (thread-safe snapshot).
    bool find_record(uint64_t schedule_index, FakeBurstRecord& out) const
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        auto it = d_records.find(schedule_index);
        if (it == d_records.end())
            return false;
        out = it->second;
        return true;
    }

private:
    struct PendingRx {
        uint64_t schedule_index = 0;
        int64_t rx_ticks = 0;
        uint64_t total_samples = 0;
        BurstFragment fragments[kEchoMaxFragmentsPerBurst] = {};
        size_t fragment_count = 0;
    };

    void reset_locked()
    {
        d_stop = false;
        d_rx_armed = false;
        d_armed_index = 0;
        d_active_fault = BurstStatus::Ok;
        d_device_ticks = d_cfg.device_time_start;
        d_results.clear();
        d_records.clear();
        d_collect_waiters = 0;
        d_abort_count = 0;
        d_pending_rx = PendingRx{};
    }

    // One-shot fault lookup (consumed on read).
    BurstStatus take_fault_locked(uint64_t schedule_index)
    {
        auto it = d_cfg.faults.find(schedule_index);
        if (it == d_cfg.faults.end())
            return BurstStatus::Ok;
        const BurstStatus f = it->second;
        d_cfg.faults.erase(it);
        return f;
    }

    const char* fault_error_locked(BurstStatus f) const
    {
        switch (f) {
        case BurstStatus::Timeout:
            return "RX stream did not complete (injected timeout)";
        case BurstStatus::Overflow:
            return "RX overflow reported by device (injected)";
        case BurstStatus::StopDuringIo:
            return "stop while I/O in flight (injected)";
        default:
            return "injected fault";
        }
    }

    static bool valid_fragment_list(const BurstFragment* frags, size_t count)
    {
        if (frags == nullptr || count == 0 ||
            count > kEchoMaxFragmentsPerBurst)
            return false;
        for (size_t i = 0; i < count; ++i) {
            const unsigned f = frags[i].flags;
            if (frags[i].count == 0)
                return false;
            if (i == 0) {
                if (!(f & kFlagTimeSpec) || !(f & kFlagStartOfBurst))
                    return false; // first fragment: time spec + SOB
            } else if (f & (kFlagTimeSpec | kFlagStartOfBurst)) {
                return false; // SOB / time spec outside first fragment
            }
            if (i + 1 == count) {
                if (!(f & kFlagEndOfBurst))
                    return false; // last fragment: EOB
            } else if (f & kFlagEndOfBurst) {
                return false; // EOB outside last fragment
            }
        }
        return true;
    }

    Config d_cfg;
    mutable std::mutex d_mutex;
    std::condition_variable d_cv;

    bool d_prepared = false;
    bool d_stop = false;
    bool d_rx_armed = false;
    uint64_t d_armed_index = 0;
    BurstStatus d_active_fault = BurstStatus::Ok;
    int64_t d_device_ticks = 0;
    uint64_t d_collect_waiters = 0;
    uint64_t d_abort_count = 0;
    std::map<uint64_t, BurstResult> d_results;
    std::map<uint64_t, FakeBurstRecord> d_records;
    PendingRx d_pending_rx;
};

} // namespace echo
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_FAKE_BURST_BACKEND_H */
