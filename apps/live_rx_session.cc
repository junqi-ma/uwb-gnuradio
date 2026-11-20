/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Live receive session: source -> streaming acquisition -> bounded packet
 * jobs -> existing C++ demod -> console reporter.
 *
 * The lock state machine lives in core::TargetLockCore (GNU Radio
 * independent, scheduler-thread only).  DSP workers never touch the USRP or
 * the lock; they push bounded observations back.
 */
#include "live_rx_session.h"

#include "console_reporter.h"
#include "timed_replay_source.h"

#include "bpm_ablation_env.h"

#include <gnuradio/uwb/uwb_autocorr_gate.h>
#include <gnuradio/uwb/uwb_demod_grid.h>
#include <gnuradio/uwb/uwb_fixed_tone_cancel_source.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_live_acquisition.h>
#include <gnuradio/uwb/uwb_qm35_acquisition_tracker.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_target_lock_core.h>
#include <gnuradio/uwb/uwb_timed_rx_plan.h>
#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace openuwb;

namespace openuwb {
namespace live {

static volatile std::sig_atomic_t interrupted = 0;
static void on_signal(int) { interrupted = 1; }

static double elapsed_us(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::micro>(end-start).count();
}
static double thread_cpu_us() {
    timespec t{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    return t.tv_sec*1e6 + t.tv_nsec/1e3;
}

template<class T> std::vector<T> load(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot open " + path);
    const auto bytes = f.tellg();
    if (bytes <= 0 || size_t(bytes) % sizeof(T)) throw std::runtime_error("invalid " + path);
    std::vector<T> v(size_t(bytes) / sizeof(T));
    f.seekg(0); if (!f.read(reinterpret_cast<char*>(v.data()), bytes))
        throw std::runtime_error("cannot read " + path);
    return v;
}

struct PendingSubmit : live::PacketRequest {
    size_t count = 0;
    double metric = 0.0;
    bool is_candidate = false; // autocorrelation candidate, classify at submit
};

struct Workers {
    ConsoleReporter& report;
    core::TargetLockCore& lock;
    live::PacketSlotPool pool;
    // Native <-> work coordinate conversion for lock feedback.  Both the
    // prediction and the observation go through the same function, so a
    // constant group delay cancels in the residual; the absolute t0 stays
    // consistent with the mapped observation convention.
    double resample_group_delay = 0.0;
    std::atomic<uint64_t> interference_events{0};
    std::atomic<uint64_t> target_dedup{0};

    Workers(ConsoleReporter& r, core::TargetLockCore& lk, const LiveRxConfig& o,
            const std::vector<float>& taps,
            const std::vector<std::complex<float>>& tmpl)
        : report(r), lock(lk), pool(o.capacity, window) {
        resample_group_delay = taps.empty() ? 0.0 : 0.5 * (double(taps.size()) - 1.0);
        pool.start(o.workers, [&, o, taps, tmpl](size_t worker, live::PacketSlotPool::WorkerLoop& loop) {
            live::PacketProcessor processor(taps, tmpl, window);
            // Acquisition / interference profile: unchanged legacy wide acquisition.
            demod::Qm35825Profile acq;
            acq.code_index = 10; acq.preamble_repetitions = 256;
            acq.cir_repetitions = 246;
            acq.sfd_mode = "4z2";
            acq.cir_soft_chip_mode = o.cir == "full"
                ? demod::CirSoftChipMode::Full
                : demod::CirSoftChipMode::Bypass;
            acq.rx_fast_sfd = true; acq.rx_fast_cfo_rotation = true;
            acq.rx_seed_refine_tail = true;
            acq.rx_softchip_on_demand = o.ondemand_resolved;
            acq.rx_fused_payload_cmf = o.fused;
            acq.rx_fused_phr_cmf = o.fused;
            acq.rx_sfo_from_cfo = o.sfo_from_cfo;
            acq.carrier_freq_hz = o.freq;
            // Runtime ablation level (UWB_BPM_LEVEL=0..4); unset keeps --fused.
            apps::apply_bpm_ablation(acq, true);
            acq.fused_gate_ppm = o.fused_gate_ppm;
            acq.cfo_refit_mode = (o.cfo_refit == "auto") ? 1
                               : (o.cfo_refit == "skip") ? 2 : 0;
            acq.cfo_refit_skip_rad = o.cfo_refit_skip_rad;
            acq.cfo_refit_min_peaks = o.cfo_refit_min_peaks;
            // Capture-only mode: stop after the SFD.  Set before the locked profile
            // is copied from `acq`, so both the acquisition and the locked slots carry it.
            acq.rx_stop_after_sfd = (o.stop_after_sfd == "on");
            acq.rx_stop_after_cir = (o.stop_after_cir == "on");
            acq.rx_timing_sparse = (o.timing_sparse == "on");
            acq.rx_timing_shaped_sparse =
                (o.timing_shaped == "on") ||
                (o.timing_shaped == "auto" && o.timing_sparse == "off");
            acq.rx_sync_par_three_samples = o.par_only == "window3";
            acq.rx_sfd_par_three_samples = acq.rx_sync_par_three_samples ||
                                           o.par_only == "window3-sfd";
            acq.rx_par_confirm = o.par_only == "guarded" || acq.rx_sfd_par_three_samples;
            acq.rx_sync_par_only = o.par_only == "sync" || o.par_only == "both" ||
                                   acq.rx_par_confirm;
            acq.rx_sfd_par_only = o.par_only == "sfd" || o.par_only == "both" ||
                                  acq.rx_par_confirm;
            // Blind sparse autocorrelation seeds can precede or follow the true
            // first SYNC by several symbols.
            acq.timing_search_margin = 131072;
            acq.timing_max_backtrack_symbols = 128;
            // Locked profile: same PHY, bounded first-SYNC ROI.
            demod::Qm35825Profile locked = acq;
            locked.sync_strategy = demod::SyncStrategy::LockedBounded;
            locked.locked_roi_samples = o.locked_roi;
            locked.locked_allow_backtrack = false;
            locked.locked_grid_timing = (o.locked_timing == "grid");
            locked.timing_search_margin = 0;
            locked.timing_max_backtrack_symbols = 0;
            loop.run([&](live::PacketJob& job) {
                try {
                    const auto compute_start = Clock::now();
                    const double cpu_start = thread_cpu_us();
                    PacketCost cost;
                    cost.worker = worker;
                    cost.queue_us = elapsed_us(job.ready, compute_start);
                    const bool bounded = job.target_slot && o.sync == "locked";
                    const demod::Qm35825Profile& p = bounded ? locked : acq;
                    auto processed = processor.process(job, job.iq.data(),
                        job.count, p, o.cir == "auto");
                    auto& r = processed.decoded;
                    const bool retry = processed.retried;
                    const auto resample_end = processed.resample_end;
                    const auto demod_end = processed.demod_end;
                    const auto compute_end = processed.compute_end;
                    cost.resample_us = elapsed_us(compute_start, resample_end);
                    cost.demod_us = elapsed_us(resample_end, demod_end);
                    cost.retry_us = retry ? elapsed_us(demod_end, compute_end) : 0;
                    cost.compute_us = elapsed_us(compute_start, compute_end);
                    cost.cpu_us = thread_cpu_us() - cpu_start;

                    // ---- lock feedback ------------------------
                    const auto obs = live::make_target_observation(
                        job, r, work_fs, fs, resample_group_delay);
                    if (job.feeds_lock)
                        lock.push_observation(obs);

                    std::ostringstream tag;
                    if (r.timing.ok) {
                        tag << "start="
                            << (job.window_start +
                                core::map_obs_sample_to_native(
                                    r.timing.preamble_start_sample,
                                    work_fs, fs, resample_group_delay))
                            << ' ';
                    }
                    tag << format_request_tag(job, true, obs.residual);
                    report.result(job.id, job.seed, status_name(r.status), &r,
                        elapsed_us(job.ready, compute_end) / 1000, retry, &cost,
                        tag.str());
                    if (o.lock_fcs == "on" && o.stop_after_sfd != "on" &&
                        o.stop_after_cir != "on" &&
                        !job.target_slot && r.timing.ok && r.sfd.ok) {
                        const bool identity = r.payload.fcs_pass &&
                            crc_ok(r.payload.bytes);
                        if (!identity) {
                            ++interference_events;
                            std::ostringstream s;
                            s << std::fixed << std::setprecision(3)
                              << "INTERFERENCE start="
                              << (obs.observed_origin >= 0 ? obs.observed_origin
                                                           : job.seed)
                              << " metric=" << r.timing.metric
                              << " sfd=" << r.sfd.ok
                              << " fcs=" << r.payload.fcs_pass
                              << " reason=identity_mismatch";
                            report.info(s.str());
                        }
                    }
                } catch (const std::exception& e) {
                    report.result(job.id, job.seed, "INTERNAL_ERROR", nullptr,
                                  0, false, nullptr, format_request_tag(job));
                    report.info(std::string("WORKER_ERROR ") + e.what());
                }
            });
        });
    }
    ~Workers() { stop(); }
    void stop() { pool.stop(); }
    size_t backlog() { return pool.backlog(); }
    std::string failure() { return pool.failure(); }
    /** Submit from the ring buffer at absolute native `req.window_start`. */
    [[nodiscard]] bool submit(const live::PacketRequest& req, size_t count,
                              const std::vector<std::complex<int16_t>>& ring) {
        return pool.submit(req, count, ring);
    }
};

struct WindowSubmissionTracker {
    struct AssocPending {
        uint64_t slot;
        int64_t start;
        double metric;
    };
    struct TimedPending : live::PacketRequest {
        size_t count = 0;
    };

    ConsoleReporter::SubmissionSummary sub_stats{};
    uint64_t cand_submit_attempts = 0, cand_admitted = 0, cand_dropped_queue = 0;
    uint64_t cand_incomplete = 0, cand_overrun = 0, cand_overlapping = 0;
    uint64_t cand_invalid_gap = 0, autocorr_ignored = 0;
    int64_t pending_seed = -1;
    uint64_t pending_id = 0;
    size_t pending_guard = 0;
    std::deque<PendingSubmit> pending;
    core::TargetSlotPlan pending_plan;
    bool have_pending_plan = false;
    uint64_t pending_slot = 0;
    std::vector<AssocPending> assoc;
    uint64_t dedup_confirmed = 0, dedup_ambiguous = 0;
    uint64_t target_slots = 0, target_dropped = 0, target_expired = 0, dedup = 0, unknown = 0;
    std::deque<TimedPending> timed_pending;

    explicit WindowSubmissionTracker(size_t initial_guard)
        : pending_guard(initial_guard) {}

    void record_candidate_incomplete(ConsoleReporter& report, uint64_t req_id,
                                     int64_t seed, const std::string& tag) {
        report.result(req_id, seed, "INCOMPLETE_WINDOW", nullptr, 0, false,
                      nullptr, tag);
        ++cand_incomplete;
    }

    void record_candidate_overlapping(ConsoleReporter& report, uint64_t req_id,
                                      int64_t seed, size_t guard) {
        report.result(req_id, seed, "OVERLAPPING_CANDIDATE", nullptr, 0, false,
                      nullptr, "path=acquire guard=" + std::to_string(guard));
        ++cand_overlapping;
    }

    void record_candidate_overrun(ConsoleReporter& report,
                                  const live::PacketRequest& req) {
        report.result(req.id, req.seed, "HISTORY_OVERRUN", nullptr, 0, false,
                      nullptr, format_request_tag(req));
        ++cand_overrun;
    }

    void record_candidate_queue_drop(ConsoleReporter& report,
                                     const live::PacketRequest& req,
                                     bool count_submit_attempt) {
        report.result(req.id, req.seed, "DROPPED_QUEUE", nullptr, 0, false,
                      nullptr, format_request_tag(req));
        if (count_submit_attempt)
            ++sub_stats.dropped_queue;
        ++cand_dropped_queue;
    }

    void submit_candidate_window(ConsoleReporter& report, Workers& workers,
                                 const live::PacketRequest& req, size_t count,
                                 const std::vector<std::complex<int16_t>>& ring) {
        ++sub_stats.submit_attempts;
        ++cand_submit_attempts;
        if (!workers.submit(req, count, ring)) {
            record_candidate_queue_drop(report, req, /*count_submit_attempt=*/true);
        } else {
            ++sub_stats.admitted;
            ++cand_admitted;
        }
    }

    void flush_pending_on_gap(ConsoleReporter& report) {
        if (pending_seed >= 0) {
            report.result(pending_id, pending_seed, "INVALID_GAP", nullptr,
                          0, false, nullptr,
                          "path=acquire guard=" + std::to_string(pending_guard));
            ++cand_invalid_gap;
        }
        pending_seed = -1;
        for (const auto& ps : pending) {
            report.result(ps.id, ps.seed, "INVALID_GAP", nullptr, 0,
                          false, nullptr, format_request_tag(ps));
            ++cand_invalid_gap;
        }
        pending.clear();
        have_pending_plan = false;
    }

    void handle_autocorr_seed(int64_t seed, double metric, int64_t valid_begin,
                              size_t cur_pre_guard, size_t window,
                              bool interference_enabled, uint64_t& id,
                              core::TargetLockCore& lock, ConsoleReporter& report) {
        ++sub_stats.acq_seeds;
        const std::string acq_tag =
            "path=acquire guard=" + std::to_string(cur_pre_guard);
        if (lock.scheduled()) {
            // Classification is deferred to submission time: the predicted
            // target window for this slot is only committed ~0.46 ms after the
            // packet start, i.e. after this 6 ms-window autocorrelation
            // detection.
            if (!interference_enabled) {
                ++unknown;
                ++autocorr_ignored;
                return;
            }
            ++id;
            if (seed - int64_t(cur_pre_guard) < valid_begin) {
                record_candidate_incomplete(report, id, seed, acq_tag);
                return;
            }
            PendingSubmit ps;
            ps.id = id;
            ps.seed = seed;
            ps.window_start = seed - int64_t(cur_pre_guard);
            ps.count = window;
            ps.predicted_rel_work = std::llround(cur_pre_guard * 65.0 / 48);
            ps.guard = cur_pre_guard;
            ps.generation = lock.generation();
            ps.metric = metric;
            ps.is_candidate = true;
            if (pending.size() >= 8) {
                record_candidate_queue_drop(
                    report, ps, /*count_submit_attempt=*/false);
            } else {
                pending.push_back(ps);
                ++unknown;
            }
            return;
        }
        if (pending_seed >= 0) {
            ++id;
            record_candidate_overlapping(report, id, seed, cur_pre_guard);
        } else if (seed - int64_t(cur_pre_guard) < valid_begin) {
            ++id;
            record_candidate_incomplete(report, id, seed, acq_tag);
        } else {
            ++id;
            pending_seed = seed;
            pending_id = id;
            pending_guard = cur_pre_guard;
        }
    }

    void submit_ready_acquire_window(int64_t end, int64_t valid_begin,
                                     size_t window,
                                     const std::vector<std::complex<int16_t>>& ring,
                                     core::TargetLockCore& lock,
                                     Workers& workers, ConsoleReporter& report) {
        if (pending_seed < 0 ||
            end < pending_seed - int64_t(pending_guard) + int64_t(window)) {
            return;
        }
        const int64_t ws = pending_seed - int64_t(pending_guard);
        live::PacketRequest req;
        req.id = pending_id;
        req.seed = pending_seed;
        req.window_start = ws;
        req.predicted_rel_work = std::llround(pending_guard * 65.0 / 48);
        req.guard = pending_guard;
        req.slot_index = std::numeric_limits<uint64_t>::max();
        req.generation = lock.generation();
        req.target_slot = false;
        req.feeds_lock = true;
        if (ws < valid_begin) {
            record_candidate_overrun(report, req);
        } else {
            submit_candidate_window(report, workers, req, window, ring);
        }
        pending_seed = -1;
    }

    void submit_ready_deferred_candidates(
        int64_t end, int64_t valid_begin,
        const std::vector<std::complex<int16_t>>& ring,
        core::TargetLockCore& lock, Workers& workers, ConsoleReporter& report) {
        if (pending.empty())
            return;
        std::deque<PendingSubmit> still;
        for (const auto& ps : pending) {
            if (ps.window_start < valid_begin) {
                record_candidate_overrun(report, ps);
                continue;
            }
            if (ps.window_start + int64_t(ps.count) > end) {
                still.push_back(ps);
                continue;
            }
            if (ps.is_candidate) {
                // Now that the target plan for this slot exists, decide
                // whether this autocorrelation response is the target's own.
                // Association is confirmed from the slot result.
                core::InterferenceCandidate c;
                c.start_sample = ps.seed;
                c.metric = ps.metric;
                const auto v = lock.classify_candidate(c);
                if (v.cls == core::CandidateClass::TargetAssociated) {
                    ++dedup;
                    if (assoc.size() < 256)
                        assoc.push_back({ v.slot_index, ps.seed, ps.metric });
                    continue;
                }
            }
            submit_candidate_window(report, workers, ps, ps.count, ring);
        }
        pending.swap(still);
    }

    void submit_continuous_target_windows(
        int64_t end, int64_t valid_begin,
        const std::vector<std::complex<int16_t>>& ring,
        uint64_t& id, core::TargetLockCore& lock,
        Workers& workers, ConsoleReporter& report) {
        if (have_pending_plan &&
            pending_plan.generation != lock.generation()) {
            // The grid changed under us; the old plan is void.
            have_pending_plan = false;
        }
        if (!have_pending_plan) {
            have_pending_plan = lock.next_plan(end, pending_plan);
            if (have_pending_plan)
                pending_slot = pending_plan.slot_index;
        }
        while (have_pending_plan) {
            core::TargetSlotPlan plan = lock.plan_for_slot(pending_slot);
            if (plan.window_end > end)
                break; // still filling
            have_pending_plan = false;
            const size_t count = size_t(plan.window_end - plan.window_start);
            live::PacketRequest req;
            req.id = ++id;
            req.seed = plan.predicted_origin;
            req.window_start = plan.window_start;
            req.predicted_rel_work =
                std::llround((plan.predicted_origin - plan.window_start) * 65.0 / 48);
            req.guard = plan.pre_guard;
            req.slot_index = plan.slot_index;
            req.generation = plan.generation;
            req.target_slot = true;
            req.feeds_lock = true;
            if (plan.window_start < valid_begin) {
                report.result(req.id, req.seed, "HISTORY_OVERRUN", nullptr,
                              0, false, nullptr, format_request_tag(req));
                lock.abandon_slot(plan.slot_index);
                ++target_expired;
            } else {
                ++sub_stats.submit_attempts;
                if (!workers.submit(req, count, ring)) {
                    report.result(req.id, req.seed, "DROPPED_QUEUE",
                                  nullptr, 0, false, nullptr,
                                  format_request_tag(req));
                    lock.note_slot_outcome(plan.slot_index,
                                           core::TargetSlotOutcome::DroppedQueue);
                    ++target_dropped;
                    ++sub_stats.dropped_queue;
                } else {
                    lock.commit_plan(plan);
                    ++target_slots;
                    ++sub_stats.admitted;
                }
            }
            have_pending_plan = lock.next_plan(end, pending_plan);
            if (have_pending_plan)
                pending_slot = pending_plan.slot_index;
        }
    }

    void submit_timed_windows(
        int64_t end, int64_t valid_begin,
        const std::vector<std::complex<int16_t>>& ring,
        core::TargetLockCore& lock, Workers& workers, ConsoleReporter& report) {
        while (!timed_pending.empty()) {
            const TimedPending tp = timed_pending.front();
            if (tp.window_start + int64_t(tp.count) > end)
                break;
            timed_pending.pop_front();
            if (tp.window_start < valid_begin) {
                report.result(tp.id, tp.seed, "HISTORY_OVERRUN", nullptr,
                              0, false, nullptr, format_request_tag(tp));
                if (tp.target_slot)
                    lock.abandon_slot(tp.slot_index);
                continue;
            }
            ++sub_stats.submit_attempts;
            if (!workers.submit(tp, tp.count, ring)) {
                report.result(tp.id, tp.seed, "DROPPED_QUEUE", nullptr,
                              0, false, nullptr, format_request_tag(tp));
                ++sub_stats.dropped_queue;
                if (tp.target_slot) {
                    lock.note_slot_outcome(tp.slot_index,
                                           core::TargetSlotOutcome::DroppedQueue);
                    ++target_dropped;
                }
            } else {
                ++sub_stats.admitted;
            }
        }
    }

    void resolve_deferred_associations(core::TargetLockCore& lock,
                                       ConsoleReporter& report) {
        if (assoc.empty())
            return;
        std::vector<AssocPending> still;
        for (const auto& a : assoc) {
            if (!lock.slot_terminal(a.slot)) {
                still.push_back(a);
                continue;
            }
            if (lock.slot_succeeded(a.slot)) {
                ++dedup_confirmed;
            } else {
                ++dedup_ambiguous;
                std::ostringstream s;
                s << std::fixed << std::setprecision(3)
                  << "INTERFERENCE_AMBIGUOUS slot=" << a.slot
                  << " start=" << a.start << " metric=" << a.metric
                  << " reason=target_slot_failed";
                report.info(s.str());
            }
        }
        assoc.swap(still);
    }

    void flush_pending_at_eof(core::TargetLockCore& lock, ConsoleReporter& report) {
        if (pending_seed >= 0) {
            record_candidate_incomplete(
                report, pending_id, pending_seed,
                "path=acquire guard=" + std::to_string(pending_guard));
            pending_seed = -1;
        }
        for (const auto& ps : pending) {
            record_candidate_incomplete(
                report, ps.id, ps.seed, format_request_tag(ps));
        }
        pending.clear();
        for (const auto& tp : timed_pending) {
            report.result(tp.id, tp.seed, "INCOMPLETE_WINDOW", nullptr, 0,
                          false, nullptr, format_request_tag(tp));
            if (tp.target_slot)
                lock.abandon_slot(tp.slot_index);
        }
        timed_pending.clear();
    }
};

using openuwb::live::Acquisition;

int run_live_rx(const LiveRxConfig& o)
{
    std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal);
    const std::string res_root = resolve_res_dir(o);
    const std::string taps_file =
        (o.taps == "quality")  ? "taps_quality_minorder.txt"
        : (o.taps == "eq32")   ? "taps_quality_eq32.txt"
        : (o.taps == "realtime") ? "taps_realtime_minorder.txt"
                                 : "taps_realtime_eq16.txt";
    const auto taps = load<float>(res_root + "/resampler_65_48/" + taps_file);
    const auto tmpl=load<std::complex<float>>(res_root+"/reference_preamble_code10_998p4.cf32");

    // ---- PHY-derived target window geometry ------------------------------
    // 127 B is the maximum PSDU, so the capture capacity never depends on the
    // PHR of the packet being decoded.  The 998.4 MS/s work-grid span is mapped
    // to native samples with the same 48/65 ratio the resampler uses.
    demod::Qm35825Profile geom;
    geom.code_index = 10; geom.preamble_repetitions = 256; geom.sfd_mode = "4z2";
    geom.max_psdu_bytes = 127;
    const size_t work_span = mod::packet_samples_998p4(
        geom.preamble_repetitions, 8, geom.max_psdu_bytes);
    const size_t native_span =
        size_t(std::llround(double(work_span) * 48.0 / 65.0)) + 1024;
    const size_t pre_guard = pre;
    const size_t post_guard = o.post_guard;
    if (pre_guard + native_span + post_guard > window)
        throw std::runtime_error("target window geometry exceeds the job buffer");
    // The LOCKED guard only has to hold the bounded first-SYNC ROI plus the
    // resampler transient in front of the packet.  Below that floor the cut
    // window would start inside the search region, so refuse it instead of
    // silently truncating the ROI.
    const size_t locked_roi_native =
        size_t(std::llround(double(o.locked_roi) * 48.0 / 65.0));
    const size_t resampler_delay_native = size_t(
        std::llround(0.5 * double(taps.size() - 1) * 48.0 / 65.0));
    const size_t locked_pre_guard_min =
        locked_roi_native + resampler_delay_native + 1024;
    if (o.locked_pre_guard < locked_pre_guard_min)
        throw std::runtime_error(
            "--locked-pre-guard must be >= " +
            std::to_string(locked_pre_guard_min) +
            " (locked ROI " + std::to_string(locked_roi_native) +
            " + resampler group delay " + std::to_string(resampler_delay_native) +
            " + 1024 margin native samples)");
    if (o.locked_pre_guard + native_span + post_guard > window)
        throw std::runtime_error(
            "locked target window geometry exceeds the job buffer");

    core::TargetLockConfig lock_cfg;
    lock_cfg.sample_rate = fs;
    lock_cfg.nominal_period_s = o.period_ms * 1e-3;
    lock_cfg.verify_observations = o.lock_observations;
    lock_cfg.holdover_miss_count = o.holdover;
    lock_cfg.reacquire_miss_count = o.reacquire;
    // Scale the grid-residual outlier gate with the requested locked ROI when
    // no explicit limit is given so wide-ROI jittery emitters (e.g. QM35825)
    // do not trip grid_invalid reacquire loops.
    lock_cfg.residual_limit_samples =
        o.lock_residual_limit > 0.0
            ? o.lock_residual_limit
            : std::max(lock_cfg.residual_limit_samples,
                       double(locked_roi_native));
    lock_cfg.code_index = 10;
    lock_cfg.preamble_repetitions = 256;
    lock_cfg.sfd_mode = "4z2";
    // Low-SNR lock identity: with `--lock-fcs off` a preamble+SFD success
    // confirms the target even when the payload cannot be decoded; the grid
    // residual then discriminates target from interference.
    lock_cfg.require_fcs = (o.lock_fcs == "on");
    lock_cfg.geometry = core::target_window_geometry(
        /*sync_span=*/size_t(std::llround(256.0 * 1016.0 * 48.0 / 65.0)),
        /*sfd_span=*/size_t(std::llround(8.0 * 1016.0 * 48.0 / 65.0)),
        /*phr_span=*/size_t(std::llround(21.0 * 512.0 * 2.0 * 48.0 / 65.0)),
        /*payload_span=*/size_t(std::llround(1208.0 * 64.0 * 2.0 * 48.0 / 65.0)),
        /*pulse_halo=*/1024, pre_guard, post_guard);
    // The job buffer is allocated once for the widest geometry; a runtime
    // pre-guard update may only move the window inside that budget.
    lock_cfg.window_capacity = window;
    core::TargetLockCore lock(lock_cfg);
    core::TargetLockState last_state = lock.state();
    ConsoleReporter report(o.summary, o.print == "packets");
    report.info("START mode="+o.mode+" sync="+o.sync+
        " par_only="+o.par_only+
        " channel="+o.channel+" antenna="+o.antenna+" cir="+o.cir+
        " workers="+std::to_string(o.workers)+" summary="+std::to_string(o.summary)+
        " period_ms="+std::to_string(o.period_ms)+
        " taps="+o.taps+" taps_n="+std::to_string(taps.size())+
        " print="+o.print+
        " pre_guard="+std::to_string(pre_guard)+
        " locked_pre_guard="+std::to_string(o.locked_pre_guard)+
        " capture="+std::to_string(lock_cfg.geometry.capture)+
        " post_guard="+std::to_string(post_guard)+
        " window_capacity="+std::to_string(lock_cfg.geometry.capacity())+
        " locked_window_capacity="+
            std::to_string(o.locked_pre_guard+lock_cfg.geometry.capture+post_guard)+
        " interference="+std::string(o.interference?"on":"off")+
        " fused="+std::string(o.fused?"on":"off")+
        " ondemand="+std::string(o.ondemand_resolved?"on":"off")+
        " acq_span="+std::to_string(o.acq_span)+
        " acq_step="+std::to_string(o.acq_step)+
        " acq_persist="+std::to_string(o.acq_persist)+
        " acq_low="+std::to_string(o.acq_low)+
        " acq_peak="+std::to_string(o.acq_peak)+
        " lock_fcs="+o.lock_fcs+
        " stop_after_sfd="+o.stop_after_sfd+
        " autocorr_when_locked="+std::string(o.autocorr_suspend?"suspend":"scan"));
    // Effective pre-guard for the next cut window.  It follows the lock state:
    // the blind acquisition platform needs the wide guard, the locked
    // predictor only needs the bounded ROI.  Every window cut (target,
    // autocorrelation candidate, timed blind grid) must use this value so the
    // window start and the reported plan can never disagree.
    size_t cur_pre_guard = lock.pre_guard();
    const auto sync_pre_guard = [&]() {
        const size_t want = (lock.state() == core::TargetLockState::Locked)
                                ? o.locked_pre_guard
                                : pre_guard;
        if (want == cur_pre_guard)
            return;
        if (!lock.set_pre_guard(want))
            throw std::runtime_error(
                "runtime pre_guard update to " + std::to_string(want) +
                " rejected: pre_guard+capture+post_guard exceeds the job buffer "
                "(" + std::to_string(window) + ")");
        cur_pre_guard = want;
        std::ostringstream s;
        s << "PRE_GUARD state=" << core::target_lock_state_name(lock.state())
          << " pre_guard=" << want
          << " window_capacity="
          << (want + lock_cfg.geometry.capture + post_guard);
        report.info(s.str());
    };
    Workers workers(report,lock,o,taps,tmpl); // All worker allocations before streaming.

    const auto emit_lock = [&](const char* why) {
        const auto st = lock.stats();
        std::ostringstream s;
        s << std::fixed << std::setprecision(3) << "LOCK state="
          << core::target_lock_state_name(lock.state())
          << " why=" << why << " gen=" << lock.generation()
          << " epoch=" << lock.epoch()
          << " period_samples=" << lock.period_samples()
          << " period_ppm=" << st.period_ppm
          << " t0=" << lock.t0_exact()
          << " last_valid=" << st.last_valid_origin
          << " lock_slots=" << st.lock_slots
          << " holdover_len=" << st.holdover_length;
        report.info(s.str());
        last_state = lock.state();
    };

    pipeline::PipelineConfig cfg;
    cfg.uhd_args=o.args; cfg.uhd_channel=o.channel; cfg.uhd_antenna=o.antenna;
    cfg.uhd_subdev=o.subdev; cfg.native_rate=fs; cfg.uhd_center_freq=o.freq; cfg.uhd_gain=o.gain;
    std::unique_ptr<pipeline::ISc16StreamSource> inner;
    TimedReplaySc16Source* timed = nullptr;
    if (!o.replay.empty()) {
        if (o.mode == "timed") {
            core::TimedRxPlanConfig tcfg;
            tcfg.sample_rate = fs;
            tcfg.alignment = o.timed_alignment;
            tcfg.max_lookahead = o.timed_lookahead;
            tcfg.command_lead_s = o.timed_lead_us * 1e-6;
            auto ts = std::make_unique<TimedReplaySc16Source>(o.replay, fs, tcfg);
            timed = ts.get();
            inner = std::move(ts);
        } else {
            inner=std::make_unique<pipeline::ReplaySc16Source>(o.replay,fs,true);
        }
    } else {
        inner=std::make_unique<pipeline::UhdDeviceSc16Source>(cfg);
    }
    // Fixed-tone cancellation is a CONTINUOUS-stream module: it trains on the
    // first 0.5 ms of every 20 ms period.  Under timed_burst the stream has
    // planned gaps, so the training window can land on the packet itself and
    // corrupt it.  The device-timed tone schedule exists as a core helper
    // (core::merge_physical_rx_intervals) with QA, but it is NOT wired into the
    // live source yet, so timed mode runs with cancellation off rather than
    // silently subtracting a packet-derived model.
    core::FixedToneCancelConfig tone;
    tone.enable = o.tone && (o.mode != "timed");
    tone.period_s = o.tone_period_ms * 1e-3;
    tone.training_s = o.tone_training_ms * 1e-3;
    if (o.mode == "timed")
        report.info("TONE_CANCEL disabled reason=timed_burst_training_unscheduled");
    else if (!o.tone)
        report.info("TONE_CANCEL disabled reason=operator_off");
    // In continuous mode the tone decorator sits between the device and the
    // app.  In timed mode it must NOT: the decorator's reader thread would
    // spin on the empty space between planned windows and the scheduler (this
    // thread) could never queue the next window.  Tone cancellation is off in
    // timed mode anyway, so the source is used directly.
    std::unique_ptr<pipeline::ISc16StreamSource> source_owner;
    pipeline::FixedToneCancelSc16Source* decorator = nullptr;
    pipeline::ISc16StreamSource* src = nullptr;
    if (!tone.enable) {
        // No decorator at all when cancellation is off: wrapping a source that
        // is never cancelled only adds a bounded ring that can overflow.
        source_owner = std::move(inner);
        src = source_owner.get();
    } else {
        auto dec = std::make_unique<pipeline::FixedToneCancelSc16Source>(
            std::move(inner), tone, block, o.raw_blocks);
        decorator = dec.get();
        source_owner = std::move(dec);
        src = decorator;
    }
    const auto source_stats = [&]() -> pipeline::FixedToneCancelStats {
        if (decorator)
            return decorator->stats();
        pipeline::FixedToneCancelStats z{};
        z.source_samples = uint64_t(timed ? timed->delivered_samples() : 0);
        return z;
    };
    // Touch ring pages before UHD starts, rather than faulting them in recv.
    std::vector<std::complex<int16_t>> ring(history_size);
    std::vector<std::complex<int16_t>> incoming(block);
    Acquisition acq;
    acq.configure(int64_t(o.acq_span), int64_t(o.acq_step),
                  int64_t(o.acq_persist), o.acq_low, o.acq_peak);
    // Locked-state autocorrelation gate.  The policy was resolved during CLI
    // validation (before the source opened); only the -1 lookback default
    // depends on the configured acq_span, so it is resolved here.
    live::AutocorrGate autocorr_gate;
    autocorr_gate.enabled = o.autocorr_suspend;
    const int64_t autocorr_lookback =
        (o.autocorr_resume_lookback == -1)
            ? int64_t(o.acq_span) + int64_t(window)
            : int64_t(o.autocorr_resume_lookback);
    if (!src->open()) throw std::runtime_error("cannot open UHD/replay or tone configuration");
    if (std::abs(src->actual_rx_rate()-fs)>.5) throw std::runtime_error("unexpected RX rate");
    if (!src->start_stream()) throw std::runtime_error("cannot start RX stream");
    report.info("STREAM_STARTED native_rate=737280000 tone_hz=-232960000 period_ms=20 training_ms=0.5");
    const auto started=Clock::now(); auto snapshot=started;
    int64_t end=0, valid_begin=0; uint64_t id=0, gaps=0;
    // Autocorrelation seeds produced by the streaming frontend (continuous
    // mode only).  `id` also counts timed windows, so the capture count the
    // acquisition acceptance is stated in needs its own counter.
    WindowSubmissionTracker submissions(cur_pre_guard);
    auto& sub_stats = submissions.sub_stats;
    auto& target_slots = submissions.target_slots;
    auto& target_dropped = submissions.target_dropped;
    auto& target_expired = submissions.target_expired;
    auto& dedup = submissions.dedup;
    auto& dedup_confirmed = submissions.dedup_confirmed;
    auto& dedup_ambiguous = submissions.dedup_ambiguous;
    auto& unknown = submissions.unknown;
    auto& cand_submit_attempts = submissions.cand_submit_attempts;
    auto& cand_admitted = submissions.cand_admitted;
    auto& cand_dropped_queue = submissions.cand_dropped_queue;
    auto& cand_incomplete = submissions.cand_incomplete;
    auto& cand_overrun = submissions.cand_overrun;
    auto& cand_overlapping = submissions.cand_overlapping;
    auto& cand_invalid_gap = submissions.cand_invalid_gap;
    auto& autocorr_ignored = submissions.autocorr_ignored;
    auto& timed_pending = submissions.timed_pending;
    using TimedPending = WindowSubmissionTracker::TimedPending;
    bool failed=false;
    double frontend_us=0; uint64_t frontend_samples=0;
    // Split of frontend_us: the residual ring copy (wrap + gap blocks only) and
    // the acquisition scan.  Kept as separate diagnostics because the frontend
    // budget is decided by these two numbers.
    double frontend_copy_us=0, frontend_scan_us=0;
    int64_t timed_cursor = std::llround(o.timed_t0_ms * 1e-3 * fs);
    const int64_t nominal_period_samples = std::llround(o.period_ms * 1e-3 * fs);

    // Queue a bounded lookahead of device-timed windows.  Before the lock the
    // grid is the nominal period; after the lock it is the predicted one.  A
    // window already queued is never rewritten.  This must be callable BEFORE
    // the first read: the read blocks until a window exists.
    const auto queue_timed_windows = [&]() {
        if (!timed)
            return;
        while (timed->plan_stats().requested <
                   timed->plan_stats().issued + o.timed_lookahead &&
               timed_cursor < end + int64_t(4 * nominal_period_samples)) {
            core::TargetSlotPlan plan;
            if (lock.next_plan(timed_cursor, plan)) {
                const size_t count = size_t(plan.window_end - plan.window_start);
                if (timed->queue_window(plan.window_start, count,
                                        plan.generation)) {
                    TimedPending tp;
                    tp.id = ++id;
                    tp.seed = plan.predicted_origin;
                    tp.window_start = plan.window_start;
                    tp.predicted_rel_work =
                        std::llround((plan.predicted_origin - plan.window_start) * 65.0 / 48);
                    tp.guard = plan.pre_guard;
                    tp.slot_index = plan.slot_index;
                    tp.generation = plan.generation;
                    tp.target_slot = true;
                    tp.feeds_lock = true;
                    tp.count = count;
                    timed_pending.push_back(tp);
                    lock.commit_plan(plan);
                    ++target_slots;
                } else {
                    lock.note_slot_outcome(plan.slot_index,
                                           core::TargetSlotOutcome::DroppedQueue);
                    ++target_dropped;
                }
                timed_cursor = plan.window_end;
            } else {
                // Blind grid: the packet is expected at `timed_cursor`, so the
                // window starts one pre_guard earlier.
                const int64_t ws = timed_cursor - int64_t(cur_pre_guard);
                const int64_t win_start = ws < 0 ? 0 : ws;
                if (timed->queue_window(win_start, window,
                                        lock.generation())) {
                    TimedPending tp;
                    tp.id = ++id;
                    tp.seed = ws < 0 ? int64_t(cur_pre_guard) : timed_cursor;
                    tp.window_start = win_start;
                    tp.predicted_rel_work =
                        std::llround((tp.seed - tp.window_start) * 65.0 / 48);
                    tp.guard = cur_pre_guard;
                    tp.slot_index = std::numeric_limits<uint64_t>::max();
                    tp.generation = lock.generation();
                    tp.target_slot = false;
                    tp.feeds_lock = true;
                    tp.count = window;
                    timed_pending.push_back(tp);
                }
                timed_cursor += nominal_period_samples;
            }
        }
    };
    queue_timed_windows();

    while (!interrupted && (o.seconds==0 || std::chrono::duration<double>(Clock::now()-started).count()<o.seconds)) {
        pipeline::StreamReadMeta meta;
        // Receive straight into the ring whenever the block does not straddle
        // the wrap point: the staging copy was 45-70 ms of the 0.5 s frontend
        // budget (1.47 GB in and out per 0.5 s), i.e. as much as the whole
        // acquisition scan.  The staging buffer is still used for the one block
        // in 32 that wraps, and to relocate a block whose origin moved (gap):
        // the decorator may change the absolute origin at a gap, so the ring's
        // absolute mapping must always stay correct.
        const size_t at0 = size_t(end) & (history_size-1);
        const bool direct = (at0 + block <= history_size);
        std::complex<int16_t>* dst = direct ? ring.data()+at0 : incoming.data();
        const auto n=src->read_samples(dst,block,meta);
        if (meta.is_gap || (n && meta.start_sample_abs!=end)) {
            // A planned timed gap is expected: it is the monitoring blind
            // time, not a device error, and it must not invalidate the grid.
            const bool planned = (o.mode == "timed");
            if (!planned) ++gaps;
            report.info(std::string(planned ? "RX_PLANNED_GAP" : "RX_GAP")+
                        " sample="+std::to_string(meta.start_sample_abs)+
                        " error_code="+std::to_string(meta.error_code)+
                        " gap_samples="+std::to_string(meta.gap_samples));
            submissions.flush_pending_on_gap(report);
            end=meta.start_sample_abs; valid_begin=end; acq.reset(end);
            // The block was read at the pre-gap origin; move it to the new one.
            // Gaps are rare, so the relocation cost never shows up in the
            // steady-state budget.
            if (n && direct) {
                const size_t at = size_t(end) & (history_size-1);
                if (at + n <= history_size) {
                    std::memmove(ring.data()+at, ring.data()+at0,
                                 n*sizeof(std::complex<int16_t>));
                    dst = ring.data()+at;
                } else {
                    std::copy_n(ring.data()+at0, n, incoming.data());
                    dst = incoming.data();
                }
            }
            // An unexpected gap invalidates the grid; a planned timed gap is
            // expected and must not.
            if (o.mode != "timed") lock.note_time_discontinuity();
        }
        if (n) {
            const auto frontend_start = Clock::now();
            const size_t at=size_t(end)&(history_size-1);
            if (dst != ring.data()+at) {
                const size_t first=std::min(n,history_size-at);
                std::copy_n(incoming.data(),first,ring.data()+at);
                std::copy_n(incoming.data()+first,n-first,ring.data());
            }
            end+=n; valid_begin=std::max(valid_begin,end-int64_t(history_size));
            const auto copy_done = Clock::now();
            if (o.mode != "timed") {
                // Locked-state scan gate: while the lock is tracking the scan
                // may be suspended (see --autocorr-when-locked).  On resume the
                // acquisition is reset into the backlog so packets that arrived
                // during the teardown latency are still scanned.
                const auto gate_action = autocorr_gate.update(
                    lock.scheduled(), end, valid_begin, autocorr_lookback, n);
                if (gate_action.reset)
                    acq.reset(gate_action.reset_at);
                // A block reads [probe, probe+step-1] and [probe+lag,
                // probe+step-1+lag], so the tail of the block must already be
                // inside the valid range.
                if (gate_action.scan) {
                    while (acq.probe+acq.step-1+Acquisition::lag<end) {
                        double metric = 0.0;
                        const auto seed=acq.scan(ring,&metric);
                        if (seed<0) continue;
                        submissions.handle_autocorr_seed(
                            seed, metric, valid_begin, cur_pre_guard, window,
                            o.interference, id, lock, report);
                    }
                }
                submissions.submit_ready_acquire_window(
                    end, valid_begin, window, ring, lock, workers, report);
            }
            frontend_us += elapsed_us(frontend_start, Clock::now());
            frontend_copy_us += elapsed_us(frontend_start, copy_done);
            frontend_scan_us += elapsed_us(copy_done, Clock::now());
            frontend_samples += n;
        }
        // ---- deferred candidate submissions -------------------------------
        // A candidate window is cut `pre_guard` before its seed, so it only
        // becomes available once enough samples have arrived.  Waiting here
        // (rather than dropping it) is what keeps every autocorrelation
        // candidate accounted for.
        submissions.submit_ready_deferred_candidates(
            end, valid_begin, ring, lock, workers, report);
        // ---- target path (continuous) ------------------------------------
        // The chosen slot is remembered across iterations, but its window is
        // RE-COMPUTED from the current model every time: a plan that has not
        // been handed to the workers yet must follow a period/phase update.
        // (Caching the window itself cut the first post-lock slots with the
        // nominal 5 ms period and missed them by ~300 samples.)
        if (o.mode != "timed") {
            // A lock-state change moves the guard (and with it the window
            // start); apply it before the next plan is chosen so the plan and
            // the cut window are always cut from the same geometry.
            sync_pre_guard();
            submissions.submit_continuous_target_windows(
                end, valid_begin, ring, id, lock, workers, report);
        }
        // ---- target path (timed) -----------------------------------------
        if (o.mode == "timed" && timed) {
            // Submit every planned window whose samples are now in the ring.
            submissions.submit_timed_windows(
                end, valid_begin, ring, lock, workers, report);
            queue_timed_windows();
        }
        // ---- lock feedback ------------------------------------------------
        lock.drain_observations();
        // Resolve deferred autocorrelation associations now that the target
        // slot results are in.
        submissions.resolve_deferred_associations(lock, report);
        if (lock.state() != last_state) {
            emit_lock(lock.state() == core::TargetLockState::Locked ? "converged"
                     : lock.state() == core::TargetLockState::Holdover ? "target_miss"
                     : lock.state() == core::TargetLockState::Reacquire ? "grid_invalid"
                     : "seeded");
            // The guard follows the state: LOCKED only needs the bounded ROI,
            // every other state needs the wide acquisition platform.
            sync_pre_guard();
        }
        if (Clock::now()-snapshot>std::chrono::milliseconds(100)) {
            report.snapshot(source_stats(),gaps,workers.backlog()); snapshot=Clock::now();
        }
        std::string error;
        if (src->fatal_error(&error)) {report.info("SOURCE_ERROR "+error);failed=true;break;}
        error = workers.failure();
        if (!error.empty()) {report.info("WORKER_ERROR "+error);failed=true;break;}
        if (meta.is_eof || (!n && src->is_eof())) break;
    }
    src->close(); // Stop hardware before draining workers / printing.
    report.snapshot(source_stats(),gaps,workers.backlog());
    submissions.flush_pending_at_eof(lock, report);
    workers.stop();
    // Workers stop only after every queued job finished, so the last batch of
    // observations must still be applied before the final statistics.
    lock.drain_observations();
    report.snapshot(source_stats(),gaps,workers.backlog());
    const auto st = source_stats();
    std::ostringstream costs;
    costs << std::fixed << std::setprecision(3) << "COST_TOTAL frontend_us=" << frontend_us
          << " frontend_copy_us=" << frontend_copy_us
          << " frontend_scan_us=" << frontend_scan_us
          // Acquisition capture count; `completed` is limited by how many jobs
          // the demod drained before EOF, not by detection.
          << " acq_seeds=" << sub_stats.acq_seeds
          << " frontend_samples=" << frontend_samples << " estimate_us=" << st.estimate_us
          << " cancel_us=" << st.cancel_us << " cancelled_samples=" << st.cancelled_samples
          << " bypassed_samples=" << st.bypassed_samples
          << " model_updates=" << st.model_updates
          << " model_rejects=" << st.model_rejects
          << " training_windows=" << st.training_windows
          << " reader_us=" << st.reader_us;
    report.info(costs.str());
    {
        const auto ls = lock.stats();
        std::ostringstream s;
        s << "LOCK_STATS final_state=" << core::target_lock_state_name(lock.state())
          << " expected_slots=" << ls.expected_slots
          << " submitted=" << ls.submitted
          << " dropped_queue=" << ls.dropped_queue
          << " expired=" << ls.expired
          << " not_detected=" << ls.not_detected
          << " decode_failed=" << ls.decode_failed
          << " fcs_ok=" << ls.fcs_ok
          << " fcs_failed=" << ls.fcs_failed
          << " observations=" << ls.observations
          << " stale=" << ls.stale_observations
          << " duplicate=" << ls.duplicate_observations
          << " identity_rejects=" << ls.identity_rejects
          << " out_of_grid=" << ls.out_of_grid
          << " queue_full=" << ls.queue_full
          << " residual_outliers=" << ls.residual_outliers
          << " lock_entries=" << ls.lock_entries
          << " holdover_entries=" << ls.holdover_entries
          << " reacquire_entries=" << ls.reacquire_entries
          << " discontinuities=" << ls.discontinuity_events
          << " candidates=" << ls.candidates
          << " cand_target=" << ls.candidates_target
          << " cand_interference=" << ls.candidates_interference
          << " cand_unknown=" << ls.candidates_unknown
          << " period_updates=" << ls.period_updates
          << " pre_guard_updates=" << ls.pre_guard_updates
          << " lock_slots=" << ls.lock_slots
          << " period_samples=" << ls.period_samples
          << " period_ppm=" << ls.period_ppm
          << " scheduler_target_slots=" << target_slots
          << " scheduler_target_dropped=" << target_dropped
          << " scheduler_target_expired=" << target_expired
          << " autocorr_dedup=" << dedup
          << " autocorr_dedup_confirmed=" << dedup_confirmed
          << " autocorr_dedup_ambiguous=" << dedup_ambiguous
          << " autocorr_unknown=" << unknown
          << " interference_events=" << workers.interference_events.load()
          << " cand_submit_attempts=" << cand_submit_attempts
          << " cand_admitted=" << cand_admitted
          << " cand_dropped_queue=" << cand_dropped_queue
          << " cand_incomplete=" << cand_incomplete
          << " cand_overrun=" << cand_overrun
          << " cand_overlapping=" << cand_overlapping
          << " cand_invalid_gap=" << cand_invalid_gap
          << " autocorr_ignored=" << autocorr_ignored
          << " autocorr_suspend_events=" << autocorr_gate.suspend_events
          << " autocorr_resume_events=" << autocorr_gate.resume_events
          << " autocorr_suspended_samples=" << autocorr_gate.suspended_samples
          << " autocorr_resume_backlog_samples=" << autocorr_gate.resume_backlog_samples;
        report.info(s.str());
    }
    if (timed) {
        const auto& ts = timed->plan_stats();
        std::ostringstream s;
        s << "TIMED_PLAN requested=" << ts.requested
          << " issued=" << ts.issued
          << " completed=" << ts.completed
          << " partial=" << ts.partial
          << " timeouts=" << ts.timeouts
          << " late_commands=" << ts.late_commands
          << " overflows=" << ts.overflows
          << " device_time_jumps=" << ts.device_time_jumps
          << " planned_gaps=" << ts.planned_gaps
          << " planned_gap_samples=" << ts.planned_gap_samples
          << " unexpected_gaps=" << ts.unexpected_gaps
          << " unexpected_gap_samples=" << ts.unexpected_gap_samples
          << " cancelled=" << ts.cancelled
          << " request_queue_full=" << ts.request_queue_full
          << " lookahead_blocked=" << ts.lookahead_blocked
          << " alignment_expanded_samples=" << ts.alignment_expanded_samples;
        report.info(s.str());
    }
    report.finish(sub_stats);
    return failed ? 2 : 0;
}

} // namespace live
} // namespace openuwb
