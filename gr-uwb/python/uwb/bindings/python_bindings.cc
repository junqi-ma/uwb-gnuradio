/*
 * Copyright 2020 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <pmt/pmt.h>

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_energy_detector.h>
#include <gnuradio/uwb/uwb_preamble_detector.h>
#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_detector_sc16.h>
#include <gnuradio/uwb/uwb_packet_writer.h>
#include <gnuradio/uwb/uwb_realtime_demodulator.h>
#include <gnuradio/uwb/uwb_scheduled_extractor.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_sc16.h>
#include <gnuradio/uwb/uwb_auto_scheduled_extractor_sc16.h>
#include <gnuradio/uwb/uwb_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_48.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h>
#include <gnuradio/uwb/uwb_pdu_window_crop.h>
#include <gnuradio/uwb/uwb_radar_packet_source.h>
#include <gnuradio/uwb/uwb_hrp_packet_source.h>
#include <gnuradio/uwb/uwb_loopback_echo.h>
#include <gnuradio/uwb/uwb_radar_cir_estimator_block.h>
#include <gnuradio/uwb/uwb_cir_writer.h>
#include <gnuradio/uwb/uwb_echo_burst_backend.h>
#include <gnuradio/uwb/uwb_echo_timer_stream.h>
#include <gnuradio/uwb/uwb_realtime_echo_timer.h>
#include <gnuradio/uwb/uwb_fake_burst_backend.h>
#ifdef UWB_HAVE_UHD
#include <gnuradio/uwb/uwb_uhd_burst_backend.h> // assign_tx_channels helper
#endif

namespace py = pybind11;

void bind_energy_detector(py::module& m)
{
    py::class_<gr::uwb::UwbEnergyDetector,
               gr::sync_block,
               std::shared_ptr<gr::uwb::UwbEnergyDetector>>(m,
                                                             "energy_detector")
        .def(py::init(&gr::uwb::UwbEnergyDetector::make),
             py::arg("threshold") = 0.5f,
             py::arg("window") = size_t(16))
        .def("threshold", &gr::uwb::UwbEnergyDetector::threshold)
        .def("set_threshold",
             &gr::uwb::UwbEnergyDetector::set_threshold,
             py::arg("threshold"))
        .def("window", &gr::uwb::UwbEnergyDetector::window)
        .def("set_window",
             &gr::uwb::UwbEnergyDetector::set_window,
             py::arg("window"));
}

void bind_preamble_detector(py::module& m)
{
    py::class_<gr::uwb::UwbPreambleDetector,
               gr::sync_block,
               std::shared_ptr<gr::uwb::UwbPreambleDetector>>(m,
                                                               "preamble_detector")
        .def(py::init(&gr::uwb::UwbPreambleDetector::make),
             py::arg("known_preamble"),
             py::arg("threshold") = 0.7f,
             py::arg("energy_threshold") = 0.0f,
             py::arg("energy_window") = size_t(8),
             py::arg("energy_decimation") = size_t(1),
             py::arg("coarse_decimation") = size_t(8),
             py::arg("coarse_repetitions") = size_t(1),
             py::arg("coarse_margin") = size_t(16))
        .def_static("make_from_file",
                    &gr::uwb::UwbPreambleDetector::make_from_file,
                    py::arg("template_file"),
                    py::arg("threshold") = 0.7f,
                    py::arg("energy_threshold") = 0.0f,
                    py::arg("energy_window") = size_t(8),
                    py::arg("energy_decimation") = size_t(1),
                    py::arg("coarse_decimation") = size_t(8),
                    py::arg("coarse_repetitions") = size_t(1),
                    py::arg("coarse_margin") = size_t(16))
        .def("threshold", &gr::uwb::UwbPreambleDetector::threshold)
        .def("set_threshold",
             &gr::uwb::UwbPreambleDetector::set_threshold,
             py::arg("threshold"))
        .def("template_length", &gr::uwb::UwbPreambleDetector::template_length)
        .def("energy_threshold", &gr::uwb::UwbPreambleDetector::energy_threshold)
        .def("set_energy_threshold",
             &gr::uwb::UwbPreambleDetector::set_energy_threshold,
             py::arg("energy_threshold"))
        .def("energy_window", &gr::uwb::UwbPreambleDetector::energy_window)
        .def("set_energy_window",
             &gr::uwb::UwbPreambleDetector::set_energy_window,
             py::arg("energy_window"))
        .def("energy_decimation", &gr::uwb::UwbPreambleDetector::energy_decimation)
        .def("set_energy_decimation",
             &gr::uwb::UwbPreambleDetector::set_energy_decimation,
             py::arg("energy_decimation"))
        .def("coarse_decimation", &gr::uwb::UwbPreambleDetector::coarse_decimation)
        .def("set_coarse_decimation",
             &gr::uwb::UwbPreambleDetector::set_coarse_decimation,
             py::arg("coarse_decimation"))
        .def("coarse_repetitions",
             &gr::uwb::UwbPreambleDetector::coarse_repetitions)
        .def("set_coarse_repetitions",
             &gr::uwb::UwbPreambleDetector::set_coarse_repetitions,
             py::arg("coarse_repetitions"));
}

void bind_detector(py::module& m)
{
    using gr::uwb::defaults::kDetectorCapture;
    using gr::uwb::defaults::kDetectorCoarseDecimation;
    using gr::uwb::defaults::kDetectorCoarseMargin;
    using gr::uwb::defaults::kDetectorCoarseRepetitions;
    using gr::uwb::defaults::kDetectorEnergyGateDecimation;
    using gr::uwb::defaults::kDetectorEnergyThreshold;
    using gr::uwb::defaults::kDetectorPreTrigger;
    using gr::uwb::defaults::kSampleRateHz;

    py::class_<gr::uwb::UwbDetector,
               gr::sync_block,
               std::shared_ptr<gr::uwb::UwbDetector>>(m, "detector")
        .def(py::init(&gr::uwb::UwbDetector::make),
             py::arg("known_preamble"),
             py::arg("pre_trigger") = kDetectorPreTrigger,
             py::arg("capture") = kDetectorCapture,
             py::arg("energy_threshold") = kDetectorEnergyThreshold,
             py::arg("energy_gate_decimation") = kDetectorEnergyGateDecimation,
             py::arg("coarse_decimation") = kDetectorCoarseDecimation,
             py::arg("coarse_repetitions") = kDetectorCoarseRepetitions,
             py::arg("coarse_margin") = kDetectorCoarseMargin,
             py::arg("sample_rate") = kSampleRateHz)
        .def_static("make_from_file",
                    &gr::uwb::UwbDetector::make_from_file,
                    py::arg("template_file"),
                    py::arg("pre_trigger") = kDetectorPreTrigger,
                    py::arg("capture") = kDetectorCapture,
                    py::arg("energy_threshold") = kDetectorEnergyThreshold,
                    py::arg("energy_gate_decimation") =
                        kDetectorEnergyGateDecimation,
                    py::arg("coarse_decimation") = kDetectorCoarseDecimation,
                    py::arg("coarse_repetitions") = kDetectorCoarseRepetitions,
                    py::arg("coarse_margin") = kDetectorCoarseMargin,
                    py::arg("sample_rate") = kSampleRateHz)
        .def("pre_trigger", &gr::uwb::UwbDetector::pre_trigger)
        .def("set_pre_trigger",
             &gr::uwb::UwbDetector::set_pre_trigger,
             py::arg("pre_trigger"))
        .def("capture", &gr::uwb::UwbDetector::capture)
        .def("set_capture",
             &gr::uwb::UwbDetector::set_capture,
             py::arg("capture"))
        .def("sample_rate", &gr::uwb::UwbDetector::sample_rate)
        .def("set_sample_rate",
             &gr::uwb::UwbDetector::set_sample_rate,
             py::arg("sample_rate"))
        .def("dropped_regions", &gr::uwb::UwbDetector::dropped_regions);
}

void bind_detector_sc16(py::module& m)
{
    using gr::uwb::defaults::kDetectorCapture;
    using gr::uwb::defaults::kDetectorCoarseDecimation;
    using gr::uwb::defaults::kDetectorCoarseMargin;
    using gr::uwb::defaults::kDetectorCoarseRepetitions;
    using gr::uwb::defaults::kDetectorEnergyGateDecimation;
    using gr::uwb::defaults::kDetectorEnergyThreshold;
    using gr::uwb::defaults::kDetectorPreTrigger;
    using gr::uwb::defaults::kSampleRateHz;

    py::class_<gr::uwb::UwbDetectorSc16,
               gr::sync_block,
               std::shared_ptr<gr::uwb::UwbDetectorSc16>>(m, "detector_sc16")
        .def(py::init(&gr::uwb::UwbDetectorSc16::make),
             py::arg("known_preamble"),
             py::arg("pre_trigger") = kDetectorPreTrigger,
             py::arg("capture") = kDetectorCapture,
             py::arg("energy_threshold") = kDetectorEnergyThreshold,
             py::arg("energy_gate_decimation") = kDetectorEnergyGateDecimation,
             py::arg("coarse_decimation") = kDetectorCoarseDecimation,
             py::arg("coarse_repetitions") = kDetectorCoarseRepetitions,
             py::arg("coarse_margin") = kDetectorCoarseMargin,
             py::arg("sample_rate") = kSampleRateHz)
        .def_static("make_from_file", &gr::uwb::UwbDetectorSc16::make_from_file,
                    py::arg("template_file"),
                    py::arg("pre_trigger") = kDetectorPreTrigger,
                    py::arg("capture") = kDetectorCapture,
                    py::arg("energy_threshold") = kDetectorEnergyThreshold,
                    py::arg("energy_gate_decimation") =
                        kDetectorEnergyGateDecimation,
                    py::arg("coarse_decimation") = kDetectorCoarseDecimation,
                    py::arg("coarse_repetitions") = kDetectorCoarseRepetitions,
                    py::arg("coarse_margin") = kDetectorCoarseMargin,
                    py::arg("sample_rate") = kSampleRateHz)
        .def("sample_rate", &gr::uwb::UwbDetectorSc16::sample_rate)
        .def("set_sample_rate",
             &gr::uwb::UwbDetectorSc16::set_sample_rate,
             py::arg("sample_rate"))
        .def("dropped_regions", &gr::uwb::UwbDetectorSc16::dropped_regions)
        .def("work_calls", &gr::uwb::UwbDetectorSc16::work_calls)
        .def("work_mean_noutput_items",
             &gr::uwb::UwbDetectorSc16::work_mean_noutput_items);
}

void bind_packet_writer(py::module& m)
{
    py::class_<gr::uwb::UwbPacketWriter,
               gr::block,
               std::shared_ptr<gr::uwb::UwbPacketWriter>>(m, "packet_writer")
        .def(py::init(&gr::uwb::UwbPacketWriter::make),
             py::arg("directory"),
             py::arg("base_name") = std::string("capture"),
             py::arg("one_file_per_packet") = false)
        .def("directory", &gr::uwb::UwbPacketWriter::directory)
        .def("base_name", &gr::uwb::UwbPacketWriter::base_name)
        .def("one_file_per_packet", &gr::uwb::UwbPacketWriter::one_file_per_packet)
        .def("packets_written", &gr::uwb::UwbPacketWriter::packets_written)
        .def("samples_written", &gr::uwb::UwbPacketWriter::samples_written)
        .def("packets_received", &gr::uwb::UwbPacketWriter::packets_received)
        .def("packets_dropped", &gr::uwb::UwbPacketWriter::packets_dropped)
        .def("queue_high_watermark",
             &gr::uwb::UwbPacketWriter::queue_high_watermark);
}

void bind_scheduled_extractor(py::module& m)
{
    py::class_<gr::uwb::UwbScheduledExtractor,
               gr::sync_block,
               std::shared_ptr<gr::uwb::UwbScheduledExtractor>>
        se(m, "scheduled_extractor");
    py::enum_<gr::uwb::UwbScheduledExtractor::EmitPolicy>(se, "EmitPolicy")
        .value("EverySlot", gr::uwb::UwbScheduledExtractor::EmitPolicy::EverySlot)
        .value("VerifiedOnly",
               gr::uwb::UwbScheduledExtractor::EmitPolicy::VerifiedOnly)
        .export_values();
    se.def(py::init(&gr::uwb::UwbScheduledExtractor::make),
           py::arg("sample_rate"),
           py::arg("packet_interval_s"),
           py::arg("first_packet_sample"),
           py::arg("pre_guard_samples") =
               gr::uwb::defaults::kScheduledPreGuard,
           py::arg("capture_samples") = gr::uwb::defaults::kScheduledCapture,
           py::arg("post_guard_samples") =
               gr::uwb::defaults::kScheduledPostGuard,
           py::arg("pool_size") = gr::uwb::defaults::kScheduledPoolSize,
           py::arg("emit_policy") =
               gr::uwb::UwbScheduledExtractor::EmitPolicy::EverySlot,
           py::arg("verification_enabled") = false,
           py::arg("radar_template") = std::vector<std::complex<float>>(),
           py::arg("radar_threshold") = 0.5f,
           py::arg("comm_template") = std::vector<std::complex<float>>(),
           py::arg("comm_threshold") = 0.5f)
        .def("set_schedule",
             &gr::uwb::UwbScheduledExtractor::set_schedule,
             py::arg("first_packet_sample"),
             py::arg("packet_interval_s"))
        .def("pause_schedule", &gr::uwb::UwbScheduledExtractor::pause_schedule)
        .def("resume_schedule", &gr::uwb::UwbScheduledExtractor::resume_schedule)
        .def("reset_schedule", &gr::uwb::UwbScheduledExtractor::reset_schedule)
        .def("set_verification_enabled",
             &gr::uwb::UwbScheduledExtractor::set_verification_enabled,
             py::arg("en"))
        .def("set_schedule_lock_enabled",
             &gr::uwb::UwbScheduledExtractor::set_schedule_lock_enabled,
             py::arg("en"))
        .def("schedule_lock_enabled",
             &gr::uwb::UwbScheduledExtractor::schedule_lock_enabled)
        .def("schedule_lock_state",
             &gr::uwb::UwbScheduledExtractor::schedule_lock_state)
        .def("schedule_lock_updates",
             &gr::uwb::UwbScheduledExtractor::schedule_lock_updates)
        .def("locked_packet_interval_s",
             &gr::uwb::UwbScheduledExtractor::locked_packet_interval_s)
        .def("locked_first_packet_sample",
             &gr::uwb::UwbScheduledExtractor::locked_first_packet_sample)
        .def("locked_delta_period_samples",
             &gr::uwb::UwbScheduledExtractor::locked_delta_period_samples)
        .def("locked_bias_t0_samples",
             &gr::uwb::UwbScheduledExtractor::locked_bias_t0_samples)
        .def("observe_detection",
             &gr::uwb::UwbScheduledExtractor::observe_detection,
             py::arg("schedule_index"),
             py::arg("detected_start_sample"))
        .def("scheduled_windows",
             &gr::uwb::UwbScheduledExtractor::scheduled_windows)
        .def("completed_windows",
             &gr::uwb::UwbScheduledExtractor::completed_windows)
        .def("emitted_windows", &gr::uwb::UwbScheduledExtractor::emitted_windows)
        .def("dropped_windows", &gr::uwb::UwbScheduledExtractor::dropped_windows)
        .def("queue_high_watermark",
             &gr::uwb::UwbScheduledExtractor::queue_high_watermark);

    py::class_<gr::uwb::UwbScheduledExtractorSc16,
               gr::sync_block,
               std::shared_ptr<gr::uwb::UwbScheduledExtractorSc16>>(
        m, "scheduled_extractor_sc16")
        .def(py::init(&gr::uwb::UwbScheduledExtractorSc16::make),
             py::arg("sample_rate"),
             py::arg("packet_interval_s"),
             py::arg("first_packet_sample"),
             py::arg("pre_guard_samples") = gr::uwb::defaults::kScheduledPreGuard,
             py::arg("capture_samples") = gr::uwb::defaults::kScheduledCapture,
             py::arg("post_guard_samples") = gr::uwb::defaults::kScheduledPostGuard,
             py::arg("pool_size") = gr::uwb::defaults::kScheduledPoolSize,
             py::arg("emit_policy") =
                 gr::uwb::UwbScheduledExtractorSc16::EmitPolicy::EverySlot)
        .def("scheduled_windows", &gr::uwb::UwbScheduledExtractorSc16::scheduled_windows)
        .def("completed_windows", &gr::uwb::UwbScheduledExtractorSc16::completed_windows)
        .def("emitted_windows", &gr::uwb::UwbScheduledExtractorSc16::emitted_windows)
        .def("dropped_windows", &gr::uwb::UwbScheduledExtractorSc16::dropped_windows)
        .def("process_total_us", &gr::uwb::UwbScheduledExtractorSc16::process_total_us)
        .def("copy_total_us", &gr::uwb::UwbScheduledExtractorSc16::copy_total_us)
        .def("publish_total_us", &gr::uwb::UwbScheduledExtractorSc16::publish_total_us);
}

void bind_auto_scheduled_extractor_sc16(py::module& m)
{
    using Blk = gr::uwb::UwbAutoScheduledExtractorSc16;
    py::class_<Blk, gr::sync_block, std::shared_ptr<Blk>>(
        m, "auto_scheduled_extractor_sc16")
        .def(py::init(&Blk::make),
             py::arg("known_preamble"),
             py::arg("sample_rate") = gr::uwb::defaults::kNativeSampleRateHz,
             py::arg("packet_interval_s") =
                 gr::uwb::defaults::kQm35PacketIntervalS,
             py::arg("pre_guard_samples") =
                 gr::uwb::defaults::kNativeScheduledPreGuard,
             py::arg("capture_samples") =
                 gr::uwb::defaults::kNativeScheduledCapture,
             py::arg("post_guard_samples") =
                 gr::uwb::defaults::kNativeScheduledPostGuard,
             py::arg("energy_threshold") =
                 gr::uwb::defaults::kDetectorEnergyThreshold,
             py::arg("energy_gate_decimation") =
                 gr::uwb::defaults::kDetectorEnergyGateDecimation,
             py::arg("coarse_decimation") =
                 gr::uwb::defaults::kDetectorCoarseDecimation,
             py::arg("coarse_repetitions") =
                 gr::uwb::defaults::kDetectorCoarseRepetitions,
             py::arg("coarse_margin") = gr::uwb::defaults::kDetectorCoarseMargin,
             py::arg("lock_observations") = gr::uwb::defaults::kLockObservations,
             py::arg("holdover_miss_count") =
                 gr::uwb::defaults::kHoldoverMissCount,
             py::arg("reacquire_miss_count") =
                 gr::uwb::defaults::kReacquireMissCount,
             py::arg("provisional_guard_us") =
                 gr::uwb::defaults::kProvisionalGuardUs,
             py::arg("acquire_pre_trigger") =
                 gr::uwb::defaults::kDetectorPreTrigger,
             py::arg("acquire_capture") = gr::uwb::defaults::kDetectorCapture,
             py::arg("scheduled_pool_size") =
                 gr::uwb::defaults::kAutoScheduledPoolSize)
        .def_static("make_from_file",
                    &Blk::make_from_file,
                    py::arg("template_file"),
                    py::arg("sample_rate") =
                        gr::uwb::defaults::kNativeSampleRateHz,
                    py::arg("packet_interval_s") =
                        gr::uwb::defaults::kQm35PacketIntervalS,
                    py::arg("pre_guard_samples") =
                        gr::uwb::defaults::kNativeScheduledPreGuard,
                    py::arg("capture_samples") =
                        gr::uwb::defaults::kNativeScheduledCapture,
                    py::arg("post_guard_samples") =
                        gr::uwb::defaults::kNativeScheduledPostGuard,
                    py::arg("energy_threshold") =
                        gr::uwb::defaults::kDetectorEnergyThreshold,
                    py::arg("energy_gate_decimation") =
                        gr::uwb::defaults::kDetectorEnergyGateDecimation,
                    py::arg("coarse_decimation") =
                        gr::uwb::defaults::kDetectorCoarseDecimation,
                    py::arg("coarse_repetitions") =
                        gr::uwb::defaults::kDetectorCoarseRepetitions,
                    py::arg("coarse_margin") =
                        gr::uwb::defaults::kDetectorCoarseMargin,
                    py::arg("lock_observations") =
                        gr::uwb::defaults::kLockObservations,
                    py::arg("holdover_miss_count") =
                        gr::uwb::defaults::kHoldoverMissCount,
                    py::arg("reacquire_miss_count") =
                        gr::uwb::defaults::kReacquireMissCount,
                    py::arg("provisional_guard_us") =
                        gr::uwb::defaults::kProvisionalGuardUs,
                    py::arg("acquire_pre_trigger") =
                        gr::uwb::defaults::kDetectorPreTrigger,
                    py::arg("acquire_capture") =
                        gr::uwb::defaults::kDetectorCapture,
                    py::arg("scheduled_pool_size") =
                        gr::uwb::defaults::kAutoScheduledPoolSize)
        .def("lock_state", &Blk::lock_state)
        .def("lock_state_name", &Blk::lock_state_name)
        .def("schedule_generation", &Blk::schedule_generation)
        .def("acquisition_epoch", &Blk::acquisition_epoch)
        .def("identity_confirmed", &Blk::identity_confirmed)
        .def("locked_t0", &Blk::locked_t0)
        .def("locked_period_s", &Blk::locked_period_s)
        .def("energy_regions", &Blk::energy_regions)
        .def("energy_regions_after_lock", &Blk::energy_regions_after_lock)
        .def("candidates_emitted", &Blk::candidates_emitted)
        .def("candidates_rejected", &Blk::candidates_rejected)
        .def("scheduled_windows", &Blk::scheduled_windows)
        .def("emitted_windows", &Blk::emitted_windows)
        .def("dropped_windows", &Blk::dropped_windows)
        .def("pool_drops", &Blk::pool_drops)
        .def("queue_full_drops", &Blk::queue_full_drops)
        .def("stale_feedback", &Blk::stale_feedback)
        .def("unmapped_feedback", &Blk::unmapped_feedback)
        .def("discontinuities", &Blk::discontinuities)
        .def("post_lock_obs", &Blk::post_lock_obs, py::arg("msg"))
        .def("post_control", &Blk::post_control, py::arg("msg"));
}

void bind_realtime_demodulator(py::module& m)
{
    py::class_<gr::uwb::UwbRealtimeDemodulator,
               gr::block,
               std::shared_ptr<gr::uwb::UwbRealtimeDemodulator>>(
        m, "realtime_demodulator")
        .def(py::init(&gr::uwb::UwbRealtimeDemodulator::make),
             py::arg("template_path"),
             py::arg("num_workers") = size_t(2),
             py::arg("queue_capacity") = size_t(64),
             py::arg("sfd_mode") = std::string("4z2"),
             py::arg("cir_rake_top_k") = size_t(0),
             py::arg("cir_filter_mode") = std::string("bypass"),
             py::arg("code_index") = size_t(9),
             py::arg("preamble_repetitions") = size_t(64),
             py::arg("timing_coarse_stride") = size_t(14))
        .def_static("make_from_template",
                    &gr::uwb::UwbRealtimeDemodulator::make_from_template,
                    py::arg("template_wf"),
                    py::arg("num_workers") = size_t(2),
                    py::arg("queue_capacity") = size_t(64),
                    py::arg("sfd_mode") = std::string("4z2"),
                    py::arg("cir_rake_top_k") = size_t(0),
                    py::arg("cir_filter_mode") = std::string("bypass"),
                    py::arg("code_index") = size_t(9),
                    py::arg("preamble_repetitions") = size_t(64),
                    py::arg("timing_coarse_stride") = size_t(14))
        .def("jobs_received", &gr::uwb::UwbRealtimeDemodulator::jobs_received)
        .def("jobs_completed", &gr::uwb::UwbRealtimeDemodulator::jobs_completed)
        .def("jobs_failed", &gr::uwb::UwbRealtimeDemodulator::jobs_failed)
        .def("jobs_dropped", &gr::uwb::UwbRealtimeDemodulator::jobs_dropped)
        .def("invalid_inputs",
             &gr::uwb::UwbRealtimeDemodulator::invalid_inputs)
        .def("worker_exceptions",
             &gr::uwb::UwbRealtimeDemodulator::worker_exceptions)
        .def("queue_depth", &gr::uwb::UwbRealtimeDemodulator::queue_depth)
        .def("queue_high_watermark",
             &gr::uwb::UwbRealtimeDemodulator::queue_high_watermark)
        .def("num_workers", &gr::uwb::UwbRealtimeDemodulator::num_workers)
        .def("timing_coarse_stride",
             &gr::uwb::UwbRealtimeDemodulator::timing_coarse_stride)
        .def("latency_p50_us", &gr::uwb::UwbRealtimeDemodulator::latency_p50_us)
        .def("latency_p95_us", &gr::uwb::UwbRealtimeDemodulator::latency_p95_us)
        .def("latency_p99_us", &gr::uwb::UwbRealtimeDemodulator::latency_p99_us)
        .def("latency_max_us", &gr::uwb::UwbRealtimeDemodulator::latency_max_us)
        .def("worker_utilization_pct",
             &gr::uwb::UwbRealtimeDemodulator::worker_utilization_pct)
        .def("drained", &gr::uwb::UwbRealtimeDemodulator::drained)
        .def("drain", &gr::uwb::UwbRealtimeDemodulator::drain)
        .def("reset_stats", &gr::uwb::UwbRealtimeDemodulator::reset_stats);
}

void bind_rational_resampler_ccf_65_48(py::module& m)
{
    py::class_<gr::uwb::UwbRationalResamplerCcf65_48,
               gr::block,
               std::shared_ptr<gr::uwb::UwbRationalResamplerCcf65_48>>(
        m, "rational_resampler_ccf_65_48")
        .def(py::init(&gr::uwb::UwbRationalResamplerCcf65_48::make),
             py::arg("taps_file_or_profile") = std::string("quality"),
             py::arg("tag_propagation_enable") = true,
             py::arg("reset_on_discontinuity") = true,
             py::arg("num_workers") = 1)
        .def_static("make_from_taps",
                    &gr::uwb::UwbRationalResamplerCcf65_48::make_from_taps,
                    py::arg("taps"),
                    py::arg("tag_propagation_enable") = true,
                    py::arg("reset_on_discontinuity") = true,
                    py::arg("num_workers") = 1)
        .def("taps", &gr::uwb::UwbRationalResamplerCcf65_48::taps)
        .def("tap_count", &gr::uwb::UwbRationalResamplerCcf65_48::tap_count)
        .def("arm_length", &gr::uwb::UwbRationalResamplerCcf65_48::arm_length)
        .def("kernel_name", &gr::uwb::UwbRationalResamplerCcf65_48::kernel_name)
        .def("num_workers", &gr::uwb::UwbRationalResamplerCcf65_48::num_workers)
        .def("set_num_workers",
             &gr::uwb::UwbRationalResamplerCcf65_48::set_num_workers,
             py::arg("n"))
        .def("set_kernel",
             &gr::uwb::UwbRationalResamplerCcf65_48::set_kernel,
             py::arg("name"))
        .def("input_items", &gr::uwb::UwbRationalResamplerCcf65_48::input_items)
        .def("output_items",
             &gr::uwb::UwbRationalResamplerCcf65_48::output_items)
        .def("resets", &gr::uwb::UwbRationalResamplerCcf65_48::resets)
        .def("discontinuities",
             &gr::uwb::UwbRationalResamplerCcf65_48::discontinuities)
        .def("tag_errors", &gr::uwb::UwbRationalResamplerCcf65_48::tag_errors)
        .def("reset", &gr::uwb::UwbRationalResamplerCcf65_48::reset)
        .def("map_input_offset_to_output",
             &gr::uwb::UwbRationalResamplerCcf65_48::map_input_offset_to_output,
             py::arg("p"));
}

void bind_rational_resampler_ccf_65_32(py::module& m)
{
    using Blk = gr::uwb::UwbRationalResamplerCcf65_32;

    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(
        m, "rational_resampler_ccf_65_32")
        .def(py::init(&Blk::make),
             py::arg("taps_file_or_profile") = std::string("quality_minorder"),
             py::arg("map_radar_tags") = true,
             py::arg("num_workers") = 1,
             py::arg("lengthtagname") = std::string("packet_len"))
        .def_static("make_from_taps",
                    &Blk::make_from_taps,
                    py::arg("taps"),
                    py::arg("map_radar_tags") = true,
                    py::arg("num_workers") = 1,
                    py::arg("lengthtagname") = std::string("packet_len"))
        .def("taps", &Blk::taps)
        .def("tap_count", &Blk::tap_count)
        .def("map_radar_tags", &Blk::map_radar_tags)
        .def("kernel_name", &Blk::kernel_name)
        .def("num_workers", &Blk::num_workers)
        .def("set_num_workers", &Blk::set_num_workers, py::arg("n"))
        .def("set_kernel", &Blk::set_kernel, py::arg("name"))
        .def("windows", &Blk::windows)
        .def("input_items", &Blk::input_items)
        .def("output_items", &Blk::output_items)
        .def("tag_errors", &Blk::tag_errors)
        .def("map_input_offset_to_output",
             &Blk::map_input_offset_to_output,
             py::arg("p"));
}

void bind_pdu_rational_resampler_ccf_65_48(py::module& m)
{
    using Blk = gr::uwb::UwbPduRationalResamplerCcf65_48;

    py::enum_<Blk::EmitPolicy>(m, "pdu_resampler_emit_policy")
        .value("FullWindow", Blk::EmitPolicy::FullWindow)
        .value("CaptureOnly", Blk::EmitPolicy::CaptureOnly)
        .export_values();

    // Shared with 65/32 (see uwb_sc16_scale.h).  Registered once here because
    // this binder runs before bind_pdu_rational_resampler_ccf_65_32().
    py::enum_<Blk::Sc16ScalePolicy>(m, "Sc16ScalePolicy")
        .value("RawInteger", Blk::Sc16ScalePolicy::RawInteger)
        .value("UnitRange", Blk::Sc16ScalePolicy::UnitRange);

    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(
        m, "pdu_rational_resampler_ccf_65_48")
        .def(py::init(&Blk::make),
             py::arg("taps_file_or_profile") = std::string("quality_minorder"),
             py::arg("output_sample_rate") = Blk::kOutputRateHz,
             py::arg("validate_input_rate") = true,
             py::arg("emit_policy") = Blk::EmitPolicy::FullWindow,
             py::arg("max_input_samples") = Blk::kDefaultMaxInputSamples,
             py::arg("sc16_scale") = Blk::Sc16ScalePolicy::RawInteger)
        .def_static("make_from_taps",
                    &Blk::make_from_taps,
                    py::arg("taps"),
                    py::arg("output_sample_rate") = Blk::kOutputRateHz,
                    py::arg("validate_input_rate") = true,
                    py::arg("emit_policy") = Blk::EmitPolicy::FullWindow,
                    py::arg("max_input_samples") = Blk::kDefaultMaxInputSamples,
                    py::arg("sc16_scale") = Blk::Sc16ScalePolicy::RawInteger)
        .def("taps", &Blk::taps)
        .def("tap_count", &Blk::tap_count)
        .def("output_sample_rate", &Blk::output_sample_rate)
        .def("validate_input_rate", &Blk::validate_input_rate)
        .def("emit_policy", &Blk::emit_policy)
        .def("set_emit_policy", &Blk::set_emit_policy, py::arg("p"))
        .def("max_input_samples", &Blk::max_input_samples)
        .def("max_output_samples", &Blk::max_output_samples)
        .def("set_sc16_scale", &Blk::set_sc16_scale, py::arg("p"))
        .def("sc16_scale", &Blk::sc16_scale)
        .def("map_input_offset_to_output",
             &Blk::map_input_offset_to_output,
             py::arg("p"))
        .def("pdus_received", &Blk::pdus_received)
        .def("pdus_emitted", &Blk::pdus_emitted)
        .def("pdus_dropped", &Blk::pdus_dropped)
        .def("total_input_samples", &Blk::total_input_samples)
        .def("total_output_samples", &Blk::total_output_samples)
        .def("resets", &Blk::resets)
        .def("short_guard_events", &Blk::short_guard_events)
        .def("resample_total_us", &Blk::resample_total_us)
        .def("resample_max_us", &Blk::resample_max_us)
        .def("handler_total_us", &Blk::handler_total_us)
        .def("input_convert_total_us", &Blk::input_convert_total_us)
        .def("publish_total_us", &Blk::publish_total_us)
        .def("reset_stats", &Blk::reset_stats);
}

void bind_pdu_rational_resampler_ccf_65_32(py::module& m)
{
    using Blk = gr::uwb::UwbPduRationalResamplerCcf65_32;
    // Sc16ScalePolicy is registered once by the 65/48 binder (shared type).
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(
        m, "pdu_rational_resampler_ccf_65_32")
        .def(py::init([](const std::string& taps,
                         double output_sample_rate,
                         bool validate_input_rate,
                         size_t max_input_samples,
                         int num_workers,
                         Blk::Sc16ScalePolicy sc16_scale) {
                 return Blk::make(taps,
                                  output_sample_rate,
                                  validate_input_rate,
                                  Blk::EmitPolicy::FullWindow,
                                  max_input_samples,
                                  num_workers,
                                  sc16_scale);
             }),
             py::arg("taps_file_or_profile") = std::string("quality_minorder"),
             py::arg("output_sample_rate") = Blk::kOutputRateHz,
             py::arg("validate_input_rate") = true,
             py::arg("max_input_samples") = Blk::kDefaultMaxInputSamples,
             py::arg("num_workers") = 1,
             py::arg("sc16_scale") = Blk::Sc16ScalePolicy::RawInteger)
        .def_static(
            "make_from_taps",
            [](const std::vector<float>& taps,
               double output_sample_rate,
               bool validate_input_rate,
               int emit_policy,
               size_t max_input_samples,
               int num_workers,
               Blk::Sc16ScalePolicy sc16_scale) {
                return Blk::make_from_taps(
                    taps,
                    output_sample_rate,
                    validate_input_rate,
                    static_cast<Blk::EmitPolicy>(emit_policy),
                    max_input_samples,
                    num_workers,
                    sc16_scale);
            },
            py::arg("taps"),
            py::arg("output_sample_rate") = Blk::kOutputRateHz,
            py::arg("validate_input_rate") = true,
            py::arg("emit_policy") =
                static_cast<int>(Blk::EmitPolicy::FullWindow),
            py::arg("max_input_samples") = Blk::kDefaultMaxInputSamples,
            py::arg("num_workers") = 1,
            py::arg("sc16_scale") = Blk::Sc16ScalePolicy::RawInteger)
        .def("taps", &Blk::taps)
        .def("tap_count", &Blk::tap_count)
        .def("output_sample_rate", &Blk::output_sample_rate)
        .def("validate_input_rate", &Blk::validate_input_rate)
        .def("emit_policy",
             [](const Blk& self) {
                 return static_cast<int>(self.emit_policy());
             })
        .def(
            "set_emit_policy",
            [](Blk& self, int p) {
                self.set_emit_policy(static_cast<Blk::EmitPolicy>(p));
            },
            py::arg("p"))
        .def("max_input_samples", &Blk::max_input_samples)
        .def("max_output_samples", &Blk::max_output_samples)
        .def("map_input_offset_to_output",
             &Blk::map_input_offset_to_output,
             py::arg("p"))
        .def("pdus_received", &Blk::pdus_received)
        .def("pdus_emitted", &Blk::pdus_emitted)
        .def("pdus_dropped", &Blk::pdus_dropped)
        .def("total_input_samples", &Blk::total_input_samples)
        .def("total_output_samples", &Blk::total_output_samples)
        .def("resets", &Blk::resets)
        .def("short_guard_events", &Blk::short_guard_events)
        .def("resample_total_us", &Blk::resample_total_us)
        .def("resample_max_us", &Blk::resample_max_us)
        .def("handler_total_us", &Blk::handler_total_us)
        .def("input_convert_total_us", &Blk::input_convert_total_us)
        .def("publish_total_us", &Blk::publish_total_us)
        .def("set_num_workers", &Blk::set_num_workers, py::arg("n"))
        .def("num_workers", &Blk::num_workers)
        .def("resampler_kernel", &Blk::resampler_kernel)
        .def("set_sc16_scale", &Blk::set_sc16_scale, py::arg("p"))
        .def("sc16_scale", &Blk::sc16_scale)
        .def("reset_stats", &Blk::reset_stats);
}

void bind_pdu_window_crop(py::module& m)
{
    using Blk = gr::uwb::UwbPduWindowCrop;
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m, "pdu_window_crop")
        .def(py::init(&Blk::make),
             py::arg("pre_samples") =
                 gr::uwb::defaults::kNativeScheduledPreGuard,
             py::arg("capture_samples") =
                 gr::uwb::defaults::kNativeScheduledCapture,
             py::arg("post_samples") =
                 gr::uwb::defaults::kNativeScheduledPostGuard)
        .def("pre_samples", &Blk::pre_samples)
        .def("capture_samples", &Blk::capture_samples)
        .def("post_samples", &Blk::post_samples)
        .def("set_geometry",
             &Blk::set_geometry,
             py::arg("pre"),
             py::arg("capture"),
             py::arg("post"))
        .def("pdus_received", &Blk::pdus_received)
        .def("pdus_emitted", &Blk::pdus_emitted)
        .def("pdus_passthrough", &Blk::pdus_passthrough)
        .def("pdus_cropped", &Blk::pdus_cropped)
        .def("pdus_clamped", &Blk::pdus_clamped)
        .def("pdus_dropped", &Blk::pdus_dropped)
        .def("total_input_samples", &Blk::total_input_samples)
        .def("total_output_samples", &Blk::total_output_samples)
        .def("reset_stats", &Blk::reset_stats);
}

void bind_radar_packet_source(py::module& m)
{
    using Blk = gr::uwb::UwbRadarPacketSource;
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m, "radar_packet_source")
        .def(py::init(&Blk::make),
             py::arg("path"),
             py::arg("sample_rate"),
             py::arg("sample_format") = std::string("fc32"),
             py::arg("sync_repetitions") = size_t(64),
             py::arg("sfd_mode") = std::string("4z2"),
             py::arg("code_index") = size_t(9),
             py::arg("pri_s") = gr::uwb::defaults::kQm35PacketIntervalS,
             py::arg("auto_emit") = false,
             py::arg("descriptor_path") = std::string())
        .def("path", &Blk::path)
        .def("sample_rate", &Blk::sample_rate)
        .def("sample_format", &Blk::sample_format)
        .def("sync_repetitions", &Blk::sync_repetitions)
        .def("sfd_mode", &Blk::sfd_mode)
        .def("code_index", &Blk::code_index)
        .def("pri_s", &Blk::pri_s)
        .def("auto_emit", &Blk::auto_emit)
        .def("descriptor_path", &Blk::descriptor_path)
        .def("num_samples", &Blk::num_samples)
        .def("samples", &Blk::samples)
        .def("sc16_samples", &Blk::sc16_samples)
        .def("emits_received", &Blk::emits_received)
        .def("pdus_emitted", &Blk::pdus_emitted)
        .def("pdus_dropped", &Blk::pdus_dropped);
}

void bind_hrp_packet_source(py::module& m)
{
    using Blk = gr::uwb::UwbHrpPacketSource;
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m, "hrp_packet_source")
        .def(py::init(&Blk::make),
             py::arg("psdu"),
             py::arg("sync_repetitions") = size_t(64),
             py::arg("sfd_mode") = std::string("ieee"),
             py::arg("code_index") = size_t(9),
             py::arg("peak_amplitude") = gr::uwb::mod::kDefaultPeakAmplitude,
             py::arg("pri_s") = gr::uwb::defaults::kQm35PacketIntervalS,
             py::arg("auto_emit") = false,
             py::arg("insert_sts") = false,
             py::arg("append_fcs") = false,
             py::arg("pulse_shape") = std::string("legacy"),
             py::arg("pulse_sigma_ns") = 2.5f,
             py::arg("pulse_bw_mhz") = 200.0f,
             py::arg("pulse_taps_file") = std::string(""))
        .def("sync_repetitions", &Blk::sync_repetitions)
        .def("sfd_mode", &Blk::sfd_mode)
        .def("code_index", &Blk::code_index)
        .def("peak_amplitude", &Blk::peak_amplitude)
        .def("pri_s", &Blk::pri_s)
        .def("auto_emit", &Blk::auto_emit)
        .def("insert_sts", &Blk::insert_sts)
        .def("append_fcs", &Blk::append_fcs)
        .def("pulse_shape", &Blk::pulse_shape)
        .def("pulse_taps_file", &Blk::pulse_taps_file)
        .def("pulse_sigma_ns", &Blk::pulse_sigma_ns)
        .def("pulse_bw_mhz", &Blk::pulse_bw_mhz)
        .def("pulse_taps", &Blk::pulse_taps)
        .def("pulse_center_taps", &Blk::pulse_center_taps)
        .def("num_samples", &Blk::num_samples)
        .def("samples", &Blk::samples)
        .def("psdu", &Blk::psdu)
        .def("emits_received", &Blk::emits_received)
        .def("pdus_emitted", &Blk::pdus_emitted)
        .def("pdus_dropped", &Blk::pdus_dropped);
}

void bind_loopback_echo(py::module& m)
{
    using Blk = gr::uwb::UwbLoopbackEcho;
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m, "loopback_echo")
        .def(py::init(&Blk::make),
             py::arg("pre_guard_samples"),
             py::arg("tail_samples"),
             py::arg("delay_samples") = std::vector<double>(),
             py::arg("gains") = std::vector<std::complex<float>>(),
             py::arg("noise_std") = 0.0f,
             py::arg("rng_seed") = uint32_t(1),
             py::arg("max_tx_samples") = size_t(2097152),
             py::arg("max_rx_samples") = size_t(4194304),
             py::arg("num_delay_samps") = size_t(0),
             py::arg("t0_s") = 0.0,
             py::arg("pri_s") = gr::uwb::defaults::kQm35PacketIntervalS)
        .def("pre_guard_samples", &Blk::pre_guard_samples)
        .def("tail_samples", &Blk::tail_samples)
        .def("delay_samples", &Blk::delay_samples)
        .def("gains", &Blk::gains)
        .def("noise_std", &Blk::noise_std)
        .def("rng_seed", &Blk::rng_seed)
        .def("max_tx_samples", &Blk::max_tx_samples)
        .def("max_rx_samples", &Blk::max_rx_samples)
        .def("num_delay_samps", &Blk::num_delay_samps)
        .def("t0_s", &Blk::t0_s)
        .def("pri_s", &Blk::pri_s)
        .def("pdus_received", &Blk::pdus_received)
        .def("pdus_emitted", &Blk::pdus_emitted)
        .def("pdus_dropped", &Blk::pdus_dropped);
}

void bind_radar_cir_estimator(py::module& m)
{
    using Blk = gr::uwb::UwbRadarCirEstimator;
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m,
                                                     "radar_cir_estimator")
        .def(py::init(&Blk::make),
             py::arg("template_path"),
             py::arg("sync_repetitions") = size_t(64),
             py::arg("sfd_mode") = std::string("4z2"),
             py::arg("code_index") = size_t(9),
             py::arg("cir_pre") = size_t(16),
             py::arg("cir_post") = size_t(100),
             py::arg("cir_skip_initial") = size_t(10),
             py::arg("cir_repetitions") = size_t(0),
             py::arg("sfd_search_margin") = int64_t(64),
             py::arg("sync_refine_margin") = int64_t(8),
             py::arg("sfd_threshold") = 0.3f,
             py::arg("sync_refine_threshold") = 0.3f,
             py::arg("emit_normalized") = true,
             py::arg("queue_capacity") = size_t(64),
             py::arg("use_predicted_timing") = false,
             py::arg("emit_individual_repetitions") = false)
        .def("template_path", &Blk::template_path)
        .def("sync_repetitions", &Blk::sync_repetitions)
        .def("sfd_mode", &Blk::sfd_mode)
        .def("code_index", &Blk::code_index)
        .def("cir_pre", &Blk::cir_pre)
        .def("cir_post", &Blk::cir_post)
        .def("cir_skip_initial", &Blk::cir_skip_initial)
        .def("cir_repetitions", &Blk::cir_repetitions)
        .def("sfd_search_margin", &Blk::sfd_search_margin)
        .def("sync_refine_margin", &Blk::sync_refine_margin)
        .def("emit_normalized", &Blk::emit_normalized)
        .def("queue_capacity", &Blk::queue_capacity)
        .def("use_predicted_timing", &Blk::use_predicted_timing)
        .def("emit_individual_repetitions",
             &Blk::emit_individual_repetitions)
        .def("pdus_received", &Blk::pdus_received)
        .def("pdus_enqueued", &Blk::pdus_enqueued)
        .def("pdus_completed", &Blk::pdus_completed)
        .def("pdus_failed", &Blk::pdus_failed)
        .def("pdus_published", &Blk::pdus_published)
        .def("pdus_dropped", &Blk::pdus_dropped)
        .def("invalid_inputs", &Blk::invalid_inputs)
        .def("worker_exceptions", &Blk::worker_exceptions)
        .def("queue_depth", &Blk::queue_depth)
        .def("queue_high_watermark", &Blk::queue_high_watermark)
        .def("service_mean_us", &Blk::service_mean_us)
        .def("service_p95_us", &Blk::service_p95_us)
        .def("service_p99_us", &Blk::service_p99_us)
        .def("service_max_us", &Blk::service_max_us)
        .def("drained", &Blk::drained)
        .def("drain", &Blk::drain)
        .def("reset_stats", &Blk::reset_stats);
}

void bind_cir_writer(py::module& m)
{
    using Blk = gr::uwb::UwbCirWriter;
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m, "cir_writer")
        .def(py::init(&Blk::make),
             py::arg("directory"),
             py::arg("base_name") = std::string("cir"),
             py::arg("write_normalized") = false,
             py::arg("queue_capacity") = size_t(64))
        .def("directory", &Blk::directory)
        .def("base_name", &Blk::base_name)
        .def("write_normalized", &Blk::write_normalized)
        .def("queue_capacity", &Blk::queue_capacity)
        .def("frames_received", &Blk::frames_received)
        .def("frames_written", &Blk::frames_written)
        .def("frames_failed", &Blk::frames_failed)
        .def("frames_dropped", &Blk::frames_dropped)
        .def("frames_invalid", &Blk::frames_invalid)
        .def("taps_written", &Blk::taps_written)
        .def("queue_high_watermark", &Blk::queue_high_watermark);
}

void bind_echo_backends(py::module& m)
{
    using Backend = gr::uwb::echo::IRadioBurstBackend;
    using Fake = gr::uwb::echo::FakeBurstBackend;

    // Abstract backend (allows Python to pass a concrete backend into the
    // generic factories below).
    py::class_<Backend, std::shared_ptr<Backend>>(m, "echo_burst_backend");

    py::class_<Fake, Backend, std::shared_ptr<Fake>>(m, "fake_burst_backend")
        .def(py::init([](uint64_t max_io_chunk,
                         int64_t device_time_start,
                         int64_t device_time_auto_advance) {
                 Fake::Config cfg;
                 cfg.max_io_chunk = max_io_chunk;
                 cfg.device_time_start = device_time_start;
                 cfg.device_time_auto_advance = device_time_auto_advance;
                 return std::make_shared<Fake>(cfg);
             }),
             py::arg("max_io_chunk") = 0,
             py::arg("device_time_start") = 0,
             py::arg("device_time_auto_advance") = 0)
        .def("center_freq_hz", &Fake::center_freq_hz)
        .def("burst_count", &Fake::burst_count)
        .def("set_device_time", &Fake::set_device_time, py::arg("ticks"));
}

void bind_echo_timer_stream(py::module& m)
{
    using Backend = gr::uwb::echo::IRadioBurstBackend;
    using Blk = gr::uwb::UwbEchoTimerStream;

    // Generic factory: any injected IRadioBurstBackend (Python can pass a
    // fake_burst_backend).  The scheduler config is given as scalars.
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m, "echo_timer_stream")
        .def(py::init([](int64_t pri_num,
                         int64_t pri_den,
                         int64_t pre_guard_ticks,
                         uint64_t max_fragment_size,
                         uint64_t max_catchup_slots,
                         std::shared_ptr<Backend> backend,
                         uint64_t tx_samples,
                         uint64_t rx_samples,
                         double sample_rate_hz,
                         uint64_t pre_guard_samples,
                         uint64_t capture_samples,
                         uint64_t post_guard_samples,
                         size_t sync_repetitions,
                         const std::string& sfd_mode,
                         size_t code_index,
                         double calibration_delay_native_samples,
                         int64_t t0_ticks,
                         double arm_margin_s,
                         uint64_t max_frames,
                         uint64_t collect_wait_ms,
                         size_t max_tx_samples,
                         size_t max_rx_samples,
                         const std::string& lengthtagname) {
                 gr::uwb::echo::EchoSchedulerConfig cfg;
                 cfg.pri_num = pri_num;
                 cfg.pri_den = pri_den;
                 cfg.pre_guard_ticks = pre_guard_ticks;
                 cfg.max_fragment_size = max_fragment_size;
                 cfg.max_catchup_slots = max_catchup_slots;
                 return Blk::make(cfg,
                                  std::move(backend),
                                  tx_samples,
                                  rx_samples,
                                  sample_rate_hz,
                                  pre_guard_samples,
                                  capture_samples,
                                  post_guard_samples,
                                  sync_repetitions,
                                  sfd_mode,
                                  code_index,
                                  calibration_delay_native_samples,
                                  t0_ticks,
                                  arm_margin_s,
                                  max_frames,
                                  collect_wait_ms,
                                  max_tx_samples,
                                  max_rx_samples,
                                  lengthtagname);
             }),
             py::arg("pri_num") = int64_t(3686400),
             py::arg("pri_den") = int64_t(1),
             py::arg("pre_guard_ticks") = int64_t(1475),
             py::arg("max_fragment_size") = uint64_t(65536),
             py::arg("max_catchup_slots") = uint64_t(1) << 20,
             py::arg("backend"),
             py::arg("tx_samples"),
             py::arg("rx_samples"),
             py::arg("sample_rate_hz") = 737280000.0,
             py::arg("pre_guard_samples") = uint64_t(0),
             py::arg("capture_samples") = uint64_t(0),
             py::arg("post_guard_samples") = uint64_t(0),
             py::arg("sync_repetitions") = size_t(64),
             py::arg("sfd_mode") = std::string("4z2"),
             py::arg("code_index") = size_t(9),
             py::arg("calibration_delay_native_samples") = 0.0,
             py::arg("t0_ticks") = int64_t(0),
             py::arg("arm_margin_s") = 0.05,
             py::arg("max_frames") = uint64_t(0),
             py::arg("collect_wait_ms") = uint64_t(1000),
             py::arg("max_tx_samples") = size_t(1) << 21,
             py::arg("max_rx_samples") = size_t(1) << 21,
             py::arg("lengthtagname") = std::string("packet_len"))
        .def("set_freq", &Blk::set_freq, py::arg("hz"))
        .def("freq", &Blk::freq)
        .def("set_cal_delay_native", &Blk::set_cal_delay_native, py::arg("v"))
        .def("cal_delay_native", &Blk::cal_delay_native)
        .def("bursts_ok", &Blk::bursts_ok)
        .def("bursts_failed", &Blk::bursts_failed)
        .def("late_slot_skips", &Blk::late_slot_skips)
        .def("grid_errors", &Blk::grid_errors)
        .def("rx_samples", &Blk::rx_samples)
        .def("tx_samples", &Blk::tx_samples)
        .def("frames", &Blk::frames)
        .def("last_error", &Blk::last_error);

#ifdef UWB_HAVE_UHD
    // UHD convenience factory: builds the UhdBurstBackend internally from
    // scalar args.  Only available when the build has UHD.
    m.def(
        "echo_timer_stream_uhd",
        [](const std::string& device_args,
           double sample_rate_hz,
           size_t tx_channel,
           size_t rx_channel,
           const std::string& tx_antenna,
           const std::string& rx_antenna,
           double tx_gain_db,
           double rx_gain_db,
           double center_freq_hz,
           const std::string& clock_source,
           const std::string& time_source,
           int64_t pri_num,
           int64_t pri_den,
           int64_t pre_guard_ticks,
           int64_t t0_ticks,
           double arm_margin_s,
           uint64_t tx_samples,
           uint64_t rx_samples,
           uint64_t pre_guard_samples,
           uint64_t capture_samples,
           uint64_t post_guard_samples,
           size_t sync_repetitions,
           const std::string& sfd_mode,
           size_t code_index,
           double calibration_delay_native_samples,
           uint64_t max_frames,
           uint64_t collect_wait_ms,
           size_t max_tx_samples,
           size_t max_rx_samples,
           const std::string& lengthtagname) {
            gr::uwb::uhd::UhdBurstBackendConfig ucfg;
            ucfg.device_args = device_args;
            ucfg.sample_rate_hz = sample_rate_hz;
            ucfg.tx_channel = tx_channel;
            ucfg.rx_channel = rx_channel;
            ucfg.tx_antenna = tx_antenna;
            ucfg.rx_antenna = rx_antenna;
            ucfg.tx_gain_db = tx_gain_db;
            ucfg.rx_gain_db = rx_gain_db;
            ucfg.center_freq_hz = center_freq_hz;
            ucfg.clock_source = clock_source;
            ucfg.time_source = time_source;
            gr::uwb::echo::EchoSchedulerConfig scfg;
            scfg.pri_num = pri_num;
            scfg.pri_den = pri_den;
            scfg.pre_guard_ticks = pre_guard_ticks;
            return Blk::make_uhd(ucfg,
                                 scfg,
                                 tx_samples,
                                 rx_samples,
                                 sample_rate_hz,
                                 pre_guard_samples,
                                 capture_samples,
                                 post_guard_samples,
                                 sync_repetitions,
                                 sfd_mode,
                                 code_index,
                                 calibration_delay_native_samples,
                                 t0_ticks,
                                 arm_margin_s,
                                 max_frames,
                                 collect_wait_ms,
                                 max_tx_samples,
                                 max_rx_samples,
                                 lengthtagname);
        },
        py::arg("device_args") = std::string(),
        py::arg("sample_rate_hz") = 737280000.0,
        py::arg("tx_channel") = size_t(0),
        py::arg("rx_channel") = size_t(1),
        py::arg("tx_antenna") = std::string(),
        py::arg("rx_antenna") = std::string(),
        py::arg("tx_gain_db") = -1.0,
        py::arg("rx_gain_db") = -1.0,
        py::arg("center_freq_hz") = 0.0,
        py::arg("clock_source") = std::string("internal"),
        py::arg("time_source") = std::string("internal"),
        py::arg("pri_num") = int64_t(3686400),
        py::arg("pri_den") = int64_t(1),
        py::arg("pre_guard_ticks") = int64_t(1475),
        py::arg("t0_ticks") = int64_t(0),
        py::arg("arm_margin_s") = 0.05,
        py::arg("tx_samples"),
        py::arg("rx_samples"),
        py::arg("pre_guard_samples") = uint64_t(0),
        py::arg("capture_samples") = uint64_t(0),
        py::arg("post_guard_samples") = uint64_t(0),
        py::arg("sync_repetitions") = size_t(64),
        py::arg("sfd_mode") = std::string("4z2"),
        py::arg("code_index") = size_t(9),
        py::arg("calibration_delay_native_samples") = 0.0,
        py::arg("max_frames") = uint64_t(0),
        py::arg("collect_wait_ms") = uint64_t(1000),
        py::arg("max_tx_samples") = size_t(1) << 21,
        py::arg("max_rx_samples") = size_t(1) << 21,
        py::arg("lengthtagname") = std::string("packet_len"));
#endif
}

#ifdef UWB_HAVE_UHD
namespace {
// Shared UHD config builder (X410 dual-TX M2, §6.3): parallel per-channel
// arrays; an empty gains/antennas/freqs list falls back per channel to the
// legacy scalars (gain -1 = leave, antenna "" = leave, freq =
// center_freq_hz).  Non-empty lists must match tx_channels in length.
// Throws std::invalid_argument on violations (same surface as the backend
// ctor's own config validation).
gr::uwb::uhd::UhdBurstBackendConfig build_multitx_uhd_config(
    const std::string& device_args,
    double sample_rate_hz,
    const std::vector<size_t>& tx_channels,
    size_t rx_channel,
    const std::vector<std::string>& tx_antennas,
    const std::string& rx_antenna,
    const std::vector<double>& tx_gains_db,
    double rx_gain_db,
    const std::vector<double>& tx_freqs_hz,
    double center_freq_hz,
    double rx_freq_offset_hz,
    const std::string& clock_source,
    const std::string& time_source)
{
    using namespace gr::uwb;
    if (tx_channels.empty() ||
        tx_channels.size() > echo::kEchoMaxTxChannels)
        throw std::invalid_argument("tx_channels needs 1..4 entries");
    const size_t n = tx_channels.size();
    auto need_len = [&](const char* what, size_t m) {
        if (m != 0 && m != n)
            throw std::invalid_argument(
                std::string(what) + " must be empty or match tx_channels");
    };
    need_len("tx_antennas", tx_antennas.size());
    need_len("tx_gains", tx_gains_db.size());
    need_len("tx_freqs", tx_freqs_hz.size());
    for (size_t i = 0; i < n; ++i)
        for (size_t j = i + 1; j < n; ++j)
            if (tx_channels[i] == tx_channels[j])
                throw std::invalid_argument("duplicate TX channel");
    for (double g : tx_gains_db) {
        if (!std::isfinite(g))
            throw std::invalid_argument("tx_gains must be finite");
        if (g >= 0.0 && g > 120.0)
            throw std::invalid_argument("tx_gains must be <= 120 when set");
    }
    for (double f : tx_freqs_hz) {
        if (!std::isfinite(f) || f < 0.0)
            throw std::invalid_argument("tx_freqs must be >= 0 and finite");
    }
    const std::vector<std::string> ants =
        tx_antennas.empty() ? std::vector<std::string>(n) : tx_antennas;
    const std::vector<double> gains =
        tx_gains_db.empty() ? std::vector<double>(n, -1.0) : tx_gains_db;
    const std::vector<double> freqs =
        tx_freqs_hz.empty() ? std::vector<double>(n, center_freq_hz)
                            : tx_freqs_hz;
    uhd::UhdBurstBackendConfig ucfg;
    ucfg.device_args = device_args;
    ucfg.sample_rate_hz = sample_rate_hz;
    ucfg.rx_channel = rx_channel;
    ucfg.rx_antenna = rx_antenna;
    ucfg.rx_gain_db = rx_gain_db;
    ucfg.rx_freq_offset_hz = rx_freq_offset_hz;
    ucfg.clock_source = clock_source;
    ucfg.time_source = time_source;
    // Legacy scalars from logical-0 (the single source of truth pre-M1).
    ucfg.tx_channel = tx_channels[0];
    ucfg.tx_antenna = ants[0];
    ucfg.tx_gain_db = gains[0];
    ucfg.center_freq_hz = freqs[0];
    // M1 vector when the member exists (no-op otherwise).
    uhd::assign_tx_channels(ucfg, tx_channels, ants, gains, freqs);
    return ucfg;
}
} // namespace
#endif

void bind_realtime_echo_timer(py::module& m)
{
    using Backend = gr::uwb::echo::IRadioBurstBackend;
    using Blk = gr::uwb::UwbRealtimeEchoTimer;

    // Generic factory: any injected IRadioBurstBackend (Python can pass a
    // fake_burst_backend).  The scheduler config is given as scalars.
    py::class_<Blk, gr::block, std::shared_ptr<Blk>>(m, "realtime_echo_timer")
        .def(py::init([](int64_t pri_num,
                         int64_t pri_den,
                         int64_t pre_guard_ticks,
                         uint64_t max_fragment_size,
                         uint64_t max_catchup_slots,
                         std::shared_ptr<Backend> backend,
                         size_t queue_capacity,
                         uint64_t rx_collect_wait_ms,
                         size_t max_tx_samples,
                         size_t max_rx_samples) {
                 gr::uwb::echo::EchoSchedulerConfig cfg;
                 cfg.pri_num = pri_num;
                 cfg.pri_den = pri_den;
                 cfg.pre_guard_ticks = pre_guard_ticks;
                 cfg.max_fragment_size = max_fragment_size;
                 cfg.max_catchup_slots = max_catchup_slots;
                 return Blk::make(cfg,
                                  std::move(backend),
                                  queue_capacity,
                                  rx_collect_wait_ms,
                                  max_tx_samples,
                                  max_rx_samples);
             }),
             py::arg("pri_num") = int64_t(3686400),
             py::arg("pri_den") = int64_t(1),
             py::arg("pre_guard_ticks") = int64_t(1475),
             py::arg("max_fragment_size") = uint64_t(65536),
             py::arg("max_catchup_slots") = uint64_t(1u << 20),
             py::arg("backend"),
             py::arg("queue_capacity") = size_t(64),
             py::arg("rx_collect_wait_ms") = uint64_t(1000),
             py::arg("max_tx_samples") = size_t(1u << 21),
             py::arg("max_rx_samples") = size_t(1u << 21))
        .def("queue_capacity", &Blk::queue_capacity)
        .def("queue_depth", &Blk::queue_depth)
        .def("queue_high_watermark", &Blk::queue_high_watermark)
        .def("schedules_received", &Blk::schedules_received)
        .def("schedules_enqueued", &Blk::schedules_enqueued)
        .def("schedules_dropped", &Blk::schedules_dropped)
        .def("schedules_invalid", &Blk::schedules_invalid)
        .def("bursts_published", &Blk::bursts_published)
        .def("bursts_ok", &Blk::bursts_ok)
        .def("bursts_failed", &Blk::bursts_failed)
        .def("late_slot_skips", &Blk::late_slot_skips)
        .def("tx_reissues", &Blk::tx_reissues)
        .def("rx_reissues", &Blk::rx_reissues)
        .def("grid_errors", &Blk::grid_errors)
        .def("drain", &Blk::drain)
        .def("drained", &Blk::drained)
        .def("set_freq", &Blk::set_freq, py::arg("hz"))
        .def("freq", &Blk::freq)
        .def("set_cal_delay_native",
             &Blk::set_cal_delay_native,
             py::arg("native_samples"))
        .def("cal_delay_native", &Blk::cal_delay_native)
        .def("set_publish_native", &Blk::set_publish_native, py::arg("n"))
        .def("publish_native", &Blk::publish_native)
        .def("published_samples_total", &Blk::published_samples_total)
        .def("device_time_ticks", &Blk::device_time_ticks)
        .def("last_worker_us", &Blk::last_worker_us)
        .def("max_worker_us", &Blk::max_worker_us)
        .def("mean_worker_us", &Blk::mean_worker_us)
        .def("slot_lead_us_last", &Blk::slot_lead_us_last)
        .def("issue_rx_us_mean", &Blk::issue_rx_us_mean)
        .def("tx_send_us_mean", &Blk::tx_send_us_mean)
        .def("rx_collect_us_mean", &Blk::rx_collect_us_mean)
        .def("pdu_build_us_mean", &Blk::pdu_build_us_mean)
        .def("pdu_publish_us_mean", &Blk::pdu_publish_us_mean)
        .def("worker_total_us_mean", &Blk::worker_total_us_mean)
        .def("last_error", &Blk::last_error)
        // Multi-TX introspection (M3/M4/M5; missing before M7 bindings).
        .def("tx_channel_count", &Blk::tx_channel_count)
        .def("jam_logical_channel", &Blk::jam_logical_channel)
        .def("jam_delay_native_last", &Blk::jam_delay_native_last)
        .def("jam_delay_us_last", &Blk::jam_delay_us_last)
        .def("jam_freq_plan_hz_last", &Blk::jam_freq_plan_hz_last)
        .def("jam_freq_actual_hz_last", &Blk::jam_freq_actual_hz_last)
        .def("jam_freq_offset_hz_last", &Blk::jam_freq_offset_hz_last)
        .def("jam_retunes", &Blk::jam_retunes)
        .def("jam_retune_failures", &Blk::jam_retune_failures)
        .def("tx_async_counts", [](const Blk& self) {
            py::dict out;
            pmt::pmt_t d = self.tx_async_counts();
            static const char* const keys[] = {
                "underflow", "seq_error", "time_error",
                "unmatched", "dropped", "ack"
            };
            for (const char* k : keys) {
                uint64_t v = 0;
                pmt::pmt_t pv =
                    pmt::dict_ref(d, pmt::mp(k), pmt::PMT_NIL);
                if (pmt::is_uint64(pv))
                    v = pmt::to_uint64(pv);
                else if (pmt::is_integer(pv))
                    v = static_cast<uint64_t>(pmt::to_long(pv));
                out[k] = v;
            }
            return out;
        });

#ifdef UWB_HAVE_UHD
    // UHD convenience factory: builds the UhdBurstBackend internally from
    // scalar args.  Only available when the build has UHD.  Kept as a
    // length-1 compat wrapper over the multi-TX builder below: identical
    // defaults produce an identical single-channel config.
    m.def(
        "realtime_echo_timer_uhd",
        [](const std::string& device_args,
           double sample_rate_hz,
           size_t tx_channel,
           size_t rx_channel,
           const std::string& tx_antenna,
           const std::string& rx_antenna,
           double tx_gain_db,
           double rx_gain_db,
           double center_freq_hz,
           const std::string& clock_source,
           const std::string& time_source,
           int64_t pri_num,
           int64_t pri_den,
           int64_t pre_guard_ticks,
           uint64_t max_fragment_size,
           uint64_t max_catchup_slots,
           size_t queue_capacity,
           uint64_t collect_wait_ms,
           size_t max_tx_samples,
           size_t max_rx_samples,
           double rx_freq_offset_hz) {
            gr::uwb::uhd::UhdBurstBackendConfig ucfg =
                build_multitx_uhd_config(device_args,
                                         sample_rate_hz,
                                         { tx_channel },
                                         rx_channel,
                                         { tx_antenna },
                                         rx_antenna,
                                         { tx_gain_db },
                                         rx_gain_db,
                                         { center_freq_hz },
                                         center_freq_hz,
                                         rx_freq_offset_hz,
                                         clock_source,
                                         time_source);
            gr::uwb::echo::EchoSchedulerConfig scfg;
            scfg.pri_num = pri_num;
            scfg.pri_den = pri_den;
            scfg.pre_guard_ticks = pre_guard_ticks;
            scfg.max_fragment_size = max_fragment_size;
            scfg.max_catchup_slots = max_catchup_slots;
            return Blk::make_uhd(
                ucfg, scfg, queue_capacity, collect_wait_ms, max_tx_samples, max_rx_samples);
        },
        py::arg("device_args") = std::string(),
        py::arg("sample_rate_hz") = 737280000.0,
        py::arg("tx_channel") = size_t(0),
        py::arg("rx_channel") = size_t(1),
        py::arg("tx_antenna") = std::string(),
        py::arg("rx_antenna") = std::string(),
        py::arg("tx_gain_db") = -1.0,
        py::arg("rx_gain_db") = -1.0,
        py::arg("center_freq_hz") = 0.0,
        py::arg("clock_source") = std::string("internal"),
        py::arg("time_source") = std::string("internal"),
        py::arg("pri_num") = int64_t(3686400),
        py::arg("pri_den") = int64_t(1),
        py::arg("pre_guard_ticks") = int64_t(1475),
        py::arg("max_fragment_size") = uint64_t(65536),
        py::arg("max_catchup_slots") = uint64_t(1u << 20),
        py::arg("queue_capacity") = size_t(64),
        py::arg("collect_wait_ms") = uint64_t(1000),
        py::arg("max_tx_samples") = size_t(1u << 21),
        py::arg("max_rx_samples") = size_t(1u << 21),
        py::arg("rx_freq_offset_hz") = 0.0);

    // Multi-TX factory (X410 dual-TX M2, §6.3): per-channel TX arrays.
    // Empty tx_antennas/tx_gains/tx_freqs fall back per channel to ""
    // / -1.0 / center_freq_hz; non-empty lists must match tx_channels in
    // length.  Overload resolution: a Python list/tuple 3rd positional
    // (or tx_channels= keyword) selects this factory; a scalar int keeps
    // the compat wrapper above.
    m.def(
        "realtime_echo_timer_uhd",
        [](const std::string& device_args,
           double sample_rate_hz,
           const std::vector<size_t>& tx_channels,
           size_t rx_channel,
           const std::vector<std::string>& tx_antennas,
           const std::string& rx_antenna,
           const std::vector<double>& tx_gains,
           double rx_gain_db,
           const std::vector<double>& tx_freqs,
           double center_freq_hz,
           const std::string& clock_source,
           const std::string& time_source,
           int64_t pri_num,
           int64_t pri_den,
           int64_t pre_guard_ticks,
           uint64_t max_fragment_size,
           uint64_t max_catchup_slots,
           size_t queue_capacity,
           uint64_t collect_wait_ms,
           size_t max_tx_samples,
           size_t max_rx_samples,
           double rx_freq_offset_hz) {
            gr::uwb::uhd::UhdBurstBackendConfig ucfg =
                build_multitx_uhd_config(device_args,
                                         sample_rate_hz,
                                         tx_channels,
                                         rx_channel,
                                         tx_antennas,
                                         rx_antenna,
                                         tx_gains,
                                         rx_gain_db,
                                         tx_freqs,
                                         center_freq_hz,
                                         rx_freq_offset_hz,
                                         clock_source,
                                         time_source);
            gr::uwb::echo::EchoSchedulerConfig scfg;
            scfg.pri_num = pri_num;
            scfg.pri_den = pri_den;
            scfg.pre_guard_ticks = pre_guard_ticks;
            scfg.max_fragment_size = max_fragment_size;
            scfg.max_catchup_slots = max_catchup_slots;
            return Blk::make_uhd(
                ucfg, scfg, queue_capacity, collect_wait_ms, max_tx_samples, max_rx_samples);
        },
        py::arg("device_args") = std::string(),
        py::arg("sample_rate_hz") = 737280000.0,
        py::arg("tx_channels") = std::vector<size_t>{ 0 },
        py::arg("rx_channel") = size_t(1),
        py::arg("tx_antennas") = std::vector<std::string>(),
        py::arg("rx_antenna") = std::string(),
        py::arg("tx_gains") = std::vector<double>(),
        py::arg("rx_gain_db") = -1.0,
        py::arg("tx_freqs") = std::vector<double>(),
        py::arg("center_freq_hz") = 0.0,
        py::arg("clock_source") = std::string("internal"),
        py::arg("time_source") = std::string("internal"),
        py::arg("pri_num") = int64_t(3686400),
        py::arg("pri_den") = int64_t(1),
        py::arg("pre_guard_ticks") = int64_t(1475),
        py::arg("max_fragment_size") = uint64_t(65536),
        py::arg("max_catchup_slots") = uint64_t(1u << 20),
        py::arg("queue_capacity") = size_t(64),
        py::arg("collect_wait_ms") = uint64_t(1000),
        py::arg("max_tx_samples") = size_t(1u << 21),
        py::arg("max_rx_samples") = size_t(1u << 21),
        py::arg("rx_freq_offset_hz") = 0.0);

    // Alias factory with the jammer-app kwargs (§6.3): the app probes
    // realtime_echo_timer_uhd_multitx and passes tx_gains_db/tx_freqs_hz.
    // Delegates to the same multi-TX builder as the overload above.
    m.def(
        "realtime_echo_timer_uhd_multitx",
        [](const std::string& device_args,
           double sample_rate_hz,
           const std::vector<size_t>& tx_channels,
           size_t rx_channel,
           const std::vector<std::string>& tx_antennas,
           const std::string& rx_antenna,
           const std::vector<double>& tx_gains_db,
           double rx_gain_db,
           const std::vector<double>& tx_freqs_hz,
           const std::string& clock_source,
           const std::string& time_source,
           int64_t pri_num,
           int64_t pri_den,
           int64_t pre_guard_ticks,
           uint64_t max_fragment_size,
           uint64_t max_catchup_slots,
           size_t queue_capacity,
           uint64_t collect_wait_ms,
           size_t max_tx_samples,
           size_t max_rx_samples,
           double rx_freq_offset_hz) {
            gr::uwb::uhd::UhdBurstBackendConfig ucfg =
                build_multitx_uhd_config(device_args,
                                         sample_rate_hz,
                                         tx_channels,
                                         rx_channel,
                                         tx_antennas,
                                         rx_antenna,
                                         tx_gains_db,
                                         rx_gain_db,
                                         tx_freqs_hz,
                                         0.0,
                                         rx_freq_offset_hz,
                                         clock_source,
                                         time_source);
            gr::uwb::echo::EchoSchedulerConfig scfg;
            scfg.pri_num = pri_num;
            scfg.pri_den = pri_den;
            scfg.pre_guard_ticks = pre_guard_ticks;
            scfg.max_fragment_size = max_fragment_size;
            scfg.max_catchup_slots = max_catchup_slots;
            return Blk::make_uhd(
                ucfg, scfg, queue_capacity, collect_wait_ms, max_tx_samples, max_rx_samples);
        },
        py::arg("device_args") = std::string(),
        py::arg("sample_rate_hz") = 737280000.0,
        py::arg("tx_channels") = std::vector<size_t>{ 0 },
        py::arg("rx_channel") = size_t(1),
        py::arg("tx_antennas") = std::vector<std::string>(),
        py::arg("rx_antenna") = std::string(),
        py::arg("tx_gains_db") = std::vector<double>(),
        py::arg("rx_gain_db") = -1.0,
        py::arg("tx_freqs_hz") = std::vector<double>(),
        py::arg("clock_source") = std::string("internal"),
        py::arg("time_source") = std::string("internal"),
        py::arg("pri_num") = int64_t(3686400),
        py::arg("pri_den") = int64_t(1),
        py::arg("pre_guard_ticks") = int64_t(1475),
        py::arg("max_fragment_size") = uint64_t(65536),
        py::arg("max_catchup_slots") = uint64_t(1u << 20),
        py::arg("queue_capacity") = size_t(64),
        py::arg("collect_wait_ms") = uint64_t(1000),
        py::arg("max_tx_samples") = size_t(1u << 21),
        py::arg("max_rx_samples") = size_t(1u << 21),
        py::arg("rx_freq_offset_hz") = 0.0);
#endif
}

// We need this hack because import_array() returns NULL
// for newer Python versions.
// This function is also necessary because it ensures access to the C API
// and removes a warning.
void* init_numpy()
{
    import_array();
    return NULL;
}

PYBIND11_MODULE(uwb_python, m)
{
    // Initialize the numpy C API
    // (otherwise we will see segmentation faults)
    init_numpy();

    // Allow access to base block methods
    py::module::import("gnuradio.gr");

    bind_energy_detector(m);
    bind_preamble_detector(m);
    bind_detector(m);
    bind_detector_sc16(m);
    bind_packet_writer(m);
    bind_scheduled_extractor(m);
    bind_auto_scheduled_extractor_sc16(m);
    bind_realtime_demodulator(m);
    bind_rational_resampler_ccf_65_48(m);
    bind_rational_resampler_ccf_65_32(m);
    bind_pdu_rational_resampler_ccf_65_48(m);
    bind_pdu_rational_resampler_ccf_65_32(m);
    bind_pdu_window_crop(m);
    bind_radar_packet_source(m);
    bind_hrp_packet_source(m);
    bind_loopback_echo(m);
    bind_radar_cir_estimator(m);
    bind_cir_writer(m);
    bind_echo_backends(m);
    bind_echo_timer_stream(m);
    bind_realtime_echo_timer(m);

    m.def(
        "make_pulse_taps",
        [](const std::string& shape, float sigma_ns, float bw_mhz) {
            gr::uwb::mod::PulseSpec spec;
            if (!gr::uwb::mod::parse_pulse_shape(shape, spec.shape))
                throw std::invalid_argument(
                    "pulse_shape must be legacy|gaussian|blackman");
            spec.gaussian_sigma_ns = sigma_ns;
            spec.blackman_bw_mhz = bw_mhz;
            return gr::uwb::mod::make_pulse_taps(spec);
        },
        py::arg("pulse_shape") = std::string("gaussian"),
        py::arg("pulse_sigma_ns") = 2.5f,
        py::arg("pulse_bw_mhz") = 200.0f);
    m.def("pulse_center_tap",
          [](const std::string& shape) {
              gr::uwb::mod::PulseSpec spec;
              if (!gr::uwb::mod::parse_pulse_shape(shape, spec.shape))
                  throw std::invalid_argument(
                      "pulse_shape must be legacy|gaussian|blackman");
              return gr::uwb::mod::pulse_center_tap(spec);
          },
          py::arg("pulse_shape"));
}
