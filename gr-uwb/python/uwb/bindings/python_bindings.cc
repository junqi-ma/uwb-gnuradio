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

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_energy_detector.h>
#include <gnuradio/uwb/uwb_preamble_detector.h>
#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_detector_sc16.h>
#include <gnuradio/uwb/uwb_packet_writer.h>
#include <gnuradio/uwb/uwb_realtime_demodulator.h>
#include <gnuradio/uwb/uwb_scheduled_extractor.h>

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
        .def("scheduled_windows",
             &gr::uwb::UwbScheduledExtractor::scheduled_windows)
        .def("completed_windows",
             &gr::uwb::UwbScheduledExtractor::completed_windows)
        .def("emitted_windows", &gr::uwb::UwbScheduledExtractor::emitted_windows)
        .def("dropped_windows", &gr::uwb::UwbScheduledExtractor::dropped_windows)
        .def("queue_high_watermark",
             &gr::uwb::UwbScheduledExtractor::queue_high_watermark);
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
             py::arg("cir_filter_mode") = std::string("auto"))
        .def_static("make_from_template",
                    &gr::uwb::UwbRealtimeDemodulator::make_from_template,
                    py::arg("template_wf"),
                    py::arg("num_workers") = size_t(2),
                    py::arg("queue_capacity") = size_t(64),
                    py::arg("sfd_mode") = std::string("4z2"),
                    py::arg("cir_rake_top_k") = size_t(0),
                    py::arg("cir_filter_mode") = std::string("auto"))
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
    bind_realtime_demodulator(m);
}
