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

#include <gnuradio/uwb/uwb_energy_detector.h>
#include <gnuradio/uwb/uwb_preamble_detector.h>
#include <gnuradio/uwb/uwb_detector.h>
#include <gnuradio/uwb/uwb_packet_writer.h>

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
    py::class_<gr::uwb::UwbDetector,
               gr::sync_block,
               std::shared_ptr<gr::uwb::UwbDetector>>(m, "detector")
        .def(py::init(&gr::uwb::UwbDetector::make),
             py::arg("known_preamble"),
             py::arg("pre_trigger") = size_t(2032),
             py::arg("capture") = size_t(200000),
             py::arg("energy_threshold") = 1e-3f,
             py::arg("energy_gate_decimation") = size_t(100),
             py::arg("coarse_decimation") = size_t(4),
             py::arg("coarse_repetitions") = size_t(1),
             py::arg("coarse_margin") = size_t(16))
        .def_static("make_from_file",
                    &gr::uwb::UwbDetector::make_from_file,
                    py::arg("template_file"),
                    py::arg("pre_trigger") = size_t(2032),
                    py::arg("capture") = size_t(200000),
                    py::arg("energy_threshold") = 1e-3f,
                    py::arg("energy_gate_decimation") = size_t(100),
                    py::arg("coarse_decimation") = size_t(4),
                    py::arg("coarse_repetitions") = size_t(1),
                    py::arg("coarse_margin") = size_t(16))
        .def("pre_trigger", &gr::uwb::UwbDetector::pre_trigger)
        .def("set_pre_trigger",
             &gr::uwb::UwbDetector::set_pre_trigger,
             py::arg("pre_trigger"))
        .def("capture", &gr::uwb::UwbDetector::capture)
        .def("set_capture",
             &gr::uwb::UwbDetector::set_capture,
             py::arg("capture"));
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
        .def("samples_written", &gr::uwb::UwbPacketWriter::samples_written);
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
    bind_packet_writer(m);
}
