/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for scheduled dump JSONL + SC16 window IO.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_scheduled_dump_io.h>
#include <gnuradio/uwb/uwb_window_crop_core.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

using gr::uwb::core::DemodResultMeta;
using gr::uwb::core::DumpWindowMeta;
using gr::uwb::core::json_f64;
using gr::uwb::core::json_i64;
using gr::uwb::core::json_str;
using gr::uwb::core::JsonField;
using gr::uwb::core::load_capture_jsonl;
using gr::uwb::core::load_demod_jsonl;
using gr::uwb::core::parse_demod_jsonl_line;
using gr::uwb::core::parse_dump_jsonl_line;
using gr::uwb::core::plan_window_crop;
using gr::uwb::core::read_sc16_window;
using gr::uwb::core::WindowCropGeom;
using gr::uwb::core::write_json_object;
using gr::uwb::core::write_sc16_window;

BOOST_AUTO_TEST_CASE(test_parse_writer_style_jsonl)
{
    const char* line =
        "{\"packet_id\":7,\"start_sample\":100,\"trigger_sample\":100,"
        "\"sample_rate\":737280000,\"sample_count\":434995,"
        "\"file_offset_samples\":12345,\"detection_metric\":1.000000,"
        "\"pre_trigger_samples\":221184,\"sample_format\":\"sc16\","
        "\"iq_scale\":32768,\"window_start_sample\":778816,"
        "\"predicted_start_sample\":1000000,\"pre_guard_samples\":221184,"
        "\"capture_samples\":140083,\"post_guard_samples\":73728,"
        "\"schedule_index\":3,\"capture_mode\":\"scheduled\","
        "\"lock_state\":\"locked\"}\n";
    DumpWindowMeta m;
    BOOST_REQUIRE(parse_dump_jsonl_line(line, m));
    BOOST_CHECK_EQUAL(m.packet_id, 7);
    BOOST_CHECK_EQUAL(m.schedule_index, 3);
    BOOST_CHECK_EQUAL(m.sample_count, 434995);
    BOOST_CHECK_EQUAL(m.file_offset_samples, 12345);
    BOOST_CHECK_EQUAL(m.window_start_sample, 778816);
    BOOST_CHECK_EQUAL(m.predicted_start_sample, 1000000);
    BOOST_CHECK_EQUAL(m.pre_guard_samples, 221184);
    BOOST_CHECK_EQUAL(m.capture_samples, 140083);
    BOOST_CHECK_EQUAL(m.post_guard_samples, 73728);
    BOOST_CHECK_EQUAL(m.capture_mode, "scheduled");
    BOOST_CHECK_EQUAL(m.lock_state, "locked");
    BOOST_CHECK_EQUAL(m.sample_format, "sc16");
}

BOOST_AUTO_TEST_CASE(test_parse_demod_results_jsonl)
{
    const char* line =
        "{\"packet_id\":2,\"schedule_index\":1,\"status\":\"success\","
        "\"fcs_pass\":true,\"detected_start_sample\":1354000,"
        "\"predicted_start_sample\":1353972,"
        "\"native_predicted_start\":1000000,"
        "\"native_window_start\":778816,\"resample_us\":1600.5,"
        "\"t_total_us\":2400.0}\n";
    DemodResultMeta m;
    BOOST_REQUIRE(parse_demod_jsonl_line(line, m));
    BOOST_CHECK_EQUAL(m.packet_id, 2);
    BOOST_CHECK(m.fcs_pass);
    BOOST_CHECK_EQUAL(m.status, "success");
    BOOST_CHECK(m.has_detected_start);
    BOOST_CHECK_EQUAL(m.detected_start_sample, 1354000);
    BOOST_CHECK_EQUAL(m.native_predicted_start, 1000000);
    BOOST_CHECK_CLOSE(m.resample_us, 1600.5, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_jsonl_roundtrip_files)
{
    char jsonl_tmpl[] = "/tmp/uwb_dump_io_XXXXXX";
    const int fd = mkstemp(jsonl_tmpl);
    BOOST_REQUIRE(fd >= 0);
    close(fd);
    {
        std::ofstream os(jsonl_tmpl);
        std::vector<JsonField> f = {
            { "packet_id", json_i64(1) },
            { "sample_count", json_i64(8) },
            { "file_offset_samples", json_i64(0) },
            { "window_start_sample", json_i64(10) },
            { "predicted_start_sample", json_i64(20) },
            { "capture_mode", json_str("scheduled") },
            { "sample_rate", json_f64(737280000.0) },
        };
        write_json_object(os, f);
    }
    const auto wins = load_capture_jsonl(jsonl_tmpl);
    BOOST_REQUIRE_EQUAL(wins.size(), 1u);
    BOOST_CHECK_EQUAL(wins[0].packet_id, 1);
    BOOST_CHECK_EQUAL(wins[0].sample_count, 8);
    BOOST_CHECK_EQUAL(wins[0].capture_mode, "scheduled");
    std::remove(jsonl_tmpl);
}

BOOST_AUTO_TEST_CASE(test_sc16_window_rw)
{
    char iq_tmpl[] = "/tmp/uwb_dump_iq_XXXXXX";
    const int fd = mkstemp(iq_tmpl);
    BOOST_REQUIRE(fd >= 0);
    close(fd);
    std::vector<int16_t> a(16);
    for (int i = 0; i < 16; ++i)
        a[static_cast<size_t>(i)] = static_cast<int16_t>(i * 11 - 40);
    {
        std::ofstream os(iq_tmpl, std::ios::binary);
        BOOST_REQUIRE(write_sc16_window(os, a.data(), 4));
        BOOST_REQUIRE(write_sc16_window(os, a.data() + 8, 4));
    }
    std::ifstream is(iq_tmpl, std::ios::binary);
    std::vector<int16_t> b;
    BOOST_REQUIRE(read_sc16_window(is, 4, 4, b));
    BOOST_REQUIRE_EQUAL(b.size(), 8u);
    for (int i = 0; i < 8; ++i)
        BOOST_CHECK_EQUAL(b[static_cast<size_t>(i)], a[static_cast<size_t>(8 + i)]);
    std::remove(iq_tmpl);
}

BOOST_AUTO_TEST_CASE(test_crop_plan_matches_defaults)
{
    WindowCropGeom g{ 7373, 140083, 3023 };
    const auto plan = plan_window_crop(778816, 434995, 1000000, g, false);
    BOOST_CHECK_EQUAL(plan.out_count, 7373 + 140083 + 3023);
    BOOST_CHECK_EQUAL(plan.window_start, 1000000 - 7373);
}
