/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA Test Suite for UHD SC16 Direct RX and Multi-Core Stream Pipeline.
 */

#define BOOST_TEST_MODULE qa_uwb_uhd_sc16_pipeline
#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace gr::uwb::pipeline;

BOOST_AUTO_TEST_SUITE(qa_uwb_uhd_sc16_pipeline)

// ---------------------------------------------------------------------------
// Test Case 1: test_sc16_to_cf32_conversion_precision
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sc16_to_cf32_conversion_precision)
{
    const std::vector<size_t> test_sizes = {
        1, 7, 8, 15, 16, 31, 32, 64, 100, 1024, 4000
    };

    const std::vector<int16_t> boundary_values = {
        -32768, -32767, -1, 0, 1, 32766, 32767
    };

    const std::vector<float> scales = {
        1.0f / 32768.0f, // UnitRange
        1.0f             // RawInteger
    };

    for (float scale : scales) {
        for (size_t n : test_sizes) {
            std::vector<std::complex<int16_t>> in(n);
            std::vector<std::complex<float>> out_scalar(n);
            std::vector<std::complex<float>> out_avx2(n);
            std::vector<std::complex<float>> out_auto(n);

            for (size_t i = 0; i < n; ++i) {
                int16_t re = boundary_values[(i * 3 + 1) % boundary_values.size()];
                int16_t im = boundary_values[(i * 5 + 2) % boundary_values.size()];
                in[i] = std::complex<int16_t>(re, im);
            }

            sc16_to_cf32_scalar(in.data(), out_scalar.data(), n, scale);
            sc16_to_cf32_avx2(in.data(), out_avx2.data(), n, scale);
            sc16_to_cf32(in.data(), out_auto.data(), n, scale);

            for (size_t i = 0; i < n; ++i) {
                float exp_re = static_cast<float>(in[i].real()) * scale;
                float exp_im = static_cast<float>(in[i].imag()) * scale;

                // Exact scalar accuracy
                BOOST_CHECK_EQUAL(out_scalar[i].real(), exp_re);
                BOOST_CHECK_EQUAL(out_scalar[i].imag(), exp_im);

                // Exact bit-equivalence: scalar vs AVX2
                BOOST_CHECK_EQUAL(out_scalar[i].real(), out_avx2[i].real());
                BOOST_CHECK_EQUAL(out_scalar[i].imag(), out_avx2[i].imag());

                // Exact bit-equivalence: auto dispatch
                BOOST_CHECK_EQUAL(out_auto[i].real(), out_scalar[i].real());
                BOOST_CHECK_EQUAL(out_auto[i].imag(), out_scalar[i].imag());
            }
        }
    }

    // Unaligned memory buffer testing (pointer offset by 1, 2, 3 bytes)
    const size_t unaligned_n = 64;
    const size_t buf_bytes = sizeof(std::complex<int16_t>) * unaligned_n + 64;
    std::vector<uint8_t> raw_in(buf_bytes, 0);
    std::vector<uint8_t> raw_out_scalar(sizeof(std::complex<float>) * unaligned_n + 64, 0);
    std::vector<uint8_t> raw_out_avx2(sizeof(std::complex<float>) * unaligned_n + 64, 0);

    for (size_t byte_offset : {1, 2, 3}) {
        auto* in_ptr = reinterpret_cast<std::complex<int16_t>*>(raw_in.data() + byte_offset);
        auto* out_s_ptr = reinterpret_cast<std::complex<float>*>(raw_out_scalar.data() + byte_offset);
        auto* out_a_ptr = reinterpret_cast<std::complex<float>*>(raw_out_avx2.data() + byte_offset);

        for (size_t i = 0; i < unaligned_n; ++i) {
            int16_t re = boundary_values[i % boundary_values.size()];
            int16_t im = boundary_values[(i + 4) % boundary_values.size()];
            in_ptr[i] = std::complex<int16_t>(re, im);
        }

        const float scale = 1.0f / 32768.0f;
        sc16_to_cf32_scalar(in_ptr, out_s_ptr, unaligned_n, scale);
        sc16_to_cf32_avx2(in_ptr, out_a_ptr, unaligned_n, scale);

        for (size_t i = 0; i < unaligned_n; ++i) {
            BOOST_CHECK_EQUAL(out_s_ptr[i].real(), out_a_ptr[i].real());
            BOOST_CHECK_EQUAL(out_s_ptr[i].imag(), out_a_ptr[i].imag());
        }
    }
}

// ---------------------------------------------------------------------------
// Test Case 2: test_sc16_ring_pool_spsc_concurrency
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_sc16_ring_pool_spsc_concurrency)
{
    const size_t cap = 4;
    const size_t slot_samples = 256;
    Sc16WindowRingPool pool(cap, slot_samples);
    BOOST_CHECK_EQUAL(pool.capacity(), cap);

    // Initial state: acquire 1 slot
    Sc16WindowSlot* s0 = pool.acquire_filling_slot();
    BOOST_REQUIRE(s0 != nullptr);
    BOOST_CHECK(s0->state.load() == SlotState::Filling);
    s0->window_id = 100;
    BOOST_CHECK(pool.publish_ready_slot(s0));
    BOOST_CHECK(s0->state.load() == SlotState::Ready);

    Sc16WindowSlot* r0 = pool.acquire_ready_slot();
    BOOST_REQUIRE(r0 == s0);
    BOOST_CHECK(r0->state.load() == SlotState::Reading);
    BOOST_CHECK_EQUAL(r0->window_id, 100);
    pool.release_reading_slot(r0);
    BOOST_CHECK(r0->state.load() == SlotState::Free);

    // Test abort_filling_slot
    Sc16WindowSlot* s_abort = pool.acquire_filling_slot();
    BOOST_REQUIRE(s_abort != nullptr);
    BOOST_CHECK(s_abort->state.load() == SlotState::Filling);
    pool.abort_filling_slot(s_abort);
    BOOST_CHECK(s_abort->state.load() == SlotState::Free);

    // Fill all 4 slots, 5th must return nullptr
    std::vector<Sc16WindowSlot*> filled;
    for (size_t i = 0; i < cap; ++i) {
        Sc16WindowSlot* s = pool.acquire_filling_slot();
        BOOST_REQUIRE_MESSAGE(s != nullptr, "Should acquire slot " + std::to_string(i));
        s->window_id = 1000 + i;
        BOOST_CHECK(pool.publish_ready_slot(s));
        filled.push_back(s);
    }
    // 5th attempt must return nullptr (non-blocking drop condition)
    Sc16WindowSlot* s5 = pool.acquire_filling_slot();
    BOOST_CHECK(s5 == nullptr);

    // Pop all 4 in FIFO order
    for (size_t i = 0; i < cap; ++i) {
        Sc16WindowSlot* r = pool.acquire_ready_slot();
        BOOST_REQUIRE(r != nullptr);
        BOOST_CHECK_EQUAL(r->window_id, 1000 + i);
        BOOST_CHECK(r == filled[i]);
        pool.release_reading_slot(r);
    }
    BOOST_CHECK(pool.acquire_ready_slot() == nullptr);

    // Wrap-around verification
    for (size_t i = 0; i < cap; ++i) {
        Sc16WindowSlot* s = pool.acquire_filling_slot();
        BOOST_REQUIRE(s != nullptr);
        s->window_id = 2000 + i;
        pool.publish_ready_slot(s);
    }
    for (size_t i = 0; i < 2; ++i) {
        Sc16WindowSlot* r = pool.acquire_ready_slot();
        BOOST_REQUIRE(r != nullptr);
        BOOST_CHECK_EQUAL(r->window_id, 2000 + i);
        pool.release_reading_slot(r);
    }
    for (size_t i = 0; i < 2; ++i) {
        Sc16WindowSlot* s = pool.acquire_filling_slot();
        BOOST_REQUIRE(s != nullptr);
        s->window_id = 3000 + i;
        pool.publish_ready_slot(s);
    }
    const uint64_t expected_wrap_ids[] = {2002, 2003, 3000, 3001};
    for (uint64_t exp_id : expected_wrap_ids) {
        Sc16WindowSlot* r = pool.acquire_ready_slot();
        BOOST_REQUIRE(r != nullptr);
        BOOST_CHECK_EQUAL(r->window_id, exp_id);
        pool.release_reading_slot(r);
    }
    BOOST_CHECK(pool.acquire_ready_slot() == nullptr);

    // Multi-threaded producer-consumer test (N=5000)
    const size_t N = 5000;
    std::atomic<bool> producer_done{false};
    std::atomic<size_t> items_consumed{0};
    std::vector<uint64_t> consumed_ids;
    consumed_ids.reserve(N);

    std::thread producer([&]() {
        for (size_t i = 0; i < N; ++i) {
            Sc16WindowSlot* s = nullptr;
            while (!s) {
                s = pool.acquire_filling_slot();
                if (!s) {
                    std::this_thread::yield();
                }
            }
            s->window_id = i;
            s->native_count = 10;
            if (s->sc16_samples.size() >= 10) {
                s->sc16_samples[0] = std::complex<int16_t>(
                    static_cast<int16_t>(i & 0x7FFF),
                    static_cast<int16_t>(-(i & 0x7FFF))
                );
            }
            pool.publish_ready_slot(s);
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::thread consumer([&]() {
        while (items_consumed.load(std::memory_order_relaxed) < N) {
            Sc16WindowSlot* r = pool.acquire_ready_slot();
            if (!r) {
                if (producer_done.load(std::memory_order_acquire) &&
                    items_consumed.load(std::memory_order_relaxed) >= N) {
                    break;
                }
                std::this_thread::yield();
                continue;
            }
            uint64_t wid = r->window_id;
            consumed_ids.push_back(wid);
            if (r->native_count >= 10 && !r->sc16_samples.empty()) {
                auto val = r->sc16_samples[0];
                int16_t exp_re = static_cast<int16_t>(wid & 0x7FFF);
                int16_t exp_im = static_cast<int16_t>(-(wid & 0x7FFF));
                BOOST_CHECK_EQUAL(val.real(), exp_re);
                BOOST_CHECK_EQUAL(val.imag(), exp_im);
            }
            pool.release_reading_slot(r);
            items_consumed.fetch_add(1, std::memory_order_relaxed);
        }
    });

    producer.join();
    consumer.join();

    BOOST_CHECK_EQUAL(consumed_ids.size(), N);
    for (size_t i = 0; i < N; ++i) {
        BOOST_CHECK_EQUAL(consumed_ids[i], i);
    }
}

// ---------------------------------------------------------------------------
// Test Case 3: test_replay_sc16_source
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_replay_sc16_source)
{
    const std::string tmp_file = "/tmp/test_qa_replay_sc16_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".bin";
    const size_t total_samples = 2000;
    std::vector<std::complex<int16_t>> test_data(total_samples);
    for (size_t i = 0; i < total_samples; ++i) {
        test_data[i] = std::complex<int16_t>(
            static_cast<int16_t>(i * 3),
            static_cast<int16_t>(-static_cast<int>(i) * 2)
        );
    }

    {
        std::ofstream ofs(tmp_file, std::ios::binary);
        BOOST_REQUIRE(ofs.is_open());
        ofs.write(reinterpret_cast<const char*>(test_data.data()),
                  total_samples * sizeof(std::complex<int16_t>));
    }

    const double kRate = 998.4e6;
    ReplaySc16Source src(tmp_file, kRate, false);
    BOOST_REQUIRE(src.open());
    BOOST_CHECK_CLOSE(src.actual_rx_rate(), kRate, 1e-4);
    BOOST_CHECK(!src.is_eof());

    const size_t chunk_size = 512;
    std::vector<std::complex<int16_t>> chunk_buf(chunk_size);
    size_t samples_read_total = 0;
    int64_t expected_abs = 0;
    bool saw_eof = false;

    while (samples_read_total < total_samples) {
        StreamReadMeta meta;
        size_t n = src.read_samples(chunk_buf.data(), chunk_buf.size(), meta);
        if (n == 0 && meta.is_eof) {
            saw_eof = true;
            break;
        }
        BOOST_REQUIRE(n > 0);
        BOOST_CHECK_EQUAL(meta.start_sample_abs, expected_abs);
        for (size_t j = 0; j < n; ++j) {
            BOOST_CHECK_EQUAL(chunk_buf[j].real(), test_data[samples_read_total + j].real());
            BOOST_CHECK_EQUAL(chunk_buf[j].imag(), test_data[samples_read_total + j].imag());
        }
        samples_read_total += n;
        expected_abs += static_cast<int64_t>(n);
        if (meta.is_eof) {
            saw_eof = true;
        }
    }

    BOOST_CHECK_EQUAL(samples_read_total, total_samples);

    if (!saw_eof) {
        StreamReadMeta meta;
        size_t n = src.read_samples(chunk_buf.data(), chunk_buf.size(), meta);
        BOOST_CHECK_EQUAL(n, 0);
        BOOST_CHECK(meta.is_eof || src.is_eof());
    } else {
        BOOST_CHECK(src.is_eof());
    }

    src.close();
    std::remove(tmp_file.c_str());
}

// ---------------------------------------------------------------------------
// Test Case 4: test_pipeline_e2e_mock
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pipeline_e2e_mock)
{
    const std::string tmp_file = "/tmp/test_qa_pipeline_e2e_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".bin";
    const size_t num_samples = 20000;
    std::vector<std::complex<int16_t>> synthetic_data(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
        synthetic_data[i] = std::complex<int16_t>(
            static_cast<int16_t>(std::sin(2.0 * M_PI * i / 100.0) * 1000.0),
            static_cast<int16_t>(std::cos(2.0 * M_PI * i / 100.0) * 1000.0)
        );
    }
    {
        std::ofstream ofs(tmp_file, std::ios::binary);
        BOOST_REQUIRE(ofs.is_open());
        ofs.write(reinterpret_cast<const char*>(synthetic_data.data()),
                  num_samples * sizeof(std::complex<int16_t>));
    }

    PipelineConfig cfg;
    cfg.source_type = StreamSourceType::Replay;
    cfg.replay_file = tmp_file;
    cfg.paced_replay = false;
    cfg.native_rate = 998.4e6;
    cfg.work_rate = 998.4e6;
    cfg.schedule_t0 = 0.0;
    cfg.period_s = 0.00001; // 10 us, i.e. ~9984 samples
    cfg.pre_guard_samples = 100;
    cfg.capture_samples = 800;
    cfg.post_guard_samples = 100;
    cfg.pool_capacity = 4;
    cfg.consumer_mode = ConsumerMode::SicOnly;
    cfg.max_windows = 2;

    UhdSc16Pipeline pipeline(cfg);
    BOOST_REQUIRE(pipeline.initialize());
    pipeline.start();
    pipeline.wait();

    std::string err;
    BOOST_CHECK_MESSAGE(pipeline.ledger().verify_invariants(&err), err);
    BOOST_CHECK_EQUAL(pipeline.results().size(), 2);

    for (size_t i = 0; i < pipeline.results().size(); ++i) {
        const auto& res = pipeline.results()[i];
        BOOST_CHECK_EQUAL(res.window_id, i);
        BOOST_CHECK_GE(res.timing.slice_us, 0.0);
        BOOST_CHECK_GE(res.timing.queue_us, 0.0);
        BOOST_CHECK_GE(res.timing.convert_us, 0.0);
        BOOST_CHECK_GE(res.timing.resample_us, 0.0);
        BOOST_CHECK_GE(res.timing.sic_us, 0.0);
        BOOST_CHECK_GE(res.timing.sensing_us, 0.0);
        BOOST_CHECK_GE(res.timing.e2e_us, 0.0);
    }

    std::remove(tmp_file.c_str());
}

// ---------------------------------------------------------------------------
// Test Case 5: test_pipeline_stop_and_cancel
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pipeline_stop_and_cancel)
{
    const std::string tmp_file = "/tmp/test_qa_pipeline_cancel_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".bin";
    const size_t num_samples = 200000;
    std::vector<std::complex<int16_t>> synthetic_data(num_samples);
    for (size_t i = 0; i < num_samples; ++i) {
        synthetic_data[i] = std::complex<int16_t>(
            static_cast<int16_t>(i % 32767),
            static_cast<int16_t>(-(i % 32767))
        );
    }
    {
        std::ofstream ofs(tmp_file, std::ios::binary);
        BOOST_REQUIRE(ofs.is_open());
        ofs.write(reinterpret_cast<const char*>(synthetic_data.data()),
                  num_samples * sizeof(std::complex<int16_t>));
    }

    PipelineConfig cfg;
    cfg.source_type = StreamSourceType::Replay;
    cfg.replay_file = tmp_file;
    cfg.paced_replay = false;
    cfg.native_rate = 998.4e6;
    cfg.work_rate = 998.4e6;
    cfg.schedule_t0 = 0.0;
    cfg.period_s = 0.00001; // 10 us
    cfg.pre_guard_samples = 100;
    cfg.capture_samples = 800;
    cfg.post_guard_samples = 100;
    cfg.pool_capacity = 4;
    cfg.consumer_mode = ConsumerMode::SicOnly;
    cfg.max_windows = 200; // Large number of windows so it runs continuously

    UhdSc16Pipeline pipeline(cfg);
    BOOST_REQUIRE(pipeline.initialize());

    pipeline.start();

    // Sleep briefly to allow pipeline to start processing windows
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Request stop mid-stream
    const auto t_stop_start = std::chrono::steady_clock::now();
    pipeline.stop();
    pipeline.wait();
    const auto t_stop_end = std::chrono::steady_clock::now();

    const double stop_elapsed_ms = std::chrono::duration<double, std::milli>(t_stop_end - t_stop_start).count();

    // Verify wait() exits promptly without hanging (e.g. well under 2 seconds)
    BOOST_CHECK_LE(stop_elapsed_ms, 2000.0);

    // Ledger invariants must still hold
    std::string err;
    BOOST_CHECK_MESSAGE(pipeline.ledger().verify_invariants(&err), err);

    std::remove(tmp_file.c_str());
}

namespace {

std::string find_taps_path(const std::string& rel)
{
    const char* prefixes[] = {
        "",
        "testdata/",
        "../testdata/",
        "../../testdata/",
        "../../../testdata/",
        UWB_TESTDATA_DIR "/",
    };
    for (const char* p : prefixes) {
        std::string cand = std::string(p) + rel;
        std::ifstream f(cand, std::ios::binary);
        if (f.good()) {
            return cand;
        }
    }
    return "";
}

std::vector<float> load_taps_data(const std::string& rel)
{
    std::string path = find_taps_path(rel);
    BOOST_REQUIRE_MESSAGE(!path.empty(), "Cannot find taps file: " + rel);
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE(f);
    auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()), static_cast<std::streamsize>(bytes));
    return taps;
}

} // namespace

// ---------------------------------------------------------------------------
// Test Case 6: test_resampler_process_sc16_precision
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_resampler_process_sc16_precision)
{
    using gr::uwb::core::RationalResampler65_48Core;

    const std::vector<std::string> tap_files = {
        "resampler_65_48/taps_realtime_minorder.txt",
        "resampler_65_48/taps_quality_minorder.txt"
    };

    const std::vector<size_t> test_sizes = {
        1, 7, 8, 13, 15, 16, 31, 32, 47, 48, 49, 64, 65, 127, 128, 256, 1024, 4000, 56000
    };

    const std::vector<float> scales = {
        1.0f / 32768.0f,
        1.0f
    };

    const std::vector<int16_t> boundary_values = {
        -32768, -32767, -1, 0, 1, 32766, 32767
    };

    float global_max_abs_fused_vs_ref = 0.0f;
    float global_max_abs_fused_vs_scalar = 0.0f;

    for (const auto& tap_file : tap_files) {
        const auto taps = load_taps_data(tap_file);
        BOOST_REQUIRE_MESSAGE(!taps.empty(), "Failed to load taps: " + tap_file);

        for (float scale : scales) {
            for (size_t n : test_sizes) {
                // Generate input with both sinusoidal variations and boundary/extreme values
                std::vector<std::complex<int16_t>> in_sc16(n);
                for (size_t i = 0; i < n; ++i) {
                    if (i % 8 == 0) {
                        int16_t b_re = boundary_values[(i / 8) % boundary_values.size()];
                        int16_t b_im = boundary_values[(i / 8 + 3) % boundary_values.size()];
                        in_sc16[i] = std::complex<int16_t>(b_re, b_im);
                    } else {
                        int16_t re = static_cast<int16_t>(std::sin(2.0 * M_PI * i / 37.0) * 30000.0);
                        int16_t im = static_cast<int16_t>(std::cos(2.0 * M_PI * i / 43.0) * 30000.0);
                        in_sc16[i] = std::complex<int16_t>(re, im);
                    }
                }

                // Reference step-by-step path: sc16_to_cf32 + process
                std::vector<std::complex<float>> cf32_input(n);
                sc16_to_cf32(in_sc16.data(), cf32_input.data(), n, scale);

                RationalResampler65_48Core resamp_ref(taps.data(), taps.size());
                const size_t max_out = (n * 65 + 47) / 48 + 300;
                std::vector<std::complex<float>> out_ref(max_out);
                auto res_ref = resamp_ref.process(cf32_input.data(), n, out_ref.data(), max_out);

                std::vector<std::complex<float>> flush_ref(max_out);
                size_t flush_ref_n = resamp_ref.flush(flush_ref.data(), max_out);

                // Fused path: process_sc16
                RationalResampler65_48Core resamp_fused(taps.data(), taps.size());
                std::vector<std::complex<float>> out_fused(max_out);
                auto res_fused = resamp_fused.process_sc16(in_sc16.data(), n, scale, out_fused.data(), max_out);

                std::vector<std::complex<float>> flush_fused(max_out);
                size_t flush_fused_n = resamp_fused.flush(flush_fused.data(), max_out);

                // Scalar baseline reference path for numerical verification
                RationalResampler65_48Core resamp_scalar(taps.data(), taps.size());
                resamp_scalar.force_scalar_kernel();
                std::vector<std::complex<float>> cf32_scalar(n);
                sc16_to_cf32_scalar(in_sc16.data(), cf32_scalar.data(), n, scale);
                std::vector<std::complex<float>> out_scalar(max_out);
                auto res_scalar = resamp_scalar.process(cf32_scalar.data(), n, out_scalar.data(), max_out);

                // Check production and consumption counts
                BOOST_REQUIRE_EQUAL(res_ref.produced, res_fused.produced);
                BOOST_REQUIRE_EQUAL(res_ref.consumed, res_fused.consumed);
                BOOST_REQUIRE_EQUAL(res_ref.produced, res_scalar.produced);
                BOOST_REQUIRE_EQUAL(flush_ref_n, flush_fused_n);

                // 1. Bit-exact comparison: process_sc16 vs (sc16_to_cf32 + process)
                float max_abs_diff_ref = 0.0f;
                for (size_t i = 0; i < res_ref.produced; ++i) {
                    float diff_re = std::abs(out_ref[i].real() - out_fused[i].real());
                    float diff_im = std::abs(out_ref[i].imag() - out_fused[i].imag());
                    float diff = std::max(diff_re, diff_im);
                    if (diff > max_abs_diff_ref) {
                        max_abs_diff_ref = diff;
                    }
                }
                for (size_t i = 0; i < flush_ref_n; ++i) {
                    float diff_re = std::abs(flush_ref[i].real() - flush_fused[i].real());
                    float diff_im = std::abs(flush_ref[i].imag() - flush_fused[i].imag());
                    float diff = std::max(diff_re, diff_im);
                    if (diff > max_abs_diff_ref) {
                        max_abs_diff_ref = diff;
                    }
                }
                if (max_abs_diff_ref > global_max_abs_fused_vs_ref) {
                    global_max_abs_fused_vs_ref = max_abs_diff_ref;
                }
                BOOST_CHECK_MESSAGE(max_abs_diff_ref == 0.0f,
                                    "Bit-exact mismatch between process_sc16 and (sc16_to_cf32 + process) on " +
                                    tap_file + " n=" + std::to_string(n) +
                                    " max_diff=" + std::to_string(max_abs_diff_ref));

                // 2. Comparison against scalar baseline: verify within float machine precision (< 1e-6 relative to peak scale)
                float max_abs_diff_scalar = 0.0f;
                for (size_t i = 0; i < res_ref.produced; ++i) {
                    float diff_re = std::abs(out_scalar[i].real() - out_fused[i].real());
                    float diff_im = std::abs(out_scalar[i].imag() - out_fused[i].imag());
                    float diff = std::max(diff_re, diff_im);
                    if (diff > max_abs_diff_scalar) {
                        max_abs_diff_scalar = diff;
                    }
                }
                const float normalized_scalar_err = max_abs_diff_scalar / std::max(1.0f, scale * 32768.0f);
                if (normalized_scalar_err > global_max_abs_fused_vs_scalar) {
                    global_max_abs_fused_vs_scalar = normalized_scalar_err;
                }
                BOOST_CHECK_MESSAGE(normalized_scalar_err < 1e-6f,
                                    "Precision test vs scalar reference failed for " + tap_file +
                                    " n=" + std::to_string(n) +
                                    " normalized_scalar_err=" + std::to_string(normalized_scalar_err));
            }
        }
    }

    std::cout << "[QA Precision] Global max abs diff (process_sc16 vs sc16_to_cf32+process): "
              << global_max_abs_fused_vs_ref << " (bit-exact 0.00e+00)\n";
    std::cout << "[QA Precision] Global max abs diff (process_sc16 vs scalar baseline): "
              << global_max_abs_fused_vs_scalar << " (< 1e-6)\n";
}

// ---------------------------------------------------------------------------
// Test Case 7: test_resampler_process_sc16_chunking_and_streaming
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_resampler_process_sc16_chunking_and_streaming)
{
    using gr::uwb::core::RationalResampler65_48Core;

    const std::vector<std::string> tap_files = {
        "resampler_65_48/taps_realtime_minorder.txt",
        "resampler_65_48/taps_quality_minorder.txt"
    };

    const size_t total_samples = 12000;
    const float scale = 1.0f / 32768.0f;

    const std::vector<int16_t> boundary_values = {
        -32768, -32767, -1, 0, 1, 32766, 32767
    };

    std::vector<std::complex<int16_t>> in_sc16(total_samples);
    for (size_t i = 0; i < total_samples; ++i) {
        if (i % 16 == 0) {
            int16_t b_re = boundary_values[(i / 16) % boundary_values.size()];
            int16_t b_im = boundary_values[(i / 16 + 2) % boundary_values.size()];
            in_sc16[i] = std::complex<int16_t>(b_re, b_im);
        } else {
            int16_t re = static_cast<int16_t>(std::sin(2.0 * M_PI * i / 23.0) * 28000.0);
            int16_t im = static_cast<int16_t>(std::cos(2.0 * M_PI * i / 29.0) * 28000.0);
            in_sc16[i] = std::complex<int16_t>(re, im);
        }
    }

    // Specific chunk sizes to thoroughly test:
    // small prime chunks (1, 7, 13), boundary sizes (32, 48, 64, 65, 128), and larger windows (512, 1000, 4000)
    const std::vector<size_t> fixed_chunk_sizes = {
        1, 7, 13, 32, 48, 64, 65, 128, 512, 1000, 4000
    };

    // Mixed variable chunk pattern
    const std::vector<size_t> mixed_chunk_pattern = {
        1, 7, 13, 64, 65, 128, 4000, 3, 16, 48, 100, 256, 384, 512, 768, 1024, 2048, 1500
    };

    for (const auto& tap_file : tap_files) {
        const auto taps = load_taps_data(tap_file);
        BOOST_REQUIRE_MESSAGE(!taps.empty(), "Failed to load taps: " + tap_file);

        // 1. One-shot execution as gold reference (process_sc16 + flush)
        RationalResampler65_48Core resamp_oneshot(taps.data(), taps.size());
        const size_t max_out_total = RationalResampler65_48Core::expected_output_length(total_samples, taps.size()) + 500;
        std::vector<std::complex<float>> out_gold;
        out_gold.reserve(max_out_total);

        std::vector<std::complex<float>> proc_oneshot(max_out_total);
        auto res_oneshot = resamp_oneshot.process_sc16(in_sc16.data(), total_samples, scale,
                                                      proc_oneshot.data(), max_out_total);
        out_gold.insert(out_gold.end(), proc_oneshot.begin(), proc_oneshot.begin() + res_oneshot.produced);

        std::vector<std::complex<float>> flush_oneshot(500);
        size_t flush_oneshot_n = resamp_oneshot.flush(flush_oneshot.data(), flush_oneshot.size());
        out_gold.insert(out_gold.end(), flush_oneshot.begin(), flush_oneshot.begin() + flush_oneshot_n);

        BOOST_CHECK_EQUAL(res_oneshot.consumed, total_samples);
        BOOST_CHECK(resamp_oneshot.flush_complete());
        BOOST_CHECK_EQUAL(out_gold.size(), RationalResampler65_48Core::expected_output_length(total_samples, taps.size()));

        // 2. Fixed chunk size streaming tests
        for (size_t chunk_sz : fixed_chunk_sizes) {
            RationalResampler65_48Core resamp_chunked(taps.data(), taps.size());
            std::vector<std::complex<float>> out_chunked;
            out_chunked.reserve(max_out_total);

            size_t offset = 0;
            size_t total_consumed = 0;
            while (offset < total_samples) {
                size_t c_size = std::min(chunk_sz, total_samples - offset);
                size_t chunk_max_out = (c_size * 65 + 47) / 48 + 200;
                std::vector<std::complex<float>> chunk_out(chunk_max_out);

                auto res = resamp_chunked.process_sc16(in_sc16.data() + offset, c_size, scale,
                                                      chunk_out.data(), chunk_max_out);
                out_chunked.insert(out_chunked.end(), chunk_out.begin(), chunk_out.begin() + res.produced);
                offset += res.consumed;
                total_consumed += res.consumed;
            }

            std::vector<std::complex<float>> flush_chunked(500);
            size_t flush_chunked_n = resamp_chunked.flush(flush_chunked.data(), flush_chunked.size());
            out_chunked.insert(out_chunked.end(), flush_chunked.begin(), flush_chunked.begin() + flush_chunked_n);

            BOOST_CHECK_EQUAL(total_consumed, total_samples);
            BOOST_CHECK(resamp_chunked.flush_complete());
            BOOST_REQUIRE_EQUAL(out_chunked.size(), out_gold.size());

            float max_diff = 0.0f;
            for (size_t i = 0; i < out_gold.size(); ++i) {
                float diff_re = std::abs(out_chunked[i].real() - out_gold[i].real());
                float diff_im = std::abs(out_chunked[i].imag() - out_gold[i].imag());
                float diff = std::max(diff_re, diff_im);
                if (diff > max_diff) {
                    max_diff = diff;
                }
            }
            BOOST_CHECK_MESSAGE(max_diff == 0.0f,
                                "Fixed chunk size " + std::to_string(chunk_sz) +
                                " streaming output differs from one-shot on " + tap_file +
                                ": max_diff=" + std::to_string(max_diff));
        }

        // 3. Mixed variable chunk size streaming test
        {
            RationalResampler65_48Core resamp_mixed(taps.data(), taps.size());
            std::vector<std::complex<float>> out_mixed;
            out_mixed.reserve(max_out_total);

            size_t chunk_idx = 0;
            size_t offset = 0;
            size_t total_consumed = 0;

            while (offset < total_samples) {
                size_t c_size = std::min(mixed_chunk_pattern[chunk_idx % mixed_chunk_pattern.size()],
                                         total_samples - offset);
                chunk_idx++;

                size_t chunk_max_out = (c_size * 65 + 47) / 48 + 200;
                std::vector<std::complex<float>> chunk_out(chunk_max_out);

                auto res = resamp_mixed.process_sc16(in_sc16.data() + offset, c_size, scale,
                                                    chunk_out.data(), chunk_max_out);
                out_mixed.insert(out_mixed.end(), chunk_out.begin(), chunk_out.begin() + res.produced);
                offset += res.consumed;
                total_consumed += res.consumed;
            }

            std::vector<std::complex<float>> flush_mixed(500);
            size_t flush_mixed_n = resamp_mixed.flush(flush_mixed.data(), flush_mixed.size());
            out_mixed.insert(out_mixed.end(), flush_mixed.begin(), flush_mixed.begin() + flush_mixed_n);

            BOOST_CHECK_EQUAL(total_consumed, total_samples);
            BOOST_CHECK(resamp_mixed.flush_complete());
            BOOST_REQUIRE_EQUAL(out_mixed.size(), out_gold.size());

            float max_diff = 0.0f;
            for (size_t i = 0; i < out_gold.size(); ++i) {
                float diff_re = std::abs(out_mixed[i].real() - out_gold[i].real());
                float diff_im = std::abs(out_mixed[i].imag() - out_gold[i].imag());
                float diff = std::max(diff_re, diff_im);
                if (diff > max_diff) {
                    max_diff = diff;
                }
            }
            BOOST_CHECK_MESSAGE(max_diff == 0.0f,
                                "Mixed chunk streaming output differs from one-shot on " + tap_file +
                                ": max_diff=" + std::to_string(max_diff));
        }

        std::cout << "[QA Chunking] " << tap_file
                  << " successfully verified across all fixed and mixed chunk patterns (max_diff=0.00e+00)\n";
    }
}

// ===========================================================================
// Burst-level multi-core QA (dev plan 2026-11-11, contract clauses C1-C8)
// ===========================================================================
namespace {

// Deterministic SC16 stimulus shared by the multi-core cases: two
// incommensurate sinusoids, a small LCG dither (broadband content, so the fused
// 65/48 kernel is not fed a degenerate single tone) and periodic int16
// boundary values.
std::vector<std::complex<int16_t>> make_multicore_sc16_data(size_t n)
{
    static const int16_t kBoundary[] = { -32768, -32767, -1, 0, 1, 32766, 32767 };
    const size_t kBoundaryCount = sizeof(kBoundary) / sizeof(kBoundary[0]);

    std::vector<std::complex<int16_t>> data(n);
    uint32_t lcg = 0x12345678u;
    for (size_t i = 0; i < n; ++i) {
        lcg = lcg * 1664525u + 1013904223u;
        if (i % 1024 == 0) {
            data[i] = std::complex<int16_t>(
                kBoundary[(i / 1024) % kBoundaryCount],
                kBoundary[(i / 1024 + 3) % kBoundaryCount]);
            continue;
        }
        const long tone_re =
            std::lround(std::sin(2.0 * M_PI * static_cast<double>(i) / 64.0) * 20000.0);
        const long tone_im =
            std::lround(std::cos(2.0 * M_PI * static_cast<double>(i) / 96.0) * 20000.0);
        const int16_t dither_re = static_cast<int16_t>((lcg >> 16) & 0x0FFFu) - 2048;
        const int16_t dither_im = static_cast<int16_t>((lcg >> 4) & 0x0FFFu) - 2048;
        data[i] = std::complex<int16_t>(static_cast<int16_t>(tone_re + dither_re),
                                        static_cast<int16_t>(tone_im + dither_im));
    }
    return data;
}

// Unique temp file per test invocation; the caller owns std::remove().
std::string write_multicore_sc16_file(const std::string& tag,
                                      const std::vector<std::complex<int16_t>>& data)
{
    const std::string path = "/tmp/test_qa_mc_" + tag + "_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".bin";
    {
        std::ofstream ofs(path, std::ios::binary);
        BOOST_REQUIRE_MESSAGE(ofs.is_open(), "Cannot create " + path);
        ofs.write(reinterpret_cast<const char*>(data.data()),
                  static_cast<std::streamsize>(data.size() * sizeof(std::complex<int16_t>)));
        ofs.close();
        BOOST_REQUIRE_MESSAGE(ofs.good(), "Short write on " + path);
    }
    return path;
}

} // namespace

// ---------------------------------------------------------------------------
// Test Case 8: test_pipeline_multicore_bit_exactness
//   C2 (dsp_workers<=1 stays the reference), C4 (private per-worker state) and
//   acceptance gate 3.2: every WindowResult field must be bit-identical for
//   dsp_workers in {1,2,4} on the 56,000-sample 737.28e6 -> 998.4e6 path.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pipeline_multicore_bit_exactness)
{
    const size_t kWindowSamples = 56000;
    const size_t kWindows = 8;
    const int kWorkerCounts[] = { 1, 2, 4 };
    const size_t kRunCount = sizeof(kWorkerCounts) / sizeof(kWorkerCounts[0]);

    // 8 x 56,000 samples plus a small tail margin so the last window can never
    // be cancelled by an EOF short read.
    const std::string tmp_file = write_multicore_sc16_file(
        "exact", make_multicore_sc16_data(kWindowSamples * kWindows + 1024));

    PipelineConfig base;
    base.source_type = StreamSourceType::Replay;
    base.replay_file = tmp_file;
    base.paced_replay = false;
    base.native_rate = 737.28e6;
    base.work_rate = 998.4e6;
    base.schedule_t0 = 0.0;
    base.pre_guard_samples = 0;
    base.capture_samples = kWindowSamples;
    base.post_guard_samples = 0;
    base.period_s = 56000.0 / 737.28e6;
    // The pool holds one slot per window on purpose.  A file-backed replay
    // producer is ~30x faster than a 56,000-sample resample+SIC+radar burst, so
    // a 4-slot pool legitimately drops windows 4..7 of the single-worker
    // reference run and the bit-exactness comparison would compare
    // DroppedAdmission records instead of DSP output.  Drop/backpressure
    // behaviour is covered by test_pipeline_multicore_stop_and_backpressure.
    base.pool_capacity = kWindows;
    base.consumer_mode = ConsumerMode::Both;
    base.resampler_profile = "realtime_minorder";
    base.max_windows = kWindows;

    std::vector<std::vector<WindowResult>> runs;
    runs.reserve(kRunCount);

    for (size_t r = 0; r < kRunCount; ++r) {
        PipelineConfig cfg = base;
        cfg.dsp_workers = kWorkerCounts[r];
        cfg.cpu_cores_dsp.clear();

        UhdSc16Pipeline pipeline(cfg);
        BOOST_REQUIRE_MESSAGE(pipeline.initialize(),
                              "initialize() failed for dsp_workers=" +
                                  std::to_string(kWorkerCounts[r]));
        pipeline.start();
        pipeline.wait();

        std::string err;
        BOOST_REQUIRE_MESSAGE(pipeline.ledger().verify_invariants(&err),
                              "dsp_workers=" + std::to_string(kWorkerCounts[r]) +
                                  ": " + err);
        BOOST_REQUIRE_MESSAGE(pipeline.results().size() == kWindows,
                              "dsp_workers=" + std::to_string(kWorkerCounts[r]) +
                                  ": expected " + std::to_string(kWindows) +
                                  " results, got " + std::to_string(pipeline.results().size()));

        runs.push_back(pipeline.results());
    }

    // Every window is drop-free by construction, so results() is final here.
    std::remove(tmp_file.c_str());

    // Per-run contract checks: in-order ids, Completed state, worker provenance.
    for (size_t r = 0; r < kRunCount; ++r) {
        std::string worker_ids;
        for (size_t i = 0; i < runs[r].size(); ++i) {
            const WindowResult& res = runs[r][i];
            BOOST_CHECK_EQUAL(res.window_id, i);
            BOOST_CHECK_EQUAL(static_cast<int>(res.state),
                              static_cast<int>(WindowTerminalState::Completed));
            if (kWorkerCounts[r] == 1) {
                BOOST_CHECK_EQUAL(res.worker_id, 0);
            } else {
                BOOST_CHECK(res.worker_id >= 0 && res.worker_id < 4);
            }
            worker_ids += std::to_string(res.worker_id);
            worker_ids += (i + 1 == runs[r].size()) ? "" : ",";
        }
        BOOST_TEST_MESSAGE("dsp_workers=" << kWorkerCounts[r]
                           << " window worker_ids=[" << worker_ids << "]");
    }

    // Bit-exact window-by-window comparison against the single-worker reference.
    const std::vector<WindowResult>& ref = runs[0];
    for (size_t r = 1; r < kRunCount; ++r) {
        BOOST_REQUIRE_EQUAL(runs[r].size(), ref.size());
        for (size_t i = 0; i < ref.size(); ++i) {
            const WindowResult& a = ref[i];
            const WindowResult& b = runs[r][i];

            BOOST_CHECK_EQUAL(b.window_id, a.window_id);
            BOOST_CHECK_EQUAL(static_cast<int>(b.state), static_cast<int>(a.state));
            BOOST_CHECK_EQUAL(b.native_start_abs, a.native_start_abs);
            BOOST_CHECK_EQUAL(b.native_sample_count, a.native_sample_count);
            BOOST_CHECK_EQUAL(b.sic_executed, a.sic_executed);
            BOOST_CHECK_EQUAL(b.sic_committed, a.sic_committed);
            BOOST_CHECK_EQUAL(b.actual_suppression_db, a.actual_suppression_db);
            BOOST_CHECK_EQUAL(b.radar_executed, a.radar_executed);
            BOOST_CHECK_EQUAL(b.radar_sfd_found, a.radar_sfd_found);
            BOOST_CHECK_EQUAL(b.radar_peak_metric, a.radar_peak_metric);
            BOOST_CHECK_EQUAL(b.radar_detected_start, a.radar_detected_start);
        }
    }
}

// ---------------------------------------------------------------------------
// Test Case 9: test_pipeline_multicore_load_balance_and_ledger
//   C1 (one SPSC pool per worker), C5 (strictly increasing window_id) and C6
//   (ledger conservation): 16 drop-free windows must be admitted, completed and
//   re-ordered while every one of the 4 workers actually participates.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pipeline_multicore_load_balance_and_ledger)
{
    const size_t kWindowSamples = 4000;
    const size_t kWindows = 16;
    const int kWorkers = 4;

    // No resampling (native_rate == work_rate), so this case stays cheap and
    // purely exercises dispatch / ledger / reorder behaviour.
    const std::string tmp_file = write_multicore_sc16_file(
        "balance", make_multicore_sc16_data(kWindowSamples * kWindows + 4096));

    PipelineConfig cfg;
    cfg.source_type = StreamSourceType::Replay;
    cfg.replay_file = tmp_file;
    cfg.paced_replay = false;
    cfg.native_rate = 998.4e6;
    cfg.work_rate = 998.4e6;
    cfg.schedule_t0 = 0.0;
    cfg.period_s = 4000.0 / 998.4e6;
    cfg.pre_guard_samples = 0;
    cfg.capture_samples = kWindowSamples;
    cfg.post_guard_samples = 0;
    cfg.pool_capacity = 4;
    cfg.consumer_mode = ConsumerMode::SensingCir;
    cfg.dsp_workers = kWorkers;
    cfg.max_windows = kWindows;

    UhdSc16Pipeline pipeline(cfg);
    BOOST_REQUIRE(pipeline.initialize());
    pipeline.start();
    pipeline.wait();

    const PipelineLedger& ledger = pipeline.ledger();
    std::string err;
    BOOST_CHECK_MESSAGE(ledger.verify_invariants(&err), err);

    BOOST_CHECK_EQUAL(ledger.offered_windows, static_cast<uint64_t>(kWindows));
    BOOST_CHECK_EQUAL(ledger.admitted_windows, static_cast<uint64_t>(kWindows));
    BOOST_CHECK_EQUAL(ledger.completed_windows, static_cast<uint64_t>(kWindows));
    BOOST_CHECK_EQUAL(ledger.dropped_admission, 0u);
    BOOST_CHECK_EQUAL(ledger.invalid_gap_before_admission, 0u);
    BOOST_CHECK_EQUAL(ledger.invalid_gap_after_admission, 0u);
    BOOST_CHECK_EQUAL(ledger.processing_failed, 0u);
    BOOST_CHECK_EQUAL(ledger.cancelled_windows, 0u);

    const std::vector<WindowResult>& results = pipeline.results();
    BOOST_REQUIRE_EQUAL(results.size(), kWindows);

    std::vector<size_t> per_worker(static_cast<size_t>(kWorkers), 0);
    for (size_t i = 0; i < results.size(); ++i) {
        // Strictly ascending window_id proves the reorder buffer.
        BOOST_CHECK_EQUAL(results[i].window_id, i);
        BOOST_CHECK(results[i].worker_id >= 0 && results[i].worker_id < kWorkers);
        if (results[i].worker_id >= 0 && results[i].worker_id < kWorkers) {
            per_worker[static_cast<size_t>(results[i].worker_id)]++;
        }
    }

    size_t worker_sum = 0;
    for (int w = 0; w < kWorkers; ++w) {
        BOOST_TEST_MESSAGE("dsp worker " << w << " processed " << per_worker[w]
                           << " of " << kWindows << " windows");
        // Round-robin-with-fallback dispatch: only participation is contractual,
        // an exact 4/4/4/4 split is not.
        BOOST_CHECK_GE(per_worker[static_cast<size_t>(w)], 1u);
        worker_sum += per_worker[static_cast<size_t>(w)];
    }
    BOOST_CHECK_EQUAL(worker_sum, kWindows);

    std::remove(tmp_file.c_str());
}

// ---------------------------------------------------------------------------
// Test Case 10: test_pipeline_multicore_stop_and_backpressure
//   C6 (ledger conservation under drops), C7 (bounded, non-blocking dispatch)
//   and C8 (bounded shutdown): a deliberately tiny 2-slot pool with 4 workers
//   must drop instead of back-pressuring, must conserve the ledger and must
//   still terminate every admitted window exactly once.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pipeline_multicore_stop_and_backpressure)
{
    const size_t num_samples = 200000;

    const std::string tmp_file =
        write_multicore_sc16_file("stop", make_multicore_sc16_data(num_samples));

    PipelineConfig cfg;
    cfg.source_type = StreamSourceType::Replay;
    cfg.replay_file = tmp_file;
    cfg.paced_replay = false;
    cfg.native_rate = 998.4e6;
    cfg.work_rate = 998.4e6;
    cfg.schedule_t0 = 0.0;
    cfg.period_s = 1000.0 / 998.4e6;
    cfg.pre_guard_samples = 100;
    cfg.capture_samples = 800;
    cfg.post_guard_samples = 100;
    cfg.pool_capacity = 2; // tiny: forces drops / admission backpressure
    cfg.consumer_mode = ConsumerMode::SicOnly;
    cfg.dsp_workers = 4;
    cfg.max_windows = 200;

    UhdSc16Pipeline pipeline(cfg);
    BOOST_REQUIRE(pipeline.initialize());
    pipeline.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    const auto t_stop_start = std::chrono::steady_clock::now();
    pipeline.stop();
    pipeline.wait();
    const auto t_stop_end = std::chrono::steady_clock::now();
    const double stop_elapsed_ms =
        std::chrono::duration<double, std::milli>(t_stop_end - t_stop_start).count();

    BOOST_CHECK_LE(stop_elapsed_ms, 2000.0);

    const PipelineLedger& ledger = pipeline.ledger();
    std::string err;
    BOOST_CHECK_MESSAGE(ledger.verify_invariants(&err), err);

    BOOST_CHECK_GT(ledger.offered_windows, 0u);

    // Invariant 1, spelled out for a clearer failure message.
    BOOST_CHECK_EQUAL(ledger.admitted_windows + ledger.dropped_admission +
                          ledger.invalid_gap_before_admission + ledger.cancelled_windows,
                      ledger.offered_windows);
    // Invariant 2, spelled out for a clearer failure message.
    BOOST_CHECK_EQUAL(ledger.completed_windows + ledger.processing_failed +
                          ledger.invalid_gap_after_admission,
                      ledger.admitted_windows);

    BOOST_TEST_MESSAGE("ledger: offered=" << ledger.offered_windows
                       << " admitted=" << ledger.admitted_windows
                       << " completed=" << ledger.completed_windows
                       << " dropped=" << ledger.dropped_admission
                       << " invalid_before=" << ledger.invalid_gap_before_admission
                       << " invalid_after=" << ledger.invalid_gap_after_admission
                       << " cancelled=" << ledger.cancelled_windows);

    // Every offered window that was not cancelled must yield exactly one result
    // (dev plan 1.3: DroppedAdmission is committed through the same reorder
    // buffer, so drops are present in results() as well).
    const uint64_t expected_results = ledger.admitted_windows +
                                      ledger.dropped_admission +
                                      ledger.invalid_gap_before_admission;
    const std::vector<WindowResult>& results = pipeline.results();
    BOOST_CHECK_EQUAL(results.size(), static_cast<size_t>(expected_results));
    BOOST_CHECK_GE(results.size(), static_cast<size_t>(ledger.admitted_windows));

    // Exactly one result per admitted window: no loss, no duplication, and the
    // committed ids stay strictly increasing (C5) even with interleaved drops.
    size_t admitted_results = 0;
    for (size_t i = 0; i < results.size(); ++i) {
        if (results[i].state != WindowTerminalState::DroppedAdmission) {
            admitted_results++;
            BOOST_CHECK(results[i].worker_id >= 0 && results[i].worker_id < 4);
        }
        if (i > 0) {
            BOOST_CHECK_LT(results[i - 1].window_id, results[i].window_id);
        }
    }
    BOOST_CHECK_EQUAL(admitted_results, static_cast<size_t>(ledger.admitted_windows));

    std::remove(tmp_file.c_str());
}

// ---------------------------------------------------------------------------
// Test Case 11: test_pipeline_multicore_affinity_validation
//   C4/1.4 affinity rule: a cpu_cores_dsp list whose length differs from
//   dsp_workers must fail closed instead of silently degrading.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_pipeline_multicore_affinity_validation)
{
    const std::string tmp_file =
        write_multicore_sc16_file("affinity", make_multicore_sc16_data(4096));

    PipelineConfig cfg;
    cfg.source_type = StreamSourceType::Replay;
    cfg.replay_file = tmp_file;
    cfg.paced_replay = false;
    cfg.native_rate = 998.4e6;
    cfg.work_rate = 998.4e6;
    cfg.schedule_t0 = 0.0;
    cfg.period_s = 1000.0 / 998.4e6;
    cfg.pre_guard_samples = 0;
    cfg.capture_samples = 1000;
    cfg.post_guard_samples = 0;
    cfg.pool_capacity = 2;
    cfg.consumer_mode = ConsumerMode::SicOnly;
    cfg.dsp_workers = 4;
    cfg.cpu_cores_dsp = { 1, 2 };

    // Mismatched affinity list: initialize() must refuse (fail-closed).
    {
        UhdSc16Pipeline pipeline(cfg);
        BOOST_CHECK_MESSAGE(!pipeline.initialize(),
                            "initialize() must fail closed when cpu_cores_dsp.size() (2) "
                            "!= dsp_workers (4)");
    }

    // Matching affinity list: initialize() must accept.  The pipeline copies the
    // config in its constructor, so a fresh instance is required here.
    PipelineConfig good_cfg = cfg;
    good_cfg.cpu_cores_dsp = { 1, 2, 3, 4 };
    {
        UhdSc16Pipeline pipeline(good_cfg);
        BOOST_CHECK_MESSAGE(pipeline.initialize(),
                            "initialize() must accept cpu_cores_dsp with exactly "
                            "dsp_workers entries");
        // No start(): the destructor must tear down a never-started pipeline.
    }

    std::remove(tmp_file.c_str());
}

BOOST_AUTO_TEST_SUITE_END()
