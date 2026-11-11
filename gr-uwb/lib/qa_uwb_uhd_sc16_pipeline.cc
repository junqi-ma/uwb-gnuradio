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

    const auto taps = load_taps_data("resampler_65_48/taps_quality_minorder.txt");
    BOOST_REQUIRE(!taps.empty());

    const std::vector<size_t> test_sizes = {
        32, 64, 128, 4000, 56000
    };

    const std::vector<float> scales = {
        1.0f / 32768.0f,
        1.0f
    };

    for (float scale : scales) {
        for (size_t n : test_sizes) {
            std::vector<std::complex<int16_t>> in_sc16(n);
            for (size_t i = 0; i < n; ++i) {
                int16_t re = static_cast<int16_t>(std::sin(2.0 * M_PI * i / 37.0) * 30000.0);
                int16_t im = static_cast<int16_t>(std::cos(2.0 * M_PI * i / 43.0) * 30000.0);
                in_sc16[i] = std::complex<int16_t>(re, im);
            }

            // Reference step-by-step path: sc16_to_cf32 + process
            std::vector<std::complex<float>> cf32_input(n);
            sc16_to_cf32(in_sc16.data(), cf32_input.data(), n, scale);

            RationalResampler65_48Core resamp_ref(taps.data(), taps.size());
            const size_t max_out = (n * 65 + 47) / 48 + 200;
            std::vector<std::complex<float>> out_ref(max_out);
            auto res_ref = resamp_ref.process(cf32_input.data(), n, out_ref.data(), max_out);

            // Fused path: process_sc16
            RationalResampler65_48Core resamp_fused(taps.data(), taps.size());
            std::vector<std::complex<float>> out_fused(max_out);
            auto res_fused = resamp_fused.process_sc16(in_sc16.data(), n, scale, out_fused.data(), max_out);

            BOOST_REQUIRE_EQUAL(res_ref.produced, res_fused.produced);
            BOOST_REQUIRE_EQUAL(res_ref.consumed, res_fused.consumed);

            float max_abs_diff = 0.0f;
            for (size_t i = 0; i < res_ref.produced; ++i) {
                float diff_re = std::abs(out_ref[i].real() - out_fused[i].real());
                float diff_im = std::abs(out_ref[i].imag() - out_fused[i].imag());
                float diff = std::max(diff_re, diff_im);
                if (diff > max_abs_diff) {
                    max_abs_diff = diff;
                }
            }
            BOOST_CHECK_MESSAGE(max_abs_diff < 1e-6f,
                                "Precision test failed for n=" + std::to_string(n) +
                                " scale=" + std::to_string(scale) +
                                " max_abs_diff=" + std::to_string(max_abs_diff));
        }
    }
}

// ---------------------------------------------------------------------------
// Test Case 7: test_resampler_process_sc16_chunking_and_streaming
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_resampler_process_sc16_chunking_and_streaming)
{
    using gr::uwb::core::RationalResampler65_48Core;

    const auto taps = load_taps_data("resampler_65_48/taps_quality_minorder.txt");
    BOOST_REQUIRE(!taps.empty());

    const size_t total_samples = 10000;
    const float scale = 1.0f / 32768.0f;

    std::vector<std::complex<int16_t>> in_sc16(total_samples);
    for (size_t i = 0; i < total_samples; ++i) {
        int16_t re = static_cast<int16_t>(std::sin(2.0 * M_PI * i / 23.0) * 28000.0);
        int16_t im = static_cast<int16_t>(std::cos(2.0 * M_PI * i / 29.0) * 28000.0);
        in_sc16[i] = std::complex<int16_t>(re, im);
    }

    // 1. One-shot execution as gold reference
    RationalResampler65_48Core resamp_oneshot(taps.data(), taps.size());
    const size_t max_out_total = (total_samples * 65 + 47) / 48 + 500;
    std::vector<std::complex<float>> out_oneshot(max_out_total);
    auto res_oneshot = resamp_oneshot.process_sc16(in_sc16.data(), total_samples, scale,
                                                  out_oneshot.data(), max_out_total);
    std::vector<std::complex<float>> flush_oneshot(500);
    size_t flush_oneshot_n = resamp_oneshot.flush(flush_oneshot.data(), flush_oneshot.size());

    // 2. Chunked streaming execution via process_sc16
    RationalResampler65_48Core resamp_chunked(taps.data(), taps.size());
    std::vector<std::complex<float>> out_chunked;
    out_chunked.reserve(max_out_total);

    const std::vector<size_t> chunk_sizes = {128, 256, 512, 1024, 384, 768, 64, 2048, 1500};
    size_t chunk_idx = 0;
    size_t offset = 0;
    size_t total_consumed_chunked = 0;

    while (offset < total_samples) {
        size_t c_size = std::min(chunk_sizes[chunk_idx % chunk_sizes.size()], total_samples - offset);
        chunk_idx++;

        size_t chunk_max_out = (c_size * 65 + 47) / 48 + 100;
        std::vector<std::complex<float>> chunk_out(chunk_max_out);

        auto res = resamp_chunked.process_sc16(in_sc16.data() + offset, c_size, scale,
                                              chunk_out.data(), chunk_max_out);

        out_chunked.insert(out_chunked.end(), chunk_out.begin(), chunk_out.begin() + res.produced);
        offset += res.consumed;
        total_consumed_chunked += res.consumed;
    }

    std::vector<std::complex<float>> flush_chunked(500);
    size_t flush_chunked_n = resamp_chunked.flush(flush_chunked.data(), flush_chunked.size());

    // Verify produced & consumed totals
    BOOST_CHECK_EQUAL(total_consumed_chunked, res_oneshot.consumed);
    BOOST_CHECK_EQUAL(out_chunked.size(), res_oneshot.produced);
    BOOST_CHECK_EQUAL(flush_chunked_n, flush_oneshot_n);

    // Verify sample-by-sample equivalence between chunked streaming and one-shot
    float max_diff = 0.0f;
    for (size_t i = 0; i < out_chunked.size(); ++i) {
        float diff_re = std::abs(out_chunked[i].real() - out_oneshot[i].real());
        float diff_im = std::abs(out_chunked[i].imag() - out_oneshot[i].imag());
        float diff = std::max(diff_re, diff_im);
        if (diff > max_diff) {
            max_diff = diff;
        }
    }
    BOOST_CHECK_MESSAGE(max_diff < 1e-6f,
                        "Chunked streaming output differs from one-shot: max_diff=" +
                        std::to_string(max_diff));

    // Verify flushed samples equivalence
    float flush_diff = 0.0f;
    for (size_t i = 0; i < flush_chunked_n; ++i) {
        float diff_re = std::abs(flush_chunked[i].real() - flush_oneshot[i].real());
        float diff_im = std::abs(flush_chunked[i].imag() - flush_oneshot[i].imag());
        float diff = std::max(diff_re, diff_im);
        if (diff > flush_diff) {
            flush_diff = diff;
        }
    }
    BOOST_CHECK_MESSAGE(flush_diff < 1e-6f,
                        "Chunked flush output differs from one-shot: flush_diff=" +
                        std::to_string(flush_diff));
}

BOOST_AUTO_TEST_SUITE_END()
