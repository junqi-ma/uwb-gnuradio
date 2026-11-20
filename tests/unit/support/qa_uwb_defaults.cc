/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Checks the native production sample rates and window defaults.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_defaults.h>
#include <gnuradio/uwb/uwb_radar_meta_core.h>

#include <cmath>

BOOST_AUTO_TEST_CASE(test_defaults_production_values)
{
    // Keep the numeric production contract explicit in one place.
    BOOST_CHECK_CLOSE(openuwb::defaults::kSampleRateHz, 998400000.0, 1e-12);
    BOOST_CHECK_EQUAL(openuwb::defaults::kDetectorPreTrigger, size_t(2032));
    BOOST_CHECK_EQUAL(openuwb::defaults::kDetectorCapture, size_t(200000));
    BOOST_CHECK_CLOSE(openuwb::defaults::kDetectorEnergyThreshold, 0.02f, 1e-6);
    BOOST_CHECK_CLOSE(openuwb::defaults::kDetectorSc16EnergyThreshold, 4e-3f,
                      1e-6);
    BOOST_CHECK_EQUAL(openuwb::defaults::kDetectorEnergyGateDecimation,
                      size_t(100));
    BOOST_CHECK_EQUAL(openuwb::defaults::kDetectorCoarseDecimation, size_t(4));
    BOOST_CHECK_EQUAL(openuwb::defaults::kDetectorCoarseRepetitions, size_t(1));
    BOOST_CHECK_EQUAL(openuwb::defaults::kDetectorCoarseMargin, size_t(16));
    BOOST_CHECK_EQUAL(openuwb::defaults::kScheduledPreGuard, size_t(9984));
    BOOST_CHECK_EQUAL(openuwb::defaults::kScheduledCapture, size_t(189696));
    BOOST_CHECK_EQUAL(openuwb::defaults::kScheduledPostGuard, size_t(4096));
    BOOST_CHECK_EQUAL(openuwb::defaults::kScheduledPoolSize, size_t(8));
    BOOST_CHECK_CLOSE(openuwb::defaults::kScheduledPacketIntervalS, 0.01, 1e-12);
    BOOST_CHECK_CLOSE(openuwb::defaults::kNativeSampleRateHz, 737280000.0, 1e-12);
    BOOST_CHECK_CLOSE(openuwb::defaults::kCg400NativeSampleRateHz,
                      491520000.0, 1e-12);
    BOOST_CHECK_EQUAL(openuwb::defaults::kCg400NativeScheduledPreGuard,
                      size_t(4915));
    BOOST_CHECK_EQUAL(openuwb::defaults::kCg400NativeScheduledCapture,
                      size_t(93389));
    BOOST_CHECK_EQUAL(openuwb::defaults::kCg400NativeScheduledPostGuard,
                      size_t(2015));
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(10e-6 * 491.52e6)),
        openuwb::defaults::kCg400NativeScheduledPreGuard);
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(190e-6 * 491.52e6)),
        openuwb::defaults::kCg400NativeScheduledCapture);
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(4.1e-6 * 491.52e6)),
        openuwb::defaults::kCg400NativeScheduledPostGuard);
    BOOST_CHECK_CLOSE(openuwb::defaults::kQm35PacketIntervalS, 0.005, 1e-12);
    BOOST_CHECK_EQUAL(openuwb::defaults::kMaxRadarBurstSamples, size_t(2097152));
    BOOST_CHECK_EQUAL(openuwb::radar_meta::kMaxRadarBurstSamples,
                      openuwb::defaults::kMaxRadarBurstSamples);
    BOOST_CHECK_EQUAL(openuwb::defaults::kNativeScheduledPreGuard, size_t(7373));
    BOOST_CHECK_EQUAL(openuwb::defaults::kNativeScheduledCapture, size_t(140083));
    BOOST_CHECK_EQUAL(openuwb::defaults::kNativeScheduledPostGuard, size_t(3023));
    BOOST_CHECK_CLOSE(openuwb::defaults::kNativeInterferencePreGuardS,
                      300e-6, 1e-12);
    BOOST_CHECK_CLOSE(openuwb::defaults::kNativeInterferencePostGuardS,
                      100e-6, 1e-12);
    BOOST_CHECK_EQUAL(openuwb::defaults::kNativeInterferencePreGuard,
                      size_t(221184));
    BOOST_CHECK_EQUAL(openuwb::defaults::kNativeInterferencePostGuard,
                      size_t(73728));
    BOOST_CHECK_EQUAL(openuwb::defaults::kLockObservations, size_t(3));
    BOOST_CHECK_EQUAL(openuwb::defaults::kHoldoverMissCount, size_t(3));
    BOOST_CHECK_EQUAL(openuwb::defaults::kReacquireMissCount, size_t(8));
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(10e-6 * 737.28e6)),
        openuwb::defaults::kNativeScheduledPreGuard);
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(190e-6 * 737.28e6)),
        openuwb::defaults::kNativeScheduledCapture);
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(4.1e-6 * 737.28e6)),
        openuwb::defaults::kNativeScheduledPostGuard);
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(
            openuwb::defaults::kNativeInterferencePreGuardS * 737.28e6)),
        openuwb::defaults::kNativeInterferencePreGuard);
    BOOST_CHECK_EQUAL(
        static_cast<size_t>(std::llround(
            openuwb::defaults::kNativeInterferencePostGuardS * 737.28e6)),
        openuwb::defaults::kNativeInterferencePostGuard);
}
