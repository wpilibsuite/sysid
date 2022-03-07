// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/time.h>

namespace sysid {
/**
 * Represents a preset for a specific feedback controller. This includes info
 * about the max controller output, the controller period, whether the gains
 * are time-normalized, and whether there are measurement delays from sensors or
 * onboard filtering.
 */
struct FeedbackControllerPreset {
  /**
   * The conversion factor between volts and the final controller output.
   */
  double outputConversionFactor;

  /**
   * The conversion factor for using controller output for velocity gains. This
   * is necessary as some controllers do velocity controls with different time
   * units.
   */
  double outputVelocityTimeFactor;

  /**
   * The period at which the controller runs.
   */
  units::millisecond_t period;

  /**
   * Whether the controller gains are time-normalized.
   */
  bool normalized;

  /**
   * The measurement delay in the encoder measurements.
   */
  units::millisecond_t measurementDelay;

  /**
   * Checks equality between two feedback controller presets.
   *
   * @param rhs Another FeedbackControllerPreset
   * @return If the two presets are equal
   */
  constexpr bool operator==(const FeedbackControllerPreset& rhs) const {
    return outputConversionFactor == rhs.outputConversionFactor &&
           outputVelocityTimeFactor == rhs.outputVelocityTimeFactor &&
           period == rhs.period && normalized == rhs.normalized &&
           measurementDelay == rhs.measurementDelay;
  }

  /**
   * Checks inequality between two feedback controller presets.
   *
   * @param rhs Another FeedbackControllerPreset
   * @return If the two presets are not equal
   */
  constexpr bool operator!=(const FeedbackControllerPreset& rhs) const {
    return !operator==(rhs);
  }
};

/**
 * The loop type for the feedback controller.
 */
enum class FeedbackControllerLoopType { kPosition, kVelocity };

namespace presets {
constexpr FeedbackControllerPreset kDefault{1.0, 1.0, 20_ms, true, 0_s};

constexpr FeedbackControllerPreset kWPILibNew{kDefault};
constexpr FeedbackControllerPreset kWPILibOld{1.0 / 12.0, 1.0, 50_ms, false,
                                              0_s};

// Measurement delay from a moving average filter:
//
// A moving average filter with a window size of N is an FIR filter with N taps.
// The average delay of a moving average filter with N taps and a period between
// samples of T is (N - 1)/2 T.
//
// Proof:
// N taps with delays of 0 .. (N - 1) T
//
// average delay = (sum 0 .. N - 1) / N T
// = (sum 1 .. N - 1) / N T
//
// note: sum 1 .. n = n(n + 1) / 2
//
// = (N - 1)((N - 1) + 1) / (2N) T
// = (N - 1)N / (2N) T
// = (N - 1)/2 T

// Measurement delay from a backward finite difference:
//
// Velocities calculated via a backward finite difference with a period of T
// look like:
//
// velocity = (position at timestep k - position at timestep k-1) / T
//
// The average delay is T / 2.
//
// Proof:
// average delay = (0 ms + T) / 2
//               = T / 2

/**
 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#changing-velocity-measurement-parameters
 *
 * Backward finite difference delay = 100 ms / 2 = 50 ms.
 *
 * 64-tap moving average delay = (64 - 1) / 2 * 1 ms = 31.5 ms.
 *
 * Total delay = 50 ms + 31.5 ms = 81.5 ms.
 */
constexpr FeedbackControllerPreset kCTRECANCoder{1.0 / 12.0, 60.0, 1_ms, true,
                                                 81.5_ms};
constexpr FeedbackControllerPreset kCTREDefault{1023.0 / 12.0, 0.1, 1_ms, false,
                                                81.5_ms};

/**
 * https://github.com/wpilibsuite/sysid/issues/258#issuecomment-1010658237
 *
 * 8-sample moving average with 32 ms between samples.
 *
 * Total delay = 8-tap moving average delay = (8 - 1) / 2 * 32 ms = 112 ms.
 */
constexpr FeedbackControllerPreset kREVNEOBuiltIn{1.0 / 12.0, 60.0, 1_ms, false,
                                                  112_ms};

/**
 * https://www.revrobotics.com/content/sw/max/sw-docs/cpp/classrev_1_1_c_a_n_encoder.html#a7e6ce792bc0c0558fb944771df572e6a
 *
 * Backward finite difference delay = 100 ms / 2 = 50 ms.
 *
 * 64-tap moving average delay = (64 - 1) / 2 * 1 ms = 31.5 ms.
 *
 * Total delay = 50 ms + 31.5 ms = 81.5 ms.
 */
constexpr FeedbackControllerPreset kREVNonNEO{1.0 / 12.0, 60.0, 1_ms, false,
                                              81.5_ms};

/**
 * https://github.com/wpilibsuite/sysid/pull/138#issuecomment-841734229
 *
 * Backward finite difference delay = 10 ms / 2 = 5 ms.
 */
constexpr FeedbackControllerPreset kVenom{4096.0 / 12.0, 60.0, 1_ms, false,
                                          5_ms};
}  // namespace presets
}  // namespace sysid
