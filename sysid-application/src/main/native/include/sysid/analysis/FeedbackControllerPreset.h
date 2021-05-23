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
  /** The conversion factor between volts and the final controller output. */
  double outputConversionFactor;

  /** The conversion factor for using controller output for velocity gains. This
   * is necessary as some controllers do velocity controls with different time
   * units.*/
  double outputVelocityTimeFactor;

  /** The period at which the controller runs. */
  units::second_t period;

  /** Whether the controller gains are time-normalized. */
  bool normalized;

  /** The measurement delay in the position measurements. */
  units::second_t positionMeasurementDelay;

  /** The measurement delay in the velocity measurements. */
  units::second_t velocityMeasurementDelay;

  /** Checks equality between two feedback controller presets. */
  constexpr bool operator==(const FeedbackControllerPreset& rhs) const {
    return outputConversionFactor == rhs.outputConversionFactor &&
           outputVelocityTimeFactor == rhs.outputVelocityTimeFactor &&
           period == rhs.period && normalized == rhs.normalized &&
           positionMeasurementDelay == rhs.positionMeasurementDelay &&
           velocityMeasurementDelay == rhs.velocityMeasurementDelay;
  }

  /** Checks inequality between two feedback controller presets. */
  constexpr bool operator!=(const FeedbackControllerPreset& rhs) const {
    return !operator==(rhs);
  }
};

/** The loop type for the feedback controller */
enum class FeedbackControllerLoopType { kPosition, kVelocity };

namespace presets {
constexpr FeedbackControllerPreset kDefault{1.0, 1.0, 20_ms, true, 0_s, 0_s};

constexpr FeedbackControllerPreset kWPILibNew{kDefault};
constexpr FeedbackControllerPreset kWPILibOld{1.0 / 12.0, 1.0, 50_ms,
                                              false,      0_s, 0_s};

// https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#changing-velocity-measurement-parameters
// 100 ms sampling period + a moving average window size of 64 (i.e. a 64-tap
// FIR) = 100 / 2 ms + (64 - 1) / 2 ms = 81.5 ms.
constexpr FeedbackControllerPreset kCTRENew{1.0 / 12.0, 0.1, 1_ms,
                                            true,       0_s, 81.5_ms};
constexpr FeedbackControllerPreset kCTREOld{1023.0 / 12.0, 0.1, 1_ms,
                                            false,         0_s, 81.5_ms};

// According to a Rev employee on the FRC Discord the window size is 40 so delay
// = (40-1)/2 ms = 19.5 ms.
constexpr FeedbackControllerPreset kREVBrushless{1.0 / 12.0, 60.0, 1_ms,
                                                 false,      0_s,  19.5_ms};

// https://www.revrobotics.com/content/sw/max/sw-docs/cpp/classrev_1_1_c_a_n_encoder.html#a7e6ce792bc0c0558fb944771df572e6a
// 64-tap FIR = (64 - 1) / 2 ms = 31.5 ms delay.
constexpr FeedbackControllerPreset kREVBrushed{1.0 / 12.0, 60.0, 1_ms,
                                               false,      0_s,  31.5_ms};

// https://github.com/wpilibsuite/sysid/pull/138#issuecomment-841734229
// (10 - 0) / 2 = 5ms velocity measurement delay
constexpr FeedbackControllerPreset kVenom{4096.0 / 12.0, 60.0, 1_ms,
                                          false,         0_s,  5_ms};
}  // namespace presets
}  // namespace sysid
