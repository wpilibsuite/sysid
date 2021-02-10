// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <tuple>

namespace sysid {

using FeedforwardGains = std::tuple<double, double, double>;
struct FeedbackControllerPreset;

/**
 * Represents parameters used to calculate optimal feedback gains using a
 * linear-quadratic regulator (LQR).
 */
struct LQRParameters {
  /** The maximum allowable deviation in position. */
  double qp;

  /** The maximum allowable deviation in velocity. */
  double qv;

  /** The maximum allowable control effort */
  double r;
};

/**
 * Calculates position feedback gains for the given controller preset, LQR
 * controller gain parameters and feedforward gains.
 *
 * @param preset           The feedback controller preset.
 * @param params           The parameters for calculating optimal feedback
 *                         gains.
 * @param feedforwardGains The feedforward gains for the system.
 */
std::tuple<double, double> CalculatePositionFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains);

/**
 * Calculates velocity feedback gains for the given controller preset, LQR
 * controller gain parameters and feedforward gains.
 *
 * @param preset           The feedback controller preset.
 * @param params           The parameters for calculating optimal feedback
 *                         gains.
 * @param feedforwardGains The feedforward gains for the system.
 */
std::tuple<double, double> CalculateVelocityFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains);
}  // namespace sysid
