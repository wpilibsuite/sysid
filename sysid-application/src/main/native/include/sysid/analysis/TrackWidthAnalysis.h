// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>

namespace sysid {
/**
 * Calculates the track width given the left distance, right distance, and
 * accumulated gyro angle.
 *
 * @param l     The distance traveled by the left side of the drivetrain.
 * @param r     The distance traveled by the right side of the drivetrain.
 * @param accum The accumulated gyro angle.
 */
double CalculateTrackWidth(double l, double r, units::radian_t accum);
}  // namespace sysid
