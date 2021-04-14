// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <units/time.h>

namespace sysid {

/**
 * Represents each data point after it is cleaned and various parameters are
 * calculated.
 */
struct PreparedData {
  units::second_t timestamp;
  double voltage;
  double position;
  double velocity;
  double acceleration;
  double cos;
};

/**
 * Storage used by the analysis manger.
 */
struct Storage {
  // Dataset for slow (aka quasistatic) test
  std::vector<PreparedData> slow;

  // Dataset for fast (aka dynamic) test
  std::vector<PreparedData> fast;
};

}  // namespace sysid
