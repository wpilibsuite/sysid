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
  double nextVelocity = 0.0;
  units::second_t dt = 0_s;
  double acceleration = 0.0;
  double cos = 0.0;

  constexpr bool operator==(const PreparedData& rhs) const {
    return timestamp == rhs.timestamp && voltage == rhs.voltage &&
           position == rhs.position && velocity == rhs.velocity &&
           nextVelocity == rhs.nextVelocity && dt == rhs.dt &&
           acceleration == rhs.acceleration && cos == rhs.cos;
  }
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
