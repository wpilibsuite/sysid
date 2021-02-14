// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace Constants {

namespace Drivetrain {

// Ports
constexpr int kLeftLeaderPort = 1;
constexpr int kLeftFollowerPort = 2;
constexpr int kRightLeaderPort = 3;
constexpr int kRightFollowerPort = 4;

constexpr int kLeftEncoderPorts[2] = {0, 1};
constexpr int kRightEncoderPorts[2] = {2, 3};

constexpr int kGyroPort = 0;

// Measurement Constants
constexpr units::meter_t kTrackWidth = 0.381_m * 2;
constexpr double kWheelRadius = 0.0508;  // meters
constexpr int kEncoderResolution = 4096;

// Feedforward Gains
constexpr auto kV = 1.98_V / 1_mps;
constexpr auto kA = 0.2_V / 1_mps_sq;
constexpr auto kAngularKV = 1.5_V / 1_rad_per_s;
constexpr auto kAngularKA = 0.3_V / 1_rad_per_s_sq;

}  // namespace Drivetrain

namespace SimpleMotor {
constexpr int kLeaderPort = 5;
constexpr int kFollowerPort = 6;

constexpr int kEncoderPorts[2] = {4, 5};

constexpr auto kV = 1.98_V / 1_rad_per_s;
constexpr auto kA = 0.2_V / 1_rad_per_s_sq;

}  // namespace SimpleMotor

}  // namespace Constants
