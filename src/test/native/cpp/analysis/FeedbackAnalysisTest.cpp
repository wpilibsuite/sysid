// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "gtest/gtest.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"

TEST(FeedbackAnalysis, Velocity1) {
  auto Ks = 1.01;
  auto Kv = 3.060;
  auto Ka = 0.327;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kDefault, params, {Ks, Kv, Ka});

  EXPECT_NEAR(Kp, 2.11, 0.05);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysis, Velocity2) {
  auto Ks = 0.547;
  auto Kv = 0.0693;
  auto Ka = 0.1170;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kDefault, params, {Ks, Kv, Ka});

  EXPECT_NEAR(Kp, 3.11, 0.05);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}
