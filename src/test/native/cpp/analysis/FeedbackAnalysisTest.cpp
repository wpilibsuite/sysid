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

TEST(FeedbackAnalysis, VelocityConversion) {
  auto Ks = 0.547;
  auto Kv = 0.0693;
  auto Ka = 0.1170;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kDefault, params, {Ks, Kv, Ka}, 3.0 * 1023);

  // This should have the same Kp as the test above, but scaled by a factor of 3
  // * 1023.
  EXPECT_NEAR(Kp, 3.11 / (3 * 1023), 0.005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysis, VelocityCTRE) {
  auto Ks = 0.024;
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kCTRENew, params, {Ks, Kv, Ka});

  EXPECT_NEAR(Kp, 0.25, 0.05);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysis, VelocityCTREConversion) {
  auto Ks = 0.024;
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kCTRENew, params, {Ks, Kv, Ka}, 3.0);

  // This should have the same Kp as the test above, but scaled by a factor
  // of 3.
  EXPECT_NEAR(Kp, 0.25 / 3, 0.05);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysis, VelocityREV) {
  auto Ks = 0.024;
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kREVBrushless, params, {Ks, Kv, Ka});

  EXPECT_NEAR(Kp, 0.00241, 0.005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysis, VelocityREVConversion) {
  auto Ks = 0.024;
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kREVBrushless, params, {Ks, Kv, Ka}, 3.0);

  // This should have the same Kp as the test above, but scaled by a factor
  // of 3.
  EXPECT_NEAR(Kp, 0.00241 / 3, 0.005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}
