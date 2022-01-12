// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "gtest/gtest.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"

TEST(FeedbackAnalysisTest, Velocity1) {
  auto Kv = 3.060;
  auto Ka = 0.327;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kDefault, params, Kv, Ka);

  EXPECT_NEAR(Kp, 2.11, 0.05);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysisTest, Velocity2) {
  auto Kv = 0.0693;
  auto Ka = 0.1170;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kDefault, params, Kv, Ka);

  EXPECT_NEAR(Kp, 3.11, 0.05);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysisTest, VelocityConversion) {
  auto Kv = 0.0693;
  auto Ka = 0.1170;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kDefault, params, Kv, Ka, 3.0 * 1023);

  // This should have the same Kp as the test above, but scaled by a factor of 3
  // * 1023.
  EXPECT_NEAR(Kp, 3.11 / (3 * 1023), 0.005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysisTest, VelocityCTRE) {
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kCTRECANCoder, params, Kv, Ka);

  EXPECT_NEAR(Kp, 0.000417, 0.00005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysisTest, VelocityCTREConversion) {
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kCTRECANCoder, params, Kv, Ka, 3.0);

  // This should have the same Kp as the test above, but scaled by a factor
  // of 3.
  EXPECT_NEAR(Kp, 0.000417 / 3, 0.00005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysisTest, VelocityREV) {
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kREVNEOBuiltIn, params, Kv, Ka);

  EXPECT_NEAR(Kp, 0.00241, 0.005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}

TEST(FeedbackAnalysisTest, VelocityREVConversion) {
  auto Kv = 1.97;
  auto Ka = 0.179;

  sysid::LQRParameters params{1, 1.5, 7};

  auto [Kp, Kd] = sysid::CalculateVelocityFeedbackGains(
      sysid::presets::kREVNEOBuiltIn, params, Kv, Ka, 3.0);

  // This should have the same Kp as the test above, but scaled by a factor
  // of 3.
  EXPECT_NEAR(Kp, 0.00241 / 3, 0.005);
  EXPECT_NEAR(Kd, 0.00, 0.05);
}
