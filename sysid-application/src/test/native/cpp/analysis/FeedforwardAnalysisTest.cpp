// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <gtest/gtest.h>
#include <units/time.h>
#include <units/voltage.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/ArmSim.h"
#include "sysid/analysis/ElevatorSim.h"
#include "sysid/analysis/FeedforwardAnalysis.h"
#include "sysid/analysis/SimpleMotorSim.h"

/**
 * Return simulated test data for a given simulation model.
 *
 * @param Ks Static friction gain.
 * @param Kv Velocity gain.
 * @param Ka Acceleration gain.
 * @param Kg Gravity cosine gain.
 */
template <typename Model>
sysid::Storage CollectData(Model& model) {
  constexpr auto kUstep = 0.25_V;
  constexpr units::second_t T = 5_ms;
  constexpr units::second_t kTestDuration = 5_s;

  sysid::Storage storage;
  auto& [forward, backward] = storage;

  // Slow forward test
  auto voltage = kUstep;
  bool steadyState = false;
  units::second_t steadyStateTimestamp = 0_s;
  for (int i = 0; i < (kTestDuration / T).value(); ++i) {
    auto accel = model.GetAcceleration(voltage);
    forward.emplace_back(sysid::PreparedData{
        i * T, voltage.value(), model.GetPosition(), model.GetVelocity(), T,
        accel, std::cos(model.GetPosition()), std::sin(model.GetPosition())});

    model.Update(voltage, T);
    if (accel <= 0.5 && !steadyState) {
      steadyState = true;
      steadyStateTimestamp = i * T;
    } else if (steadyState) {
      if ((i * T - steadyStateTimestamp) >= 0.5_s) {
        voltage += kUstep;
        steadyState = false;
      }
    }
  }

  // Slow backward test
  model.Reset();
  voltage = -kUstep;
  steadyState = false;
  steadyStateTimestamp = 0_s;
  for (int i = 0; i < (kTestDuration / T).value(); ++i) {
    auto accel = model.GetAcceleration(voltage);
    backward.emplace_back(sysid::PreparedData{
        i * T, voltage.value(), model.GetPosition(), model.GetVelocity(), T,
        accel, std::cos(model.GetPosition()), std::sin(model.GetPosition())});

    model.Update(voltage, T);
    if (accel <= 0.5 && !steadyState) {
      steadyState = true;
      steadyStateTimestamp = i * T;
    } else if (steadyState) {
      if (i * T - steadyStateTimestamp >= 1_s) {
        voltage -= kUstep;
        steadyState = false;
      }
    }
  }

  return storage;
}

TEST(FeedforwardAnalysisTest, Arm1) {
  constexpr double Ks = 1.01;
  constexpr double Kv = 3.060;
  constexpr double Ka = 0.327;
  constexpr double Kg = 0.211;

  for (const auto& offset : {-2.0, -1.0, 0.0, 1.0, 2.0}) {
    sysid::ArmSim model{Ks, Kv, Ka, Kg, offset};
    auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                               sysid::analysis::kArm);
    auto& gains = std::get<0>(ff);

    EXPECT_NEAR(gains[0], Ks, 0.004);
    EXPECT_NEAR(gains[1], Kv, 0.04);
    EXPECT_NEAR(gains[2], Ka, 0.005);
    EXPECT_NEAR(gains[3], Kg, 0.02);
    EXPECT_NEAR(gains[4], offset, 0.06);
  }
}

TEST(FeedforwardAnalysisTest, Arm2) {
  constexpr double Ks = 0.547;
  constexpr double Kv = 0.0693;
  constexpr double Ka = 0.1170;
  constexpr double Kg = 0.122;

  for (const auto& offset : {-2.0, -1.0, 0.0, 1.0, 2.0}) {
    sysid::ArmSim model{Ks, Kv, Ka, Kg, offset};
    auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                               sysid::analysis::kArm);
    auto& gains = std::get<0>(ff);

    EXPECT_NEAR(gains[0], Ks, 0.003);
    EXPECT_NEAR(gains[1], Kv, 0.003);
    EXPECT_NEAR(gains[2], Ka, 0.003);
    EXPECT_NEAR(gains[3], Kg, 0.003);
    EXPECT_NEAR(gains[4], offset, 0.01);
  }
}

TEST(FeedforwardAnalysisTest, Drivetrain1) {
  constexpr double Ks = 1.01;
  constexpr double Kv = 3.060;
  constexpr double Ka = 0.327;

  sysid::SimpleMotorSim model{Ks, Kv, Ka};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kDrivetrain);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
}

TEST(FeedforwardAnalysisTest, Drivetrain2) {
  constexpr double Ks = 0.547;
  constexpr double Kv = 0.0693;
  constexpr double Ka = 0.1170;

  sysid::SimpleMotorSim model{Ks, Kv, Ka};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kDrivetrain);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
}

TEST(FeedforwardAnalysisTest, DrivetrainAngular1) {
  constexpr double Ks = 1.01;
  constexpr double Kv = 3.060;
  constexpr double Ka = 0.327;

  sysid::SimpleMotorSim model{Ks, Kv, Ka};
  auto ff = sysid::CalculateFeedforwardGains(
      CollectData(model), sysid::analysis::kDrivetrainAngular);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
}

TEST(FeedforwardAnalysisTest, DrivetrainAngular2) {
  constexpr double Ks = 0.547;
  constexpr double Kv = 0.0693;
  constexpr double Ka = 0.1170;

  sysid::SimpleMotorSim model{Ks, Kv, Ka};
  auto ff = sysid::CalculateFeedforwardGains(
      CollectData(model), sysid::analysis::kDrivetrainAngular);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
}

TEST(FeedforwardAnalysisTest, Elevator1) {
  constexpr double Ks = 1.01;
  constexpr double Kv = 3.060;
  constexpr double Ka = 0.327;
  constexpr double Kg = -0.211;

  sysid::ElevatorSim model{Ks, Kv, Ka, Kg};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kElevator);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.008);
  EXPECT_NEAR(gains[2], Ka, 0.003);
  EXPECT_NEAR(gains[3], Kg, 0.003);
}

TEST(FeedforwardAnalysisTest, Elevator2) {
  constexpr double Ks = 0.547;
  constexpr double Kv = 0.0693;
  constexpr double Ka = 0.1170;
  constexpr double Kg = -0.122;

  sysid::ElevatorSim model{Ks, Kv, Ka, Kg};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kElevator);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
  EXPECT_NEAR(gains[3], Kg, 0.003);
}

TEST(FeedforwardAnalysisTest, Simple1) {
  constexpr double Ks = 1.01;
  constexpr double Kv = 3.060;
  constexpr double Ka = 0.327;

  sysid::SimpleMotorSim model{Ks, Kv, Ka};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kSimple);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
}

TEST(FeedforwardAnalysisTest, Simple2) {
  constexpr double Ks = 0.547;
  constexpr double Kv = 0.0693;
  constexpr double Ka = 0.1170;

  sysid::SimpleMotorSim model{Ks, Kv, Ka};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kSimple);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
}
