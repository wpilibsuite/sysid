// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <units/time.h>
#include <units/voltage.h>

#include "gtest/gtest.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AngularDriveSim.h"
#include "sysid/analysis/ArmSim.h"
#include "sysid/analysis/ElevatorSim.h"
#include "sysid/analysis/FeedforwardAnalysis.h"
#include "sysid/analysis/SimpleMotorSim.h"

/**
 * Return simulated test data for a given simulation model.
 *
 * @param Ks   Static friction gain.
 * @param Kv   Velocity gain.
 * @param Ka   Acceleration gain.
 * @param Kcos Gravity cosine gain.
 */
template <typename Model>
sysid::Storage CollectData(Model& model) {
  constexpr auto kUstep = 0.25_V / 1_s;
  constexpr units::volt_t kUmax = 7_V;
  constexpr units::second_t T = 5_ms;
  constexpr units::second_t kTestDuration = 5_s;

  sysid::Storage storage;
  auto& [slow, fast] = storage;

  // Slow forward test
  auto voltage = 0_V;
  for (int i = 0; i < (kTestDuration / T).to<double>(); ++i) {
    slow.emplace_back(sysid::PreparedData{
        i * T, voltage.to<double>(), model.GetPosition(), model.GetVelocity(),
        0.0, T, model.GetAcceleration(voltage), std::cos(model.GetPosition())});

    model.Update(voltage, T);
    slow.back().nextVelocity = model.GetVelocity();
    voltage += kUstep * T;
  }

  // Slow backward test
  model.Reset();
  voltage = 0_V;
  for (int i = 0; i < (kTestDuration / T).to<double>(); ++i) {
    slow.emplace_back(sysid::PreparedData{
        i * T, voltage.to<double>(), model.GetPosition(), model.GetVelocity(),
        0.0, T, model.GetAcceleration(voltage), std::cos(model.GetPosition())});

    model.Update(voltage, T);
    slow.back().nextVelocity = model.GetVelocity();
    voltage -= kUstep * T;
  }

  // Fast forward test
  model.Reset();
  voltage = 0_V;
  for (int i = 0; i < (kTestDuration / T).to<double>(); ++i) {
    fast.emplace_back(sysid::PreparedData{
        i * T, voltage.to<double>(), model.GetPosition(), model.GetVelocity(),
        0.0, T, model.GetAcceleration(voltage), std::cos(model.GetPosition())});

    model.Update(voltage, T);
    fast.back().nextVelocity = model.GetVelocity();
    voltage = kUmax;
  }

  // Fast backward test
  model.Reset();
  voltage = 0_V;
  for (int i = 0; i < (kTestDuration / T).to<double>(); ++i) {
    fast.emplace_back(sysid::PreparedData{
        i * T, voltage.to<double>(), model.GetPosition(), model.GetVelocity(),
        0.0, T, model.GetAcceleration(voltage), std::cos(model.GetPosition())});

    model.Update(voltage, T);
    fast.back().nextVelocity = model.GetVelocity();
    voltage = -kUmax;
  }

  return storage;
}

TEST(FeedforwardAnalysisTest, Arm1) {
  constexpr double Ks = 1.01;
  constexpr double Kv = 3.060;
  constexpr double Ka = 0.327;
  constexpr double Kcos = -0.211;

  sysid::ArmSim model{Ks, Kv, Ka, Kcos};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kArm);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
  EXPECT_NEAR(gains[3], Kcos, 0.003);
}

TEST(FeedforwardAnalysisTest, Arm2) {
  constexpr double Ks = 0.547;
  constexpr double Kv = 0.0693;
  constexpr double Ka = 0.1170;
  constexpr double Kcos = -0.122;

  sysid::ArmSim model{Ks, Kv, Ka, Kcos};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kArm);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
  EXPECT_NEAR(gains[3], Kcos, 0.003);
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

TEST(FeedforwardAnalysisTest, AngularDrivetrain1) {
  constexpr double Ks = 1.01;
  constexpr double Kv = 3.060;
  constexpr double Ka = 0.327;
  constexpr double KtrackWidth = 0.762;

  sysid::AngularDriveSim model{Ks, Kv, Ka, KtrackWidth};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kDrivetrain);
  auto& gains = std::get<0>(ff);

  EXPECT_NEAR(gains[0], Ks, 0.003);
  EXPECT_NEAR(gains[1], Kv, 0.003);
  EXPECT_NEAR(gains[2], Ka, 0.003);
}

TEST(FeedforwardAnalysisTest, AngularDrivetrain2) {
  constexpr double Ks = 0.547;
  constexpr double Kv = 0.0693;
  constexpr double Ka = 0.1170;
  constexpr double KtrackWidth = 0.643;

  sysid::AngularDriveSim model{Ks, Kv, Ka, KtrackWidth};
  auto ff = sysid::CalculateFeedforwardGains(CollectData(model),
                                             sysid::analysis::kDrivetrain);
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
  EXPECT_NEAR(gains[1], Kv, 0.003);
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
