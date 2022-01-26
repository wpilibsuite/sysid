// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <array>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/FeedforwardAnalysis.h"
#include "sysid/analysis/FilteringUtils.h"
#include "sysid/analysis/Storage.h"

TEST(FilterTest, MedianFilter) {
  std::vector<sysid::PreparedData> testData{
      sysid::PreparedData{0_s, 0, 0, 0},    sysid::PreparedData{0_s, 0, 0, 1},
      sysid::PreparedData{0_s, 0, 0, 10},   sysid::PreparedData{0_s, 0, 0, 5},
      sysid::PreparedData{0_s, 0, 0, 3},    sysid::PreparedData{0_s, 0, 0, 0},
      sysid::PreparedData{0_s, 0, 0, 1000}, sysid::PreparedData{0_s, 0, 0, 7},
      sysid::PreparedData{0_s, 0, 0, 6},    sysid::PreparedData{0_s, 0, 0, 5}};
  std::vector<sysid::PreparedData> expectedData{
      sysid::PreparedData{0_s, 0, 0, 0}, sysid::PreparedData{0_s, 0, 0, 1},
      sysid::PreparedData{0_s, 0, 0, 5}, sysid::PreparedData{0_s, 0, 0, 5},
      sysid::PreparedData{0_s, 0, 0, 3}, sysid::PreparedData{0_s, 0, 0, 3},
      sysid::PreparedData{0_s, 0, 0, 7}, sysid::PreparedData{0_s, 0, 0, 7},
      sysid::PreparedData{0_s, 0, 0, 6}, sysid::PreparedData{0_s, 0, 0, 5}};

  sysid::ApplyMedianFilter(&testData, 3);
  EXPECT_EQ(expectedData, testData);
}

TEST(FilterTest, AccelNoiseFloor) {
  std::vector<sysid::PreparedData> testData = {
      {0_s, 1, 2, 3, 5_ms, 0, 0},    {1_s, 1, 2, 3, 5_ms, 1, 0},
      {2_s, 1, 2, 3, 5_ms, 2, 0},    {3_s, 1, 2, 3, 5_ms, 5, 0},
      {4_s, 1, 2, 3, 5_ms, 0.35, 0}, {5_s, 1, 2, 3, 5_ms, 0.15, 0},
      {6_s, 1, 2, 3, 5_ms, 0, 0},    {7_s, 1, 2, 3, 5_ms, 0.02, 0},
      {8_s, 1, 2, 3, 5_ms, 0.01, 0}, {9_s, 1, 2, 3, 5_ms, 0, 0}};
  double noiseFloor = GetAccelNoiseFloor(testData, 2);
  EXPECT_NEAR(0.953, noiseFloor, 0.001);
}

TEST(FilterTest, StepTrim) {
  std::vector<sysid::PreparedData> testData = {
      {0_s, 1, 2, 3, 5_ms, 0, 0},    {1_s, 1, 2, 3, 5_ms, 0.25, 0},
      {2_s, 1, 2, 3, 5_ms, 0.5, 0},  {3_s, 1, 2, 3, 5_ms, 0.45, 0},
      {4_s, 1, 2, 3, 5_ms, 0.35, 0}, {5_s, 1, 2, 3, 5_ms, 0.15, 0},
      {6_s, 1, 2, 3, 5_ms, 0, 0},    {7_s, 1, 2, 3, 5_ms, 0.02, 0},
      {8_s, 1, 2, 3, 5_ms, 0.01, 0}, {9_s, 1, 2, 3, 5_ms, 0, 0},
  };

  std::vector<sysid::PreparedData> expectedData = {
      {2_s, 1, 2, 3, 5_ms, 0.5, 0},
      {3_s, 1, 2, 3, 5_ms, 0.45, 0},
      {4_s, 1, 2, 3, 5_ms, 0.35, 0},
      {5_s, 1, 2, 3, 5_ms, 0.15, 0}};

  auto maxTime = 9_s;
  auto minTime = maxTime;

  sysid::AnalysisManager::Settings settings;
  minTime = sysid::TrimStepVoltageData(&testData, &settings, minTime, maxTime);

  EXPECT_EQ(expectedData[0].acceleration, testData[0].acceleration);
  EXPECT_EQ(expectedData.back().acceleration, testData.back().acceleration);
  EXPECT_EQ(5, settings.stepTestDuration.value());
  EXPECT_EQ(2, minTime.value());
}

TEST(FilterTest, CentralFiniteDifference) {
  constexpr double h = 0.05;

  auto CheckResults = [](auto&& f, auto&& dfdx, double h, double min,
                         double max) {
    for (int i = min / h; i < max / h; ++i) {
      // The order of accuracy is O(h^(N - d)) where N is number of stencil
      // points and d is order of derivative
      EXPECT_NEAR(sysid::CentralFiniteDifference<2>(f, i, h), dfdx(i),
                  std::max(std::pow(h, 3 - 1), 1e-7));
      EXPECT_NEAR(sysid::CentralFiniteDifference<4>(f, i, h), dfdx(i),
                  std::max(std::pow(h, 5 - 1), 1e-7));
      EXPECT_NEAR(sysid::CentralFiniteDifference<6>(f, i, h), dfdx(i),
                  std::max(std::pow(h, 7 - 1), 1e-7));
      EXPECT_NEAR(sysid::CentralFiniteDifference<8>(f, i, h), dfdx(i),
                  std::max(std::pow(h, 9 - 1), 1e-7));
    }
  };

  CheckResults(
      [=](int i) {
        // f(x) = x^2
        double x = i * h;
        return x * x;
      },
      [=](int i) {
        // df/dx = 2x
        double x = i * h;
        return 2.0 * x;
      },
      h, -20.0, 20.0);

  CheckResults(
      [=](int i) {
        // f(x) = std::sin(x)
        double x = i * h;
        return std::sin(x);
      },
      [=](int i) {
        // df/dx = std::cos(x)
        double x = i * h;
        return std::cos(x);
      },
      h, -20.0, 20.0);

  CheckResults(
      [=](int i) {
        // f(x) = ln(x)
        double x = i * h;
        return std::log(x);
      },
      [=](int i) {
        // df/dx = 1 / x
        double x = i * h;
        return 1.0 / x;
      },
      h, 1.0, 20.0);
}
