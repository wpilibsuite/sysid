// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <array>
#include <vector>

#include <wpi/raw_ostream.h>

#include "gtest/gtest.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/FeedforwardAnalysis.h"
#include "sysid/analysis/FilteringUtils.h"

TEST(FilterTest, MedianFilter) {
  std::vector<std::array<double, 1>> testData = {{0}, {1},    {10}, {5}, {3},
                                                 {0}, {1000}, {7},  {6}, {5}};
  std::vector<std::array<double, 1>> expectedData = {{1}, {5}, {5}, {3},
                                                     {3}, {7}, {7}, {6}};

  sysid::ApplyMedianFilter<1, 0>(&testData, 3);
  EXPECT_EQ(expectedData, testData);
}

TEST(FilterTest, QuasistaticTrim) {
  std::vector<std::array<double, 2>> testData = {
      {0, 1}, {.5, 2}, {2, 0.1}, {4, 4}, {0, 5}};
  std::vector<std::array<double, 2>> expectedData = {{.5, 2}, {4, 4}};
  sysid::TrimQuasistaticData<2, 0, 1>(&testData, 0.2);
  EXPECT_EQ(expectedData, testData);
}

TEST(FilterTests, AccelNoiseFloor) {
  std::vector<sysid::PreparedData> testData = {
      {0_s, 1, 2, 3, 3, 5_ms, 0, 0},    {1_s, 1, 2, 3, 3, 5_ms, 1, 0},
      {2_s, 1, 2, 3, 3, 5_ms, 2, 0},    {3_s, 1, 2, 3, 3, 5_ms, 5, 0},
      {4_s, 1, 2, 3, 3, 5_ms, 0.35, 0}, {5_s, 1, 2, 3, 3, 5_ms, 0.15, 0},
      {6_s, 1, 2, 3, 3, 5_ms, 0, 0},    {7_s, 1, 2, 3, 3, 5_ms, 0.02, 0},
      {8_s, 1, 2, 3, 3, 5_ms, 0.01, 0}, {9_s, 1, 2, 3, 3, 5_ms, 0, 0}};
  double noiseFloor = GetAccelNoiseFloor(testData, 2);
  EXPECT_NEAR(0.953, noiseFloor, 0.001);
}

TEST(FilterTest, StepTrim) {
  std::vector<sysid::PreparedData> testData = {
      {0_s, 1, 2, 3, 3, 5_ms, 0, 0},    {1_s, 1, 2, 3, 3, 5_ms, 0.25, 0},
      {2_s, 1, 2, 3, 3, 5_ms, 0.5, 0},  {3_s, 1, 2, 3, 3, 5_ms, 0.45, 0},
      {4_s, 1, 2, 3, 3, 5_ms, 0.35, 0}, {5_s, 1, 2, 3, 3, 5_ms, 0.15, 0},
      {6_s, 1, 2, 3, 3, 5_ms, 0, 0},    {7_s, 1, 2, 3, 3, 5_ms, 0.02, 0},
      {8_s, 1, 2, 3, 3, 5_ms, 0.01, 0}, {9_s, 1, 2, 3, 3, 5_ms, 0, 0},
  };

  std::vector<sysid::PreparedData> expectedData = {
      {2_s, 1, 2, 3, 3, 5_ms, 0.5, 0},
      {3_s, 1, 2, 3, 3, 5_ms, 0.45, 0},
      {4_s, 1, 2, 3, 3, 5_ms, 0.35, 0},
      {5_s, 1, 2, 3, 3, 5_ms, 0.15, 0}};

  auto maxTime = 9_s;
  auto minTime = maxTime;

  sysid::AnalysisManager::Settings settings;
  minTime = sysid::TrimStepVoltageData(&testData, &settings, minTime, maxTime);

  EXPECT_EQ(expectedData[0].acceleration, testData[0].acceleration);
  EXPECT_EQ(expectedData.back().acceleration, testData.back().acceleration);
  EXPECT_EQ(5, settings.stepTestDuration.to<double>());
  EXPECT_EQ(2, minTime.to<double>());
}
