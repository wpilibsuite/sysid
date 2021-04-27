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

TEST(FilterTest, QuasistaticTrim) {
  std::vector<sysid::PreparedData> testData{
      {sysid::PreparedData{0_s, 0, 0, 1}, sysid::PreparedData{0_s, .5, 0, 2},
       sysid::PreparedData{0_s, 2, 0, 0.1}, sysid::PreparedData{0_s, 4, 0, 4},
       sysid::PreparedData{0_s, 0, 0, 5}}};
  std::vector<sysid::PreparedData> expectedData{
      sysid::PreparedData{0_s, .5, 0, 2}, sysid::PreparedData{0_s, 4, 0, 4}};
  sysid::TrimQuasistaticData(&testData, 0.2);
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
