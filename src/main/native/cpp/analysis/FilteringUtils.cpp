// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FilteringUtils.h"

#include <numeric>
#include <vector>

#include <frc/LinearFilter.h>

using namespace sysid;

units::second_t sysid::TrimStepVoltageData(std::vector<PreparedData>* data,
                                           AnalysisManager::Settings* settings,
                                           units::second_t minStepTime,
                                           units::second_t maxStepTime) {
  auto firstTimestamp = data->at(0).timestamp;

  // Trim data before max acceleration
  data->erase(data->begin(),
              std::max_element(
                  data->begin(), data->end(), [](const auto& a, const auto& b) {
                    return std::abs(a.acceleration) < std::abs(b.acceleration);
                  }));

  minStepTime = std::min(data->at(0).timestamp - firstTimestamp, minStepTime);

  // If step duration hasn't been set yet, set calculate a default (find the
  // entry before the acceleration first hits zero)
  if (settings->stepTestDuration <= minStepTime) {
    // Get noise floor
    const double accelNoiseFloor =
        GetAccelNoiseFloor(*data, settings->windowSize);
    // Find latest element with nonzero acceleration
    auto endIt = std::find_if(
        std::reverse_iterator{data->end()},
        std::reverse_iterator{data->begin()}, [&](const PreparedData& entry) {
          return std::abs(entry.acceleration) > accelNoiseFloor;
        });

    // Calculate default duration
    settings->stepTestDuration =
        std::min(endIt->timestamp - data->front().timestamp + minStepTime + 1_s,
                 maxStepTime);
  }

  // Find first entry greater than the step test duration
  auto maxIt =
      std::find_if(data->begin(), data->end(), [&](PreparedData entry) {
        return entry.timestamp - data->front().timestamp + minStepTime >
               settings->stepTestDuration;
      });

  // Trim data beyond desired step test duration
  if (maxIt != data->end()) {
    data->erase(maxIt, data->end());
  }
  return minStepTime;
}

double sysid::GetAccelNoiseFloor(const std::vector<PreparedData>& data,
                                 int window) {
  double sum = 0.0;
  size_t step = window / 2;
  frc::LinearFilter<double> averageFilter =
      frc::LinearFilter<double>::MovingAverage(window);
  for (size_t i = 0; i < data.size(); i++) {
    double mean = averageFilter.Calculate(data[i].acceleration);
    if (i >= step) {
      sum += std::pow(data[i - step].acceleration - mean, 2);
    }
  }
  return std::sqrt(sum / (data.size() - step));
}

units::second_t sysid::GetMeanTimeDelta(const Storage& data) {
  std::vector<units::second_t> dts;

  const auto& [slow, fast] = data;
  for (size_t i = 0; i < slow.size() - 1; ++i) {
    auto dt = slow[i + 1].timestamp - slow[i].timestamp;
    if (dt > 0_s && dt < 500_ms) {
      dts.emplace_back(dt);
    }
  }
  for (size_t i = 0; i < fast.size() - 1; ++i) {
    auto dt = fast[i + 1].timestamp - fast[i].timestamp;
    if (dt > 0_s && dt < 500_ms) {
      dts.emplace_back(dt);
    }
  }

  return std::accumulate(dts.begin(), dts.end(), 0_s) / dts.size();
}
