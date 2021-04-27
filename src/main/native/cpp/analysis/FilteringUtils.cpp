// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FilteringUtils.h"

#include <numeric>
#include <vector>

#include <frc/LinearFilter.h>
#include <frc/MedianFilter.h>

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

  for (const auto& pt : data.slow) {
    if (pt.dt > 0_s && pt.dt < 500_ms) {
      dts.emplace_back(pt.dt);
    }
  }

  for (const auto& pt : data.fast) {
    if (pt.dt > 0_s && pt.dt < 500_ms) {
      dts.emplace_back(pt.dt);
    }
  }

  return std::accumulate(dts.begin(), dts.end(), 0_s) / dts.size();
}

void sysid::TrimQuasistaticData(std::vector<PreparedData>* data,
                                double motionThreshold) {
  data->erase(std::remove_if(data->begin(), data->end(),
                             [motionThreshold](const auto& pt) {
                               return std::abs(pt.voltage) <= 0 ||
                                      std::abs(pt.velocity) < motionThreshold;
                             }),
              data->end());
}

void sysid::ApplyMedianFilter(std::vector<PreparedData>* data, int window) {
  size_t step = window / 2;
  frc::MedianFilter<double> medianFilter(window);

  // Load the median filter with the first value, step times for accurate
  // behaviour at the start.

  for (int i = 0; i < step; i++) {
    medianFilter.Calculate(data->at(i).velocity);
  }

  for (size_t i = 0; i < data->size(); i++) {
    double median = medianFilter.Calculate(data->at(i).velocity);
    if (i >= step) {
      data->at(i - step).velocity = median;
    }
  }

  // Run the median filter for the last values and fill it with the last value
  for (int i = data->size() - step; i < data->size(); i++) {
    double median = medianFilter.Calculate(data->at(data->size() - 1).velocity);
    data->at(i).velocity = median;
  }
}

void sysid::FilterAccelData(std::vector<PreparedData>* data) {
  for (int i = 0; i < data->size(); i++) {
    if (data->at(i).acceleration == 0.0) {
      data->erase(data->begin() + i);
      i--;
    }
  }
}
