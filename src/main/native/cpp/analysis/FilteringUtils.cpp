// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FilteringUtils.h"

#include <numeric>
#include <vector>

#include <frc/LinearFilter.h>
#include <frc/MedianFilter.h>
#include <units/math.h>
#include <wpi/math>

using namespace sysid;

/**
 * Fills in the rest of the PreparedData Structs for a PreparedData Vector.
 *
 * @param data   A reference to a vector of the raw data.
 * @param window The window across which to compute the acceleration.
 * @param unit   The units that the data is in (rotations, radians, or degrees)
 * for arm mechanisms.
 */
static void PrepareMechData(std::vector<PreparedData>* data, int window,
                            wpi::StringRef unit = "") {
  // Calculate the step size for acceleration data.
  size_t step = window / 2;

  if (data->size() <= static_cast<size_t>(window)) {
    throw std::runtime_error(
        "The data collected is too small! This can be caused by too high of a "
        "motion threshold or bad data collection.");
  }

  // Compute acceleration and add it to the vector.
  for (size_t i = step; i < data->size() - step; ++i) {
    auto& pt1 = data->at(i);
    const auto& pt2 = data->at(i + 1);

    double accel = (data->at(i + step).velocity - data->at(i - step).velocity) /
                   (data->at(i + step).timestamp - data->at(i - step).timestamp)
                       .to<double>();

    pt1.acceleration = accel;

    // Calculates the cosine of the position data for single jointed arm
    // analysis
    double cos = 0.0;
    if (unit == "Radians") {
      cos = std::cos(pt1.position);
    } else if (unit == "Degrees") {
      cos = std::cos(pt1.position * wpi::math::pi / 180.0);
    } else if (unit == "Rotations") {
      cos = std::cos(pt1.position * 2 * wpi::math::pi);
    }
    pt1.cos = cos;
  }
}

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

  // Load the median filter with the first value, "step" number of times for
  // accurate initial behavior.
  for (int i = 0; i < step; i++) {
    medianFilter.Calculate(data->at(0).velocity);
  }

  for (size_t i = 0; i < data->size(); i++) {
    double median = medianFilter.Calculate(data->at(i).velocity);
    if (i >= step) {
      data->at(i - step).velocity = median;
    }
  }

  // Run the median filter for the last "step" datapoints by loading the median
  // filter with the last recorded velocity value.
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

/**
 * Figures out the max duration of the Dynamic tests
 *
 * @tparam S The size of the arrays in the raw data vector
 *
 * @param data The raw data String Map
 * @param timeCol The index of the time column
 *
 * @return The maximum duration of the Dynamic Tests
 */
static units::second_t GetMaxTime(
    wpi::StringMap<std::vector<PreparedData>>& data) {
  std::vector<double> durations;
  sysid::ApplyToData(
      data,
      [&](wpi::StringRef key) {
        durations.push_back(
            (data[key].back().timestamp - data[key].front().timestamp)
                .to<double>());
      },
      [](wpi::StringRef key) {
        return key.contains("fast") && key.contains("raw");
      });
  return units::second_t{durations[std::distance(
      durations.begin(),
      std::max_element(durations.begin(), durations.end()))]};
}

void sysid::InitialTrimAndFilter(
    wpi::StringMap<std::vector<PreparedData>>* data,
    AnalysisManager::Settings& settings, units::second_t& minStepTime,
    units::second_t& maxStepTime, wpi::StringRef unit) {
  auto& preparedData = *data;

  // Trim quasistatic test data to remove all points where voltage is zero or
  // velocity < motion threshold.
  sysid::ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        sysid::TrimQuasistaticData(&preparedData[key],
                                   settings.motionThreshold);
      },
      [](wpi::StringRef key) { return key.contains("slow"); });

  // Apply Median filter
  sysid::ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        sysid::ApplyMedianFilter(&preparedData[key], settings.windowSize);
      },
      [](wpi::StringRef key) { return !key.startswith("raw"); });

  // Recalculate Accel and Cosine
  sysid::ApplyToData(preparedData, [&](wpi::StringRef key) {
    PrepareMechData(&preparedData[key], settings.windowSize, unit);
  });

  // Find the maximum Step Test Duration
  maxStepTime = GetMaxTime(preparedData);

  // Trims all Dynamic Test Data but excludes raw data from calculation of
  // minimum step time
  sysid::ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        auto tempMinStepTime = sysid::TrimStepVoltageData(
            &preparedData[key], &settings, minStepTime, maxStepTime);
        if (!key.startswith("raw")) {
          minStepTime = tempMinStepTime;
        }
      },
      [](wpi::StringRef key) { return key.contains("fast"); });
  // Confirm there's still data
  if (std::any_of(preparedData.begin(), preparedData.end(),
                  [](const auto& it) { return it.first().empty(); })) {
    throw std::runtime_error("Trimming removed all data");
  }
}

/**
 * Trims data with dt too far from mean.
 *
 * @param data      The data to filter.
 * @param dtMean    The mean dt.
 * @param tolerance The tolerance outside of which to trim.
 */
static void TrimByTimeDelta(std::vector<PreparedData>* data,
                            units::second_t dtMean, units::second_t tolerance) {
  data->erase(std::remove_if(data->begin(), data->end(),
                             [dtMean, tolerance](const auto& pt) {
                               return units::math::abs(pt.dt - dtMean) >
                                      tolerance;
                             }),
              data->end());
}

void sysid::AccelAndTimeFilter(wpi::StringMap<std::vector<PreparedData>>* data,
                               const Storage& tempCombined) {
  auto& preparedData = *data;
  units::second_t dtMean = GetMeanTimeDelta(tempCombined);

  // Remove points with dt too far from mean
  sysid::ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        TrimByTimeDelta(&preparedData[key], dtMean, 1_ms);
      },
      [](wpi::StringRef key) { return !key.startswith("raw"); });

  // Remove points with accel = 0
  sysid::ApplyToData(preparedData, [&](wpi::StringRef key) {
    FilterAccelData(&preparedData[key]);
  });

  // Confirm there's still data
  if (std::any_of(preparedData.begin(), preparedData.end(),
                  [](const auto& it) { return it.first().empty(); })) {
    throw std::runtime_error("Trimming removed all data");
  }
}
