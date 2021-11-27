// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FilteringUtils.h"

#include <numeric>
#include <stdexcept>
#include <vector>

#include <fmt/format.h>
#include <frc/filter/LinearFilter.h>
#include <frc/filter/MedianFilter.h>
#include <units/math.h>
#include <wpi/StringExtras.h>
#include <wpi/numbers>

using namespace sysid;

/**
 * Helper function that throws if it detects that the data vector is too small
 * for an operation of a certain window size.
 *
 * @param data The data that is being used.
 * @param window The window size for the operation.
 */
static void CheckSize(const std::vector<PreparedData>& data, int window) {
  if (data.size() < window) {
    throw std::runtime_error(
        "The data collected is too small! This can be caused by too high of a "
        "motion threshold or bad data collection.");
  }
}

/**
 * Helper function that determines if a certain key is storing raw data.
 *
 * @param key The key of the dataset
 *
 * @return True, if the key corresponds to a raw dataset.
 */
static bool IsRaw(std::string_view key) {
  return wpi::contains(key, "raw") && !wpi::contains(key, "original");
}

/**
 * Helper function that determines if a certain key is storing filtered data.
 *
 * @param key The key of the dataset
 *
 * @return True, if the key corresponds to a filtered dataset.
 */
static bool IsFiltered(std::string_view key) {
  return !wpi::contains(key, "raw") && !wpi::contains(key, "original");
}

/**
 * Fills in the rest of the PreparedData Structs for a PreparedData Vector.
 *
 * @param data A reference to a vector of the raw data.
 * @param unit The units that the data is in (rotations, radians, or degrees)
 *             for arm mechanisms.
 */
static void PrepareMechData(std::vector<PreparedData>* data,
                            std::string_view unit = "") {
  constexpr size_t kOrder = 6;
  constexpr size_t kWindow = kOrder + 1;

  CheckSize(*data, kWindow);

  const double h = GetMeanTimeDelta(*data).value();

  // Compute acceleration and add it to the vector.
  for (size_t i = kWindow / 2; i < data->size() - kWindow / 2; ++i) {
    auto& pt = data->at(i);

    pt.acceleration = CentralFiniteDifference<kOrder>(
        [&](size_t i) { return data->at(i).velocity; }, i, h);

    // Calculates the cosine of the position data for single jointed arm
    // analysis
    double cos = 0.0;
    if (unit == "Radians") {
      cos = std::cos(pt.position);
    } else if (unit == "Degrees") {
      cos = std::cos(pt.position * wpi::numbers::pi / 180.0);
    } else if (unit == "Rotations") {
      cos = std::cos(pt.position * 2 * wpi::numbers::pi);
    }
    pt.cos = cos;
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

  // If step duration hasn't been set yet, calculate a default (find the entry
  // before the acceleration first hits zero)
  if (settings->stepTestDuration <= minStepTime) {
    // Get noise floor
    const double accelNoiseFloor =
        GetAccelNoiseFloor(*data, settings->windowSize);
    // Find latest element with nonzero acceleration
    auto endIt = std::find_if(
        data->rbegin(), data->rend(), [&](const PreparedData& entry) {
          return std::abs(entry.acceleration) > accelNoiseFloor;
        });

    if (endIt != data->rend()) {
      // Calculate default duration
      settings->stepTestDuration = std::min(
          endIt->timestamp - data->front().timestamp + minStepTime + 1_s,
          maxStepTime);
    } else {
      settings->stepTestDuration = maxStepTime;
    }
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
  auto averageFilter = frc::LinearFilter<double>::MovingAverage(window);
  for (size_t i = 0; i < data.size(); i++) {
    double mean = averageFilter.Calculate(data[i].acceleration);
    if (i >= step) {
      sum += std::pow(data[i - step].acceleration - mean, 2);
    }
  }
  return std::sqrt(sum / (data.size() - step));
}

units::second_t sysid::GetMeanTimeDelta(const std::vector<PreparedData>& data) {
  std::vector<units::second_t> dts;

  for (const auto& pt : data) {
    if (pt.dt > 0_s && pt.dt < 500_ms) {
      dts.emplace_back(pt.dt);
    }
  }

  return std::accumulate(dts.begin(), dts.end(), 0_s) / dts.size();
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

void sysid::ApplyMedianFilter(std::vector<PreparedData>* data, int window) {
  CheckSize(*data, window);

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

/**
 * Removes a substring from a string reference
 *
 * @param str The std::string_view that needs modification
 * @param removeStr The substring that needs to be removed
 *
 * @return an std::string without the specified substring
 */
static std::string RemoveStr(std::string_view str, std::string_view removeStr) {
  size_t idx = str.find(removeStr);
  if (idx == std::string_view::npos) {
    return std::string{str};
  } else {
    return fmt::format("{}{}", str.substr(0, idx),
                       str.substr(idx + removeStr.size()));
  }
}

/**
 * Figures out the max duration of the Dynamic tests
 *
 * @param data The raw data String Map
 * @param timeCol The index of the time column
 *
 * @return The maximum duration of the Dynamic Tests
 */
static units::second_t GetMaxTime(
    wpi::StringMap<std::vector<PreparedData>>& data) {
  auto maxDuration = 0_s;
  for (auto& it : data) {
    auto key = it.first();
    auto& dataset = it.getValue();

    if (IsRaw(key) && wpi::contains(key, "fast")) {
      auto duration = dataset.back().timestamp - dataset.front().timestamp;
      if (duration > maxDuration) {
        maxDuration = duration;
      }
    }
  }
  return maxDuration;
}

void sysid::InitialTrimAndFilter(
    wpi::StringMap<std::vector<PreparedData>>* data,
    AnalysisManager::Settings& settings, units::second_t& minStepTime,
    units::second_t& maxStepTime, std::string_view unit) {
  auto& preparedData = *data;

  // Find the maximum Step Test Duration of the dynamic tests
  maxStepTime = GetMaxTime(preparedData);

  for (auto& it : preparedData) {
    auto key = it.first();
    auto& dataset = it.getValue();

    // Trim quasistatic test data to remove all points where voltage is zero or
    // velocity < motion threshold.
    if (wpi::contains(key, "slow")) {
      dataset.erase(std::remove_if(dataset.begin(), dataset.end(),
                                   [&](const auto& pt) {
                                     return std::abs(pt.voltage) <= 0 ||
                                            std::abs(pt.velocity) <
                                                settings.motionThreshold;
                                   }),
                    dataset.end());

      // Confirm there's still data
      if (dataset.empty()) {
        throw std::runtime_error("Quasistatic test trimming removed all data");
      }
    }

    // Apply Median filter
    if (IsFiltered(key)) {
      ApplyMedianFilter(&dataset, settings.windowSize);
    }

    // Recalculate Accel and Cosine
    PrepareMechData(&dataset, unit);

    // Trims filtered Dynamic Test Data
    if (IsFiltered(key) && wpi::contains(key, "fast")) {
      // Get the filtered dataset name
      auto filteredKey = RemoveStr(key, "raw-");

      // Trim Filtered Data
      auto tempMinStepTime = TrimStepVoltageData(
          &preparedData[filteredKey], &settings, minStepTime, maxStepTime);
      minStepTime = tempMinStepTime;

      // Set the Raw Data to start at the same time as the Filtered Data
      auto startTime = preparedData[filteredKey].front().timestamp;
      auto rawStart =
          std::find_if(preparedData[key].begin(), preparedData[key].end(),
                       [&](auto&& pt) { return pt.timestamp == startTime; });
      preparedData[key].erase(preparedData[key].begin(), rawStart);

      // Confirm there's still data
      if (preparedData[key].empty()) {
        throw std::runtime_error("Dynamic test trimming removed all data");
      }
    }
  }
}

void sysid::AccelFilter(wpi::StringMap<std::vector<PreparedData>>* data) {
  auto& preparedData = *data;

  // Remove points with acceleration = 0
  for (auto& it : preparedData) {
    auto& dataset = it.getValue();

    for (int i = 0; i < dataset.size(); i++) {
      if (dataset.at(i).acceleration == 0.0) {
        dataset.erase(dataset.begin() + i);
        i--;
      }
    }
  }

  // Confirm there's still data
  if (std::any_of(preparedData.begin(), preparedData.end(),
                  [](const auto& it) { return it.getValue().empty(); })) {
    throw std::runtime_error("Acceleration filtering removed all data");
  }
}
