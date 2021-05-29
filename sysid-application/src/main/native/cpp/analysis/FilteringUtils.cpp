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
 * Fills in the rest of the PreparedData Structs for a PreparedData Vector.
 *
 * @param data A reference to a vector of the raw data.
 * @param unit The units that the data is in (rotations, radians, or degrees)
 *             for arm mechanisms.
 */
static void PrepareMechData(std::vector<PreparedData>* data,
                            wpi::StringRef unit = "") {
  constexpr size_t kOrder = 6;
  constexpr size_t kWindow = kOrder + 1;

  CheckSize(*data, kWindow);

  const double h = GetMeanTimeDelta(*data).to<double>();

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
      cos = std::cos(pt.position * wpi::math::pi / 180.0);
    } else if (unit == "Rotations") {
      cos = std::cos(pt.position * 2 * wpi::math::pi);
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

void sysid::FilterAccelData(std::vector<PreparedData>* data) {
  for (int i = 0; i < data->size(); i++) {
    if (data->at(i).acceleration == 0.0) {
      data->erase(data->begin() + i);
      i--;
    }
  }
}

/**
 * Removes a substring from a string reference
 *
 * @param str The wpi::StringRef that needs modification
 * @param removeStr The substring that needs to be removed
 *
 * @return an std::string without the specified substring
 */
static std::string RemoveStr(wpi::StringRef str, wpi::StringRef removeStr) {
  auto pair = str.split(removeStr);
  return pair.first.str() + pair.second.str();
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
      [](wpi::StringRef key) { return !key.contains("raw"); });

  // Recalculate Accel and Cosine
  sysid::ApplyToData(preparedData, [&](wpi::StringRef key) {
    PrepareMechData(&preparedData[key], unit);
  });

  // Find the maximum Step Test Duration
  maxStepTime = GetMaxTime(preparedData);

  // Trims filtered Dynamic Test Data and lines up the Raw Data to facilitate
  // plotting
  sysid::ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        // Get the filtered dataset name
        auto filteredKey = RemoveStr(key, "raw-");

        // Trim Filtered Data
        auto tempMinStepTime = sysid::TrimStepVoltageData(
            &preparedData[filteredKey], &settings, minStepTime, maxStepTime);
        minStepTime = tempMinStepTime;

        // Set the Raw Data to start at the same time as the Filtered Data
        auto startTime = preparedData[filteredKey].front().timestamp;
        auto rawStart =
            std::find_if(preparedData[key].begin(), preparedData[key].end(),
                         [&](auto&& pt) { return pt.timestamp == startTime; });
        preparedData[key].erase(preparedData[key].begin(), rawStart);
      },
      [](wpi::StringRef key) {
        return key.contains("fast") && key.contains("raw");
      });
  // Confirm there's still data
  if (std::any_of(preparedData.begin(), preparedData.end(),
                  [](const auto& it) { return it.first().empty(); })) {
    throw std::runtime_error("Trimming removed all data");
  }
}

void sysid::AccelAndTimeFilter(wpi::StringMap<std::vector<PreparedData>>* data,
                               const Storage& tempCombined) {
  auto& preparedData = *data;

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
