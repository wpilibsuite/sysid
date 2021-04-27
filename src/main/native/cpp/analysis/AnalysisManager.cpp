// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/AnalysisManager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <units/angle.h>
#include <units/math.h>
#include <wpi/StringMap.h>
#include <wpi/json.h>
#include <wpi/math>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FilteringUtils.h"
#include "sysid/analysis/JSONConverter.h"
#include "sysid/analysis/Storage.h"
#include "sysid/analysis/TrackWidthAnalysis.h"

using namespace sysid;

/**
 * Helper method that applies a function onto a dataset.
 *
 * @tparam T The type of data that is stored within a StringMap
 *
 * @param data   A StringMap representing a specific dataset
 * @param action A void function that takes a StringRef representing a StringMap
 *               key and performs an action to the dataset stored with that
 *               passed key (e.g. applying a median filter)
 */
template <typename T>
void ApplyToData(const wpi::StringMap<T>& data,
                 std::function<void(wpi::StringRef)> action) {
  for (const auto& it : data) {
    action(it.first());
  }
}

/**
 * Helper method that applies a function onto a dataset.
 *
 * @tparam T The type of data that is stored within a StringMap
 *
 * @param data      A StringMap representing a specific dataset
 * @param action    A void function that takes a StringRef representing a
 *                  StringMap key and performs an action to the dataset stored
 *                  with that passed key (e.g. applying a median filter)
 * @param specifier A boolean function that takes a StringRef representing a
 *                  StringMap key and returns true if `action` should be run on
 *                  the dataset stored with that key
 */
template <typename T>
void ApplyToData(const wpi::StringMap<T>& data,
                 std::function<void(wpi::StringRef)> action,
                 std::function<bool(wpi::StringRef)> specifier) {
  for (const auto& it : data) {
    auto key = it.first();
    if (specifier(key)) {
      action(key);
    }
  }
}

/**
 * Converts a raw data vector into a PreparedData vector with only the
 * timestamp, voltage, position, and velocity fields filled out.
 *
 * @tparam S The size of the arrays in the raw data vector
 * @tparam Timestamp The index of the Timestamp data in the raw data vector
 * arrays
 * @tparam Voltage The index of the Voltage data in the raw data vector arrays
 * @tparam Position The index of the Position data in the raw data vector arrays
 * @tparam Velocity The index of the Velocity data in the raw data vector arrays
 *
 * @param data A raw data vector
 *
 * @return A PreparedData vector
 */
template <size_t S, size_t Timestamp, size_t Voltage, size_t Position,
          size_t Velocity>
static std::vector<PreparedData> ConvertToPrepared(
    const std::vector<std::array<double, S>>& data) {
  std::vector<PreparedData> prepared;
  for (int i = 0; i < data.size() - 1; i++) {
    const auto& pt1 = data[i];
    const auto& pt2 = data[i + 1];
    prepared.emplace_back(
        PreparedData{units::second_t{pt1[Timestamp]}, pt1[Voltage],
                     pt1[Position], pt1[Velocity], pt2[Velocity],
                     units::second_t{pt2[Timestamp] - pt1[Timestamp]}});
  }
  return prepared;
}

/**
 * Concatenates a list of vectors to the end of a vector. The contents of the
 * source vectors are copied (not moved) into the new vector. Also sorts the
 * datapoints by timestamp to assist with future simulation.
 */
static std::vector<PreparedData> Concatenate(
    std::vector<PreparedData> dest,
    std::initializer_list<const std::vector<PreparedData>*> srcs) {
  // Copy the contents of the source vectors into the dest vector.
  for (auto ptr : srcs) {
    dest.insert(dest.end(), ptr->cbegin(), ptr->cend());
  }

  // Sort data by timestamp to remove the possibility of negative dts in future
  // simulations.
  std::sort(dest.begin(), dest.end(), [](const auto& a, const auto& b) {
    return a.timestamp < b.timestamp;
  });

  // Return the dest vector.
  return dest;
}

/**
 * Fills in the rest of the PreparedData Structs for a PreparedData Vector.
 *
 * @param data   A reference to a vector of the raw data.
 * @param window The window across which to compute the acceleration.
 * @param unit   The units that the data is in (rotations, radians, or degrees)
 * for arm mechanisms.
 */
void PrepareMechData(std::vector<PreparedData>* data, int window,
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
  ApplyToData(
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

/**
 * Trims the quasistatic tests, applies a median filter to the velocity data,
 * calculates acceleration and cosine (arm only) data, and trims the dynamic
 * tests.
 *
 * @param data A pointer to a data vector recently created by the
 *             ConvertToPrepared method
 * @param settings A reference to the analysis settings
 * @param minStepTime A reference to the minimum dynamic test duration
 * @param maxStepTime A reference to the maximum dynamic test duration
 * @param unit The angular unit that the arm test is in (only for calculating
 *             cosine data)
 */
static void InitialTrimAndFilter(
    wpi::StringMap<std::vector<PreparedData>>* data,
    AnalysisManager::Settings& settings, units::second_t& minStepTime,
    units::second_t& maxStepTime, wpi::StringRef unit = "") {
  auto& preparedData = *data;

  // Trim quasistatic test data to remove all points where voltage is zero or
  // velocity < motion threshold.
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        sysid::TrimQuasistaticData(&preparedData[key],
                                   settings.motionThreshold);
      },
      [](wpi::StringRef key) { return key.contains("slow"); });

  // Apply Median filter
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        sysid::ApplyMedianFilter(&preparedData[key], settings.windowSize);
      },
      [](wpi::StringRef key) { return !key.startswith("raw"); });

  // Recalculate Accel and Cosine
  ApplyToData(preparedData, [&](wpi::StringRef key) {
    PrepareMechData(&preparedData[key], settings.windowSize, unit);
  });

  // Find the maximum Step Test Duration
  maxStepTime = GetMaxTime(preparedData);

  // Trims all Dynamic Test Data but excludes raw data from calculation of
  // minimum step time
  ApplyToData(
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
 * Removes all points with accel = 0 and points with dt's greater than 1ms from
 * the mean dt.
 *
 * @param data A pointer to a PreparedData vector
 * @param tempCombined A reference to the combined filtered datasets
 */
static void AccelAndTimeFilter(wpi::StringMap<std::vector<PreparedData>>* data,
                               const Storage& tempCombined) {
  auto& preparedData = *data;
  units::second_t dtMean = GetMeanTimeDelta(tempCombined);

  // Remove points with dt too far from mean
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        TrimByTimeDelta(&preparedData[key], dtMean, 1_ms);
      },
      [](wpi::StringRef key) { return !key.startswith("raw"); });

  // Remove points with accel = 0
  ApplyToData(preparedData,
              [&](wpi::StringRef key) { FilterAccelData(&preparedData[key]); });

  // Confirm there's still data
  if (std::any_of(preparedData.begin(), preparedData.end(),
                  [](const auto& it) { return it.first().empty(); })) {
    throw std::runtime_error("Trimming removed all data");
  }
}

/**
 * Prepares data for general mechanisms (i.e. not drivetrain) and stores them
 * in the analysis manager dataset.
 *
 * @param json     A reference to the JSON containing all of the collected
 * data.
 * @param settings A reference to the settings being used by the analysis
 *                 manager instance.
 * @param factor   The units per rotation to multiply positions and velocities
 *                 by.
 * @param datasets A reference to the datasets object of the relevant analysis
 *                 manager instance.
 */
static void PrepareGeneralData(const wpi::json& json,
                               AnalysisManager::Settings& settings,
                               double factor, wpi::StringRef unit,
                               wpi::StringMap<Storage>& rawDatasets,
                               wpi::StringMap<Storage>& filteredDatasets,
                               std::array<units::second_t, 4>& startTimes,
                               units::second_t& minStepTime,
                               units::second_t& maxStepTime) {
  using Data = std::array<double, 4>;
  wpi::StringMap<std::vector<Data>> data;
  wpi::StringMap<std::vector<PreparedData>> preparedData;

  // Store the raw data columns.
  static constexpr size_t kTimeCol = 0;
  static constexpr size_t kVoltageCol = 1;
  static constexpr size_t kPosCol = 2;
  static constexpr size_t kVelCol = 3;

  // Get the major components from the JSON and store them inside a StringMap.
  for (auto&& key : AnalysisManager::kJsonDataKeys) {
    data[key] = json.at(key).get<std::vector<Data>>();
  }

  // Ensure that voltage and velocity have the same sign. Also multiply
  // positions and velocities by the factor.
  for (auto it = data.begin(); it != data.end(); ++it) {
    for (auto&& pt : it->second) {
      pt[kVoltageCol] = std::copysign(pt[kVoltageCol], pt[kVelCol]);
      pt[kPosCol] *= factor;
      pt[kVelCol] *= factor;
    }
  }

  // Loads the Raw Data
  ApplyToData(data, [&](wpi::StringRef key) {
    std::string rawName{"raw-" + key.str()};
    data[rawName] = std::vector<Data>(data[key]);
  });

  // Convert data to PreparedData structs
  ApplyToData(data, [&](wpi::StringRef key) {
    preparedData[key] =
        ConvertToPrepared<4, kTimeCol, kVoltageCol, kPosCol, kVelCol>(
            data[key]);
  });

  InitialTrimAndFilter(&preparedData, settings, minStepTime, maxStepTime, unit);
  // Compute mean dt
  auto& slowForward = preparedData["slow-forward"];
  auto& slowBackward = preparedData["slow-backward"];
  auto& fastForward = preparedData["fast-forward"];
  auto& fastBackward = preparedData["fast-backward"];
  Storage tempCombined{Concatenate(slowForward, {&slowBackward}),
                       Concatenate(fastForward, {&fastBackward})};

  AccelAndTimeFilter(&preparedData, tempCombined);

  // Store the raw datasets
  rawDatasets["Forward"] = Storage{preparedData["raw-slow-forward"],
                                   preparedData["raw-fast-forward"]};
  rawDatasets["Backward"] = Storage{preparedData["raw-slow-backward"],
                                    preparedData["raw-fast-backward"]};
  rawDatasets["Combined"] =
      Storage{Concatenate(preparedData["raw-slow-forward"],
                          {&preparedData["raw-slow-backward"]}),
              Concatenate(preparedData["raw-fast-forward"],
                          {&preparedData["raw-fast-backward"]})};

  // Store the filtered datasets
  filteredDatasets["Forward"] = Storage{slowForward, fastForward};
  filteredDatasets["Backward"] = Storage{slowBackward, fastBackward};
  filteredDatasets["Combined"] =
      Storage{Concatenate(slowForward, {&slowBackward}),
              Concatenate(fastForward, {&fastBackward})};
  startTimes = {preparedData["raw-slow-forward"][0].timestamp,
                preparedData["raw-slow-backward"][0].timestamp,
                preparedData["raw-fast-forward"][0].timestamp,
                preparedData["raw-fast-backward"][0].timestamp};
}

/**
 * Prepares data for angular drivetrain test data and stores them in
 * the analysis manager dataset.
 *
 * @param json       A reference to the JSON containing all of the collected
 *                   data.
 * @param settings   A reference to the settings being used by the analysis
 *                   manager instance.
 * @param factor     The units per rotation to multiply positions and velocities
 *                   by.
 * @param trackWidth A reference to the std::optional where the track width will
 *                   be stored.
 * @param datasets   A reference to the datasets object of the relevant analysis
 *                   manager instance.
 */
static void PrepareAngularDrivetrainData(
    const wpi::json& json, AnalysisManager::Settings& settings, double factor,
    std::optional<double>& trackWidth, wpi::StringMap<Storage>& rawDatasets,
    wpi::StringMap<Storage>& filteredDatasets,
    std::array<units::second_t, 4>& startTimes, units::second_t& minStepTime,
    units::second_t& maxStepTime) {
  using Data = std::array<double, 9>;
  wpi::StringMap<std::vector<Data>> data;
  wpi::StringMap<std::vector<PreparedData>> preparedData;

  // Store the relevant raw data columns.
  static constexpr size_t kTimeCol = 0;
  static constexpr size_t kVoltageCol = 1;
  static constexpr size_t kLPosCol = 3;
  static constexpr size_t kRPosCol = 4;
  static constexpr size_t kAngleCol = 7;
  static constexpr size_t kAngularRateCol = 8;

  // Get the major components from the JSON and store them inside a StringMap.
  for (auto&& key : AnalysisManager::kJsonDataKeys) {
    data[key] = json.at(key).get<std::vector<Data>>();
  }

  // Ensure that voltage and velocity have the same sign. Also multiply
  // positions and velocities by the factor.
  for (auto it = data.begin(); it != data.end(); ++it) {
    for (auto&& pt : it->second) {
      pt[kVoltageCol] = 2 * std::copysign(pt[kVoltageCol], pt[kAngularRateCol]);
      pt[kLPosCol] *= factor;
      pt[kRPosCol] *= factor;
    }
  }

  // Convert raw data to prepared data
  ApplyToData(data, [&](wpi::StringRef key) {
    preparedData[key] =
        ConvertToPrepared<9, kTimeCol, kVoltageCol, kAngleCol, kAngularRateCol>(
            data[key]);
  });

  // Trim quasistatic test data to remove all points where voltage is zero or
  // velocity < motion threshold.
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        sysid::TrimQuasistaticData(&preparedData[key],
                                   settings.motionThreshold);
      },
      [](wpi::StringRef key) { return key.contains("slow"); });

  // Get Max Time
  maxStepTime = GetMaxTime(preparedData);

  // Trim the step voltage data.
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        minStepTime = sysid::TrimStepVoltageData(&preparedData[key], &settings,
                                                 minStepTime, maxStepTime);
      },
      [](wpi::StringRef key) { return key.contains("fast"); });

  // Compute mean dt
  auto& slowForward = preparedData["slow-forward"];
  auto& slowBackward = preparedData["slow-backward"];
  auto& fastForward = preparedData["fast-forward"];
  auto& fastBackward = preparedData["fast-backward"];
  Storage tempCombined{Concatenate(slowForward, {&slowBackward}),
                       Concatenate(fastForward, {&fastBackward})};
  units::second_t dtMean = GetMeanTimeDelta(tempCombined);

  // Remove points with dt too far from mean
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        TrimByTimeDelta(&preparedData[key], dtMean, 1_ms);
      },
      [](wpi::StringRef key) { return !key.startswith("raw"); });

  // Confirm there's still data
  if (std::all_of(preparedData.begin(), preparedData.end(),
                  [](const auto& it) { return it.first().empty(); })) {
    throw std::runtime_error("Trimming removed all data");
  }

  // Calculate track width from the slow-forward raw data.
  auto& trackWidthData = data["slow-forward"];
  double leftDelta =
      trackWidthData.back()[kLPosCol] - trackWidthData.front()[kLPosCol];
  double rightDelta =
      trackWidthData.back()[kRPosCol] - trackWidthData.front()[kRPosCol];
  double angleDelta =
      trackWidthData.back()[kAngleCol] - trackWidthData.front()[kAngleCol];
  trackWidth = sysid::CalculateTrackWidth(leftDelta, rightDelta,
                                          units::radian_t{angleDelta});

  // Create the distinct datasets and store them in our StringMap.
  filteredDatasets["Forward"] = Storage{slowForward, fastForward};
  filteredDatasets["Backward"] = Storage{slowBackward, fastBackward};
  filteredDatasets["Combined"] =
      Storage{Concatenate(slowForward, {&slowBackward}),
              Concatenate(fastForward, {&fastBackward})};
  startTimes = {slowForward[0].timestamp, slowBackward[0].timestamp,
                fastForward[0].timestamp, fastBackward[0].timestamp};
}

/**
 * Prepares data for linear drivetrain test data and stores them in
 * the analysis manager dataset.
 *
 * @param json     A reference to the JSON containing all of the collected
 * data.
 * @param settings A reference to the settings being used by the analysis
 *                 manager instance.
 * @param factor   The units per rotation to multiply positions and velocities
 *                 by.
 * @param datasets A reference to the datasets object of the relevant analysis
 *                 manager instance.
 */
static void PrepareLinearDrivetrainData(
    const wpi::json& json, AnalysisManager::Settings& settings, double factor,
    wpi::StringMap<Storage>& rawDatasets,
    wpi::StringMap<Storage>& filteredDatasets,
    std::array<units::second_t, 4>& startTimes, units::second_t& minStepTime,
    units::second_t& maxStepTime) {
  using Data = std::array<double, 9>;
  wpi::StringMap<std::vector<Data>> data;
  wpi::StringMap<std::vector<PreparedData>> preparedData;

  // Store the relevant raw data columns.
  static constexpr size_t kTimeCol = 0;
  static constexpr size_t kLVoltageCol = 1;
  static constexpr size_t kRVoltageCol = 2;
  static constexpr size_t kLPosCol = 3;
  static constexpr size_t kRPosCol = 4;
  static constexpr size_t kLVelCol = 5;
  static constexpr size_t kRVelCol = 6;

  // Get the major components from the JSON and store them inside a StringMap.
  for (auto&& key : AnalysisManager::kJsonDataKeys) {
    data[key] = json.at(key).get<std::vector<Data>>();
  }

  // Ensure that voltage and velocity have the same sign. Also multiply
  // positions and velocities by the factor.
  for (auto it = data.begin(); it != data.end(); ++it) {
    for (auto&& pt : it->second) {
      pt[kLVoltageCol] = std::copysign(pt[kLVoltageCol], pt[kLVelCol]);
      pt[kRVoltageCol] = std::copysign(pt[kRVoltageCol], pt[kRVelCol]);
      pt[kLPosCol] *= factor;
      pt[kRPosCol] *= factor;
      pt[kLVelCol] *= factor;
      pt[kRVelCol] *= factor;
    }
  }

  // Load Raw Data
  ApplyToData(data, [&](wpi::StringRef key) {
    std::string rawName{"raw-" + key.str()};
    data[rawName] = std::vector<Data>(data[key]);
  });

  // Convert data to PreparedData
  ApplyToData(data, [&](wpi::StringRef key) {
    std::string leftName{"left-" + key.str()};
    std::string rightName{"right-" + key.str()};
    preparedData[leftName] =
        ConvertToPrepared<9, kTimeCol, kLVoltageCol, kLPosCol, kLVelCol>(
            data[key]);
    preparedData[rightName] =
        ConvertToPrepared<9, kTimeCol, kRVoltageCol, kRPosCol, kRVelCol>(
            data[key]);
  });

  InitialTrimAndFilter(&preparedData, settings, minStepTime, maxStepTime);
  // Store filtered data
  auto& slowForwardLeft = preparedData["left-slow-forward"];
  auto& slowForwardRight = preparedData["right-slow-forward"];
  auto& slowBackwardLeft = preparedData["left-slow-backward"];
  auto& slowBackwardRight = preparedData["right-slow-backward"];
  auto& fastForwardLeft = preparedData["left-fast-forward"];
  auto& fastForwardRight = preparedData["right-fast-forward"];
  auto& fastBackwardLeft = preparedData["left-fast-backward"];
  auto& fastBackwardRight = preparedData["right-fast-backward"];

  auto slowForward = Concatenate(slowForwardLeft, {&slowForwardRight});
  auto slowBackward = Concatenate(slowBackwardLeft, {&slowBackwardRight});
  auto fastForward = Concatenate(fastForwardLeft, {&fastForwardRight});
  auto fastBackward = Concatenate(fastBackwardLeft, {&fastBackwardRight});

  // Compute mean dt
  Storage tempCombined{Concatenate(slowForward, {&slowBackward}),
                       Concatenate(fastForward, {&fastBackward})};

  AccelAndTimeFilter(&preparedData, tempCombined);

  // Store raw data as variables
  auto rawSlowForwardLeft = preparedData["left-raw-slow-forward"];
  auto rawSlowForwardRight = preparedData["right-raw-slow-forward"];
  auto rawSlowBackwardLeft = preparedData["left-raw-slow-backward"];
  auto rawSlowBackwardRight = preparedData["right-raw-slow-backward"];
  auto rawFastForwardLeft = preparedData["left-raw-fast-forward"];
  auto rawFastForwardRight = preparedData["right-raw-fast-forward"];
  auto rawFastBackwardLeft = preparedData["left-raw-fast-backward"];
  auto rawFastBackwardRight = preparedData["right-raw-fast-backward"];

  // Create the distinct raw datasets and store them in our StringMap.
  auto rawSlowForward = Concatenate(rawSlowForwardLeft, {&rawSlowForwardRight});
  auto rawSlowBackward =
      Concatenate(rawSlowBackwardLeft, {&rawSlowBackwardRight});
  auto rawFastForward = Concatenate(rawFastForwardLeft, {&rawFastForwardRight});
  auto rawFastBackward =
      Concatenate(rawFastBackwardLeft, {&rawFastBackwardRight});

  rawDatasets["Forward"] = Storage{rawSlowForward, rawFastForward};
  rawDatasets["Backward"] = Storage{rawSlowBackward, rawFastBackward};
  rawDatasets["Combined"] =
      Storage{Concatenate(rawSlowForward, {&rawSlowBackward}),
              Concatenate(rawFastForward, {&rawFastBackward})};

  rawDatasets["Left Forward"] = Storage{rawSlowForwardLeft, rawFastForwardLeft};
  rawDatasets["Left Backward"] =
      Storage{rawSlowBackwardLeft, rawFastBackwardLeft};
  rawDatasets["Left Combined"] =
      Storage{Concatenate(rawSlowForwardLeft, {&rawSlowBackwardLeft}),
              Concatenate(rawFastForwardLeft, {&rawFastBackwardLeft})};

  rawDatasets["Right Forward"] =
      Storage{rawSlowForwardRight, rawFastForwardRight};
  rawDatasets["Right Backward"] =
      Storage{rawSlowBackwardRight, rawFastBackwardRight};
  rawDatasets["Right Combined"] =
      Storage{Concatenate(rawSlowForwardRight, {&rawSlowBackwardRight}),
              Concatenate(rawFastForwardRight, {&rawFastBackwardRight})};

  // Create the distinct filtered datasets and store them in our StringMap.
  filteredDatasets["Forward"] = Storage{slowForward, fastForward};
  filteredDatasets["Backward"] = Storage{slowBackward, fastBackward};
  filteredDatasets["Combined"] =
      Storage{Concatenate(slowForward, {&slowBackward}),
              Concatenate(fastForward, {&fastBackward})};
  filteredDatasets["Left Forward"] = Storage{slowForwardLeft, fastForwardLeft};
  filteredDatasets["Left Backward"] =
      Storage{slowBackwardLeft, fastBackwardLeft};
  filteredDatasets["Left Combined"] =
      Storage{Concatenate(slowForwardLeft, {&slowBackwardLeft}),
              Concatenate(fastForwardLeft, {&fastBackwardLeft})};
  filteredDatasets["Right Forward"] =
      Storage{slowForwardRight, fastForwardRight};
  filteredDatasets["Right Backward"] =
      Storage{slowBackwardRight, fastBackwardRight};
  filteredDatasets["Right Combined"] =
      Storage{Concatenate(slowForwardRight, {&slowBackwardRight}),
              Concatenate(fastForwardRight, {&fastBackwardRight})};
  startTimes = {
      rawSlowForward.front().timestamp, rawSlowBackward.front().timestamp,
      rawFastForward.front().timestamp, rawFastBackward.front().timestamp};
}

AnalysisManager::AnalysisManager(wpi::StringRef path, Settings& settings,
                                 wpi::Logger& logger)
    : m_settings(settings), m_logger(logger) {
  // Read JSON from the specified path.
  std::error_code ec;
  wpi::raw_fd_istream is{path, ec};

  if (ec) {
    throw std::runtime_error("Unable to read: " + path.str());
  }

  is >> m_json;
  WPI_INFO(m_logger, "Read " << path);

  // Check that we have a sysid json.
  if (m_json.find("sysid") == m_json.end()) {
    throw std::runtime_error(
        "Incorrect JSON format detected. Please use the JSON Converter "
        "to convert a frc-char JSON to a sysid JSON.");
  } else {
    // Get the analysis type from the JSON.
    m_type = sysid::analysis::FromName(m_json.at("test").get<std::string>());

    // Get the rotation -> output units factor from the JSON.
    m_unit = m_json.at("units").get<std::string>();
    m_factor = m_json.at("unitsPerRotation").get<double>();

    // Reset settings for Dynamic Test Limits
    m_settings.stepTestDuration = units::second_t{0.0};
    m_minDuration = units::second_t{100000};

    // Prepare data.
    PrepareData();
  }
}

void AnalysisManager::PrepareData() {
  if (m_type == analysis::kDrivetrain) {
    PrepareLinearDrivetrainData(m_json, m_settings, m_factor, m_rawDatasets,
                                m_filteredDatasets, m_startTimes, m_minDuration,
                                m_maxDuration);
  } else if (m_type == analysis::kDrivetrainAngular) {
    PrepareAngularDrivetrainData(m_json, m_settings, m_factor, m_trackWidth,
                                 m_rawDatasets, m_filteredDatasets,
                                 m_startTimes, m_minDuration, m_maxDuration);
  } else {
    PrepareGeneralData(m_json, m_settings, m_factor, m_unit, m_rawDatasets,
                       m_filteredDatasets, m_startTimes, m_minDuration,
                       m_maxDuration);
  }
}

AnalysisManager::Gains AnalysisManager::Calculate() {
  // Calculate feedforward gains from the data.
  auto ffGains = sysid::CalculateFeedforwardGains(
      m_filteredDatasets[kDatasets[m_settings.dataset]], m_type);

  const auto& Kv = std::get<0>(ffGains)[1];
  const auto& Ka = std::get<0>(ffGains)[2];

  // Calculate the appropriate gains.
  FeedbackGains fbGains;
  if (m_settings.type == FeedbackControllerLoopType::kPosition) {
    fbGains = sysid::CalculatePositionFeedbackGains(
        m_settings.preset, m_settings.lqr, Kv, Ka,
        m_settings.convertGainsToEncTicks
            ? m_settings.gearing * m_settings.cpr * m_factor
            : 1);
  } else {
    fbGains = sysid::CalculateVelocityFeedbackGains(
        m_settings.preset, m_settings.lqr, Kv, Ka,
        m_settings.convertGainsToEncTicks
            ? m_settings.gearing * m_settings.cpr * m_factor
            : 1);
  }
  return {ffGains, fbGains, m_trackWidth};
}

void AnalysisManager::OverrideUnits(wpi::StringRef unit,
                                    double unitsPerRotation) {
  m_unit = unit;
  m_factor = unitsPerRotation;
  PrepareData();
}

void AnalysisManager::ResetUnitsFromJSON() {
  m_unit = m_json.at("units").get<std::string>();
  m_factor = m_json.at("unitsPerRotation").get<double>();
  PrepareData();
}
