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

#include <units/angle.h>
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
 * Concatenates a list of vectors to the end of a vector. The contents of the
 * source vectors are copied (not moved) into the new vector.
 */
static std::vector<PreparedData> Concatenate(
    std::vector<PreparedData> dest,
    std::initializer_list<const std::vector<PreparedData>*> srcs) {
  // Copy the contents of the source vectors into the dest vector.
  for (auto ptr : srcs) {
    dest.insert(dest.end(), ptr->cbegin(), ptr->cend());
  }

  // Return the dest vector.
  return dest;
}

/**
 * Computes acceleration from a vector of raw data and returns prepared data.
 *
 * @tparam S        The size of the raw data array.
 * @tparam Voltage  The index of the voltage entry in the raw data.
 * @tparam Position The index of the position entry in the raw data.
 * @tparam Velocity The index of the velocity entry in the raw data.
 *
 * @param data A reference to a vector of the raw data.
 */
template <size_t S, size_t Voltage, size_t Position, size_t Velocity>
std::vector<PreparedData> ComputeAcceleration(
    const std::vector<std::array<double, S>>& data, int window) {
  // Calculate the step size for acceleration data.
  size_t step = window / 2;

  // Create our prepared data vector.
  std::vector<PreparedData> prepared;
  if (data.size() <= static_cast<size_t>(window)) {
    throw std::runtime_error(
        "The data collected is too small! This can be caused by too high of a "
        "motion threshold or bad data collection.");
  }
  prepared.reserve(data.size());

  // Compute acceleration and add it to the vector.
  for (size_t i = step; i < data.size() - step; ++i) {
    const auto& pt = data[i];
    double acc = (data[i + step][Velocity] - data[i - step][Velocity]) /
                 (data[i + step][0] - data[i - step][0]);

    // Sometimes, if the encoder velocities are the same, it will register zero
    // acceleration. Do not include these values.
    if (acc != 0) {
      prepared.push_back(PreparedData{units::second_t{pt[0]}, pt[Voltage],
                                      pt[Position], pt[Velocity], acc, 0.0});
    }
  }
  return prepared;
}

/**
 * Calculates the cosine of the position data for single jointed arm analysis.
 *
 * @param data The data to calculate the cosine on.
 * @param unit The units that the data is in (rotations, radians, or degrees).
 */
static void CalculateCosine(std::vector<PreparedData>* data,
                            wpi::StringRef unit) {
  for (auto&& pt : *data) {
    if (unit == "Radians") {
      pt.cos = std::cos(pt.position);
    } else if (unit == "Degrees") {
      pt.cos = std::cos(pt.position * wpi::math::pi / 180.0);
    } else if (unit == "Rotations") {
      pt.cos = std::cos(pt.position * 2 * wpi::math::pi);
    }
  }
}

template <size_t S>
static units::second_t GetMaxTime(
    wpi::StringMap<std::vector<std::array<double, S>>> data, size_t timeCol) {
  return units::second_t{std::max(
      data["fast-forward"].back()[timeCol] - data["fast-forward"][0][timeCol],
      data["fast-backward"].back()[timeCol] -
          data["fast-backward"][0][timeCol])};
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

  // Trim quasistatic test data to remove all points where voltage is zero or
  // velocity < motion threshold.
  ApplyToData(
      data,
      [&](wpi::StringRef key) {
        sysid::TrimQuasistaticData<4, kVoltageCol, kVelCol>(
            &data[key], settings.motionThreshold);
      },
      [](wpi::StringRef key) { return key.contains("slow"); });

  // Apply Median filter
  ApplyToData(
      data,
      [&](wpi::StringRef key) {
        sysid::ApplyMedianFilter<4, kVelCol>(&data[key], settings.windowSize);
      },
      [](wpi::StringRef key) { return !key.startswith("raw"); });

  // Get accel
  ApplyToData(data, [&](wpi::StringRef key) {
    preparedData[key] = ComputeAcceleration<4, kVoltageCol, kPosCol, kVelCol>(
        data[key], settings.windowSize);
  });

  // Calculate cosine of position data for filtered data
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) { CalculateCosine(&preparedData[key], unit); },
      [](wpi::StringRef key) { return !key.startswith("raw"); });

  // Find the maximum Step Test Duration
  maxStepTime = GetMaxTime<4>(data, kTimeCol);

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
  auto slowForward = preparedData["slow-forward"];
  auto slowBackward = preparedData["slow-backward"];
  auto fastForward = preparedData["fast-forward"];
  auto fastBackward = preparedData["fast-backward"];
  filteredDatasets["Forward"] = Storage{slowForward, fastForward};
  filteredDatasets["Backward"] = Storage{slowBackward, fastBackward};
  filteredDatasets["Combined"] =
      Storage{Concatenate(slowForward, {&slowBackward}),
              Concatenate(fastForward, {&fastBackward})};
  startTimes = {slowForward[0].timestamp, slowBackward[0].timestamp,
                fastForward[0].timestamp, fastBackward[0].timestamp};
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

  // Trim quasistatic test data to remove all points where voltage is zero or
  // velocity < motion threshold.
  ApplyToData(
      data,
      [&](wpi::StringRef key) {
        sysid::TrimQuasistaticData<9, kVoltageCol, kAngularRateCol>(
            &data[key], settings.motionThreshold);
      },
      [](wpi::StringRef key) { return key.contains("slow"); });

  // Compute acceleration on all data sets.
  ApplyToData(data, [&](wpi::StringRef key) {
    preparedData[key] =
        ComputeAcceleration<9, kVoltageCol, kAngleCol, kAngularRateCol>(
            data[key], settings.windowSize);
  });

  // Get Max Time
  maxStepTime = GetMaxTime<9>(data, kTimeCol);

  // Trim the step voltage data.
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        minStepTime = sysid::TrimStepVoltageData(&preparedData[key], &settings,
                                                 minStepTime, maxStepTime);
      },
      [](wpi::StringRef key) { return key.contains("fast"); });

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
  auto slowForward = preparedData["slow-forward"];
  auto fastForward = preparedData["fast-forward"];
  auto slowBackward = preparedData["slow-backward"];
  auto fastBackward = preparedData["fast-backward"];
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

  // Trim Quasistatic Data
  ApplyToData(
      data,
      [&](wpi::StringRef key) {
        sysid::TrimQuasistaticData<9, kLVoltageCol, kLVelCol>(
            &data[key], settings.motionThreshold);
        sysid::TrimQuasistaticData<9, kRVoltageCol, kRVelCol>(
            &data[key], settings.motionThreshold);
      },
      [](wpi::StringRef key) { return key.contains("slow"); });

  // Apply Median Filter
  ApplyToData(
      data,
      [&](wpi::StringRef key) {
        sysid::ApplyMedianFilter<9, kLVelCol>(&data[key], settings.windowSize);
        sysid::ApplyMedianFilter<9, kRVelCol>(&data[key], settings.windowSize);
      },
      [](wpi::StringRef key) { return !key.contains("raw"); });

  // Get Accel Data
  ApplyToData(data, [&](wpi::StringRef key) {
    std::string leftName{"left-" + key.str()};
    std::string rightName{"right-" + key.str()};
    preparedData[leftName] =
        ComputeAcceleration<9, kLVoltageCol, kLPosCol, kLVelCol>(
            data[key], settings.windowSize);
    preparedData[rightName] =
        ComputeAcceleration<9, kRVoltageCol, kRPosCol, kRVelCol>(
            data[key], settings.windowSize);
  });

  // Get maximum dynamic test duration
  maxStepTime = GetMaxTime<9>(data, kTimeCol);

  // Trim step test data but exclude raw data from calculating min step time
  ApplyToData(
      preparedData,
      [&](wpi::StringRef key) {
        auto tempMinStepTime = sysid::TrimStepVoltageData(
            &preparedData[key], &settings, minStepTime, maxStepTime);
        if (!key.contains("raw")) {
          minStepTime = tempMinStepTime;
        }
      },
      [](wpi::StringRef key) { return key.contains("fast"); });

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

  // Store filtered data
  auto slowForwardLeft = preparedData["left-slow-forward"];
  auto slowForwardRight = preparedData["right-slow-forward"];
  auto slowBackwardLeft = preparedData["left-slow-backward"];
  auto slowBackwardRight = preparedData["right-slow-backward"];
  auto fastForwardLeft = preparedData["left-fast-forward"];
  auto fastForwardRight = preparedData["right-fast-forward"];
  auto fastBackwardLeft = preparedData["left-fast-backward"];
  auto fastBackwardRight = preparedData["right-fast-backward"];

  auto slowForward = Concatenate(slowForwardLeft, {&slowForwardRight});
  auto slowBackward = Concatenate(slowBackwardLeft, {&slowBackwardRight});
  auto fastForward = Concatenate(fastForwardLeft, {&fastForwardRight});
  auto fastBackward = Concatenate(fastBackwardLeft, {&fastBackwardRight});

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
  startTimes = {slowForward.front().timestamp, slowBackward.front().timestamp,
                fastForward.front().timestamp, fastBackward.front().timestamp};
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
