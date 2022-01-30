// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/AnalysisManager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <fmt/format.h>
#include <units/angle.h>
#include <units/math.h>
#include <wpi/StringExtras.h>
#include <wpi/StringMap.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FilteringUtils.h"
#include "sysid/analysis/JSONConverter.h"
#include "sysid/analysis/Storage.h"
#include "sysid/analysis/TrackWidthAnalysis.h"

using namespace sysid;

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
    prepared.emplace_back(PreparedData{
        units::second_t{pt1[Timestamp]}, pt1[Voltage], pt1[Position],
        pt1[Velocity], units::second_t{pt2[Timestamp] - pt1[Timestamp]}});
  }
  return prepared;
}

/**
 * Concatenates a list of vectors. The contents of the source vectors are copied
 * (not moved) into the new vector. Also sorts the datapoints by timestamp to
 * assist with future simulation.
 *
 * @param sources The source vectors.
 * @return The concatenated vector
 */
template <typename... Sources>
std::vector<PreparedData> DataConcat(const Sources&... sources) {
  std::vector<PreparedData> result;
  (result.insert(result.end(), sources.begin(), sources.end()), ...);

  // Sort data by timestamp to remove the possibility of negative dts in future
  // simulations.
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.timestamp < b.timestamp;
  });

  return result;
}

/**
 * To preserve a raw copy of the data, this method saves a raw version
 * in the dataset StringMap where the key of the raw data starts with "raw-"
 * before the name of the original data.
 *
 * @tparam S The size of the array data that's being used
 *
 * @param dataset A reference to the dataset being used
 */
template <size_t S>
static void CopyRawData(
    wpi::StringMap<std::vector<std::array<double, S>>>* dataset) {
  using Data = std::array<double, S>;
  auto& data = *dataset;
  // Loads the Raw Data
  for (auto& it : data) {
    auto key = it.first();
    auto& dataset = it.getValue();

    if (!wpi::contains(key, "raw")) {
      data[fmt::format("raw-{}", key)] = dataset;
      data[fmt::format("original-raw-{}", key)] = dataset;
    }
  }
}

/**
 * Assigns the forward, backward, and combined datasets for a Storage String
 * Map.
 *
 * @param dataset The Storage String Map that will store the datasets
 * @param slowForward The slow forward dataset
 * @param slowBackward The slow backward dataset
 * @param fastForward The fast forward dataset
 * @param fastBackward The fast backward dataset
 * @param prefix The prefix for the stored datasets (e.g. "Left" if you are
 *               storing data for the left side of the drivetrain)
 */
static void StoreDatasets(wpi::StringMap<Storage>* dataset,
                          const std::vector<PreparedData>& slowForward,
                          const std::vector<PreparedData>& slowBackward,
                          const std::vector<PreparedData>& fastForward,
                          const std::vector<PreparedData>& fastBackward,
                          std::string_view prefix = "") {
  std::string prefixStr;
  if (prefix != "") {
    prefixStr = fmt::format("{} ", prefix);
  }

  auto& outputData = *dataset;
  outputData[prefixStr + "Forward"] = Storage{slowForward, fastForward};
  outputData[prefixStr + "Backward"] = Storage{slowBackward, slowForward};
  outputData[prefixStr + "Combined"] =
      Storage{DataConcat(slowForward, slowBackward),
              DataConcat(fastForward, fastBackward)};
}

/**
 * Prepares data for general mechanisms (i.e. not drivetrain) and stores them
 * in the analysis manager dataset.
 *
 * @param json     A reference to the JSON containing all of the collected
 *                 data.
 * @param settings A reference to the settings being used by the analysis
 *                 manager instance.
 * @param factor   The units per rotation to multiply positions and velocities
 *                 by.
 * @param unit The name of the unit being used
 * @param originalDatasets A reference to the String Map storing the original
 *                         datasets (won't be touched in the filtering process)
 * @param rawDatasets A reference to the String Map storing the raw datasets
 * @param filteredDatasets A reference to the String Map storing the filtered
 *                         datasets
 * @param startTimes A reference to an array containing the start times for the
 *                   4 different tests
 * @param minStepTime A reference to the minimum duration of the step test as
 *                    one of the trimming procedures will remove this amount
 *                    from the start of the test.
 * @param maxStepTime A reference to the maximum duration of the step test
 *                    mainly for use in the GUI
 * @param logger A reference to a logger to help with debugging
 */
static void PrepareGeneralData(
    const wpi::json& json, AnalysisManager::Settings& settings, double factor,
    std::string_view unit, wpi::StringMap<Storage>& originalDatasets,
    wpi::StringMap<Storage>& rawDatasets,
    wpi::StringMap<Storage>& filteredDatasets,
    std::array<units::second_t, 4>& startTimes, units::second_t& minStepTime,
    units::second_t& maxStepTime, wpi::Logger& logger) {
  using Data = std::array<double, 4>;
  wpi::StringMap<std::vector<Data>> data;
  wpi::StringMap<std::vector<PreparedData>> preparedData;

  // Store the raw data columns.
  static constexpr size_t kTimeCol = 0;
  static constexpr size_t kVoltageCol = 1;
  static constexpr size_t kPosCol = 2;
  static constexpr size_t kVelCol = 3;

  WPI_INFO(logger, "{}", "Reading JSON data.");
  // Get the major components from the JSON and store them inside a StringMap.
  for (auto&& key : AnalysisManager::kJsonDataKeys) {
    data[key] = json.at(key).get<std::vector<Data>>();
  }

  WPI_INFO(logger, "{}", "Preprocessing raw data.");
  // Ensure that voltage and velocity have the same sign. Also multiply
  // positions and velocities by the factor.
  for (auto it = data.begin(); it != data.end(); ++it) {
    for (auto&& pt : it->second) {
      pt[kVoltageCol] = std::copysign(pt[kVoltageCol], pt[kVelCol]);
      pt[kPosCol] *= factor;
      pt[kVelCol] *= factor;
    }
  }

  WPI_INFO(logger, "{}", "Copying raw data.");
  CopyRawData(&data);

  WPI_INFO(logger, "{}", "Converting raw data to PreparedData struct.");
  // Convert data to PreparedData structs
  for (auto& it : data) {
    auto key = it.first();
    preparedData[key] =
        ConvertToPrepared<4, kTimeCol, kVoltageCol, kPosCol, kVelCol>(
            data[key]);
  }

  // Store the original datasets
  StoreDatasets(&originalDatasets, preparedData["original-raw-slow-forward"],
                preparedData["original-raw-slow-backward"],
                preparedData["original-raw-fast-forward"],
                preparedData["original-raw-fast-backward"]);

  WPI_INFO(logger, "{}", "Initial trimming and filtering.");
  sysid::InitialTrimAndFilter(&preparedData, settings, minStepTime, maxStepTime,
                              unit);

  WPI_INFO(logger, "{}", "Acceleration filtering.");
  sysid::AccelFilter(&preparedData);

  WPI_INFO(logger, "{}", "Storing datasets.");
  // Store the raw datasets
  StoreDatasets(&rawDatasets, preparedData["raw-slow-forward"],
                preparedData["raw-slow-backward"],
                preparedData["raw-fast-forward"],
                preparedData["raw-fast-backward"]);

  // Store the filtered datasets
  StoreDatasets(&filteredDatasets, preparedData["slow-forward"],
                preparedData["slow-backward"], preparedData["fast-forward"],
                preparedData["fast-backward"]);

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
 * @param originalDatasets A reference to the String Map storing the original
 *                         datasets (won't be touched in the filtering process)
 * @param rawDatasets A reference to the String Map storing the raw datasets
 * @param filteredDatasets A reference to the String Map storing the filtered
 *                         datasets
 * @param startTimes A reference to an array containing the start times for the
 *                   4 different tests
 * @param minStepTime A reference to the minimum duration of the step test as
 *                    one of the trimming procedures will remove this amount
 *                    from the start of the test.
 * @param maxStepTime A reference to the maximum duration of the step test
 *                    mainly for use in the GUI
 * @param logger A reference to a logger to help with debugging
 */
static void PrepareAngularDrivetrainData(
    const wpi::json& json, AnalysisManager::Settings& settings, double factor,
    std::optional<double>& trackWidth,
    wpi::StringMap<Storage>& originalDatasets,
    wpi::StringMap<Storage>& rawDatasets,
    wpi::StringMap<Storage>& filteredDatasets,
    std::array<units::second_t, 4>& startTimes, units::second_t& minStepTime,
    units::second_t& maxStepTime, wpi::Logger& logger) {
  using Data = std::array<double, 9>;
  wpi::StringMap<std::vector<Data>> data;
  wpi::StringMap<std::vector<PreparedData>> preparedData;

  // Store the relevant raw data columns.
  static constexpr size_t kTimeCol = 0;
  static constexpr size_t kLVoltageCol = 1;
  static constexpr size_t kRVoltageCol = 2;
  static constexpr size_t kLPosCol = 3;
  static constexpr size_t kRPosCol = 4;
  static constexpr size_t kAngleCol = 7;
  static constexpr size_t kAngularRateCol = 8;

  WPI_INFO(logger, "{}", "Reading JSON data.");
  // Get the major components from the JSON and store them inside a StringMap.
  for (auto&& key : AnalysisManager::kJsonDataKeys) {
    data[key] = json.at(key).get<std::vector<Data>>();
  }

  WPI_INFO(logger, "{}", "Preprocessing raw data.");
  // Ensure that voltage and velocity have the same sign. Also multiply
  // positions and velocities by the factor.
  for (auto it = data.begin(); it != data.end(); ++it) {
    for (auto&& pt : it->second) {
      pt[kLPosCol] *= factor;
      pt[kRPosCol] *= factor;

      // Store the summarized average voltages in the left voltage column
      pt[kLVoltageCol] =
          (std::copysign(pt[kLVoltageCol], pt[kAngularRateCol]) +
           std::copysign(pt[kRVoltageCol], pt[kAngularRateCol])) /
          2;
    }
  }

  WPI_INFO(logger, "{}", "Calculating trackwidth");
  // Aggregating all the deltas from all the tests
  double leftDelta = 0.0;
  double rightDelta = 0.0;
  double angleDelta = 0.0;
  for (const auto& it : data) {
    auto key = it.first();
    auto& trackWidthData = data[key];
    leftDelta += std::abs(trackWidthData.back()[kLPosCol] -
                          trackWidthData.front()[kLPosCol]);
    rightDelta += std::abs(trackWidthData.back()[kRPosCol] -
                           trackWidthData.front()[kRPosCol]);
    angleDelta += std::abs(trackWidthData.back()[kAngleCol] -
                           trackWidthData.front()[kAngleCol]);
  }
  trackWidth = sysid::CalculateTrackWidth(leftDelta, rightDelta,
                                          units::radian_t{angleDelta});

  WPI_INFO(logger, "{}", "Copying raw data.");
  CopyRawData(&data);

  WPI_INFO(logger, "{}", "Converting to PreparedData struct.");
  // Convert raw data to prepared data
  for (const auto& it : data) {
    auto key = it.first();
    preparedData[key] = ConvertToPrepared<9, kTimeCol, kLVoltageCol, kAngleCol,
                                          kAngularRateCol>(data[key]);
  }

  // To convert the angular rates into translational velocities (to work with
  // the model + OLS), v = ωr => v = ω * trackwidth / 2
  double translationalFactor = trackWidth.value() / 2.0;

  // Scale Angular Rate Data
  for (auto& it : preparedData) {
    auto& dataset = it.getValue();

    for (auto& pt : dataset) {
      pt.velocity *= translationalFactor;
      pt.acceleration *= translationalFactor;
    }
  }

  // Create the distinct datasets and store them in our StringMap
  StoreDatasets(&originalDatasets, preparedData["original-raw-slow-forward"],
                preparedData["original-raw-slow-backward"],
                preparedData["original-raw-fast-forward"],
                preparedData["original-raw-fast-backward"]);

  WPI_INFO(logger, "{}", "Applying trimming and filtering.");
  sysid::InitialTrimAndFilter(&preparedData, settings, minStepTime,
                              maxStepTime);

  WPI_INFO(logger, "{}", "Acceleration filtering.");
  sysid::AccelFilter(&preparedData);

  WPI_INFO(logger, "{}", "Storing datasets.");
  // Create the distinct datasets and store them in our StringMap.
  StoreDatasets(&rawDatasets, preparedData["raw-slow-forward"],
                preparedData["raw-slow-backward"],
                preparedData["raw-fast-forward"],
                preparedData["raw-fast-backward"]);
  StoreDatasets(&filteredDatasets, preparedData["slow-forward"],
                preparedData["slow-backward"], preparedData["fast-forward"],
                preparedData["fast-backward"]);

  startTimes = {preparedData["slow-forward"][0].timestamp,
                preparedData["slow-backward"][0].timestamp,
                preparedData["fast-forward"][0].timestamp,
                preparedData["fast-backward"][0].timestamp};
}

/**
 * Prepares data for linear drivetrain test data and stores them in
 * the analysis manager dataset.
 *
 * @param json     A reference to the JSON containing all of the collected
 *                 data.
 * @param settings A reference to the settings being used by the analysis
 *                 manager instance.
 * @param factor   The units per rotation to multiply positions and velocities
 *                 by.
 * @param originalDatasets A reference to the String Map storing the original
 *                         datasets (won't be touched in the filtering process)
 * @param rawDatasets A reference to the String Map storing the raw datasets
 * @param filteredDatasets A reference to the String Map storing the filtered
 *                         datasets
 * @param startTimes A reference to an array containing the start times for the
 *                   4 different tests
 * @param minStepTime A reference to the minimum duration of the step test as
 *                    one of the trimming procedures will remove this amount
 *                    from the start of the test.
 * @param maxStepTime A reference to the maximum duration of the step test
 *                    mainly for use in the GUI
 * @param logger A reference to a logger to help with debugging
 */
static void PrepareLinearDrivetrainData(
    const wpi::json& json, AnalysisManager::Settings& settings, double factor,
    wpi::StringMap<Storage>& originalDatasets,
    wpi::StringMap<Storage>& rawDatasets,
    wpi::StringMap<Storage>& filteredDatasets,
    std::array<units::second_t, 4>& startTimes, units::second_t& minStepTime,
    units::second_t& maxStepTime, wpi::Logger& logger) {
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
  WPI_INFO(logger, "{}", "Reading JSON data.");
  for (auto&& key : AnalysisManager::kJsonDataKeys) {
    data[key] = json.at(key).get<std::vector<Data>>();
  }

  // Ensure that voltage and velocity have the same sign. Also multiply
  // positions and velocities by the factor.
  WPI_INFO(logger, "{}", "Preprocessing raw data.");
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

  WPI_INFO(logger, "{}", "Copying raw data.");
  CopyRawData(&data);

  // Convert data to PreparedData
  WPI_INFO(logger, "{}", "Converting to PreparedData struct.");
  for (auto& it : data) {
    auto key = it.first();

    preparedData[fmt::format("left-{}", key)] =
        ConvertToPrepared<9, kTimeCol, kLVoltageCol, kLPosCol, kLVelCol>(
            data[key]);
    preparedData[fmt::format("right-{}", key)] =
        ConvertToPrepared<9, kTimeCol, kRVoltageCol, kRPosCol, kRVelCol>(
            data[key]);
  }

  // Store original data data as variables
  auto originalSlowForwardLeft = preparedData["left-original-raw-slow-forward"];
  auto originalSlowForwardRight =
      preparedData["right-original-raw-slow-forward"];
  auto originalSlowBackwardLeft =
      preparedData["left-original-raw-slow-backward"];
  auto originalSlowBackwardRight =
      preparedData["right-original-raw-slow-backward"];
  auto originalFastForwardLeft = preparedData["left-original-raw-fast-forward"];
  auto originalFastForwardRight =
      preparedData["right-original-raw-fast-forward"];
  auto originalFastBackwardLeft =
      preparedData["left-original-raw-fast-backward"];
  auto originalFastBackwardRight =
      preparedData["right-original-raw-fast-backward"];

  // Create the distinct raw datasets and store them in our StringMap.
  auto originalSlowForward =
      DataConcat(originalSlowForwardLeft, originalSlowForwardRight);
  auto originalSlowBackward =
      DataConcat(originalSlowBackwardLeft, originalSlowBackwardRight);
  auto originalFastForward =
      DataConcat(originalFastForwardLeft, originalFastForwardRight);
  auto originalFastBackward =
      DataConcat(originalFastBackwardLeft, originalFastBackwardRight);

  StoreDatasets(&originalDatasets, originalSlowForward, originalSlowBackward,
                originalFastForward, originalFastBackward);
  StoreDatasets(&originalDatasets, originalSlowForwardLeft,
                originalSlowBackwardLeft, originalFastForwardLeft,
                originalFastBackwardLeft, "Left");
  StoreDatasets(&originalDatasets, originalSlowForwardRight,
                originalSlowBackwardRight, originalFastForwardRight,
                originalFastBackwardRight, "Right");

  WPI_INFO(logger, "{}", "Applying trimming and filtering.");
  sysid::InitialTrimAndFilter(&preparedData, settings, minStepTime,
                              maxStepTime);
  // Store filtered data
  auto& slowForwardLeft = preparedData["left-slow-forward"];
  auto& slowForwardRight = preparedData["right-slow-forward"];
  auto& slowBackwardLeft = preparedData["left-slow-backward"];
  auto& slowBackwardRight = preparedData["right-slow-backward"];
  auto& fastForwardLeft = preparedData["left-fast-forward"];
  auto& fastForwardRight = preparedData["right-fast-forward"];
  auto& fastBackwardLeft = preparedData["left-fast-backward"];
  auto& fastBackwardRight = preparedData["right-fast-backward"];

  auto slowForward = DataConcat(slowForwardLeft, slowForwardRight);
  auto slowBackward = DataConcat(slowBackwardLeft, slowBackwardRight);
  auto fastForward = DataConcat(fastForwardLeft, fastForwardRight);
  auto fastBackward = DataConcat(fastBackwardLeft, fastBackwardRight);

  WPI_INFO(logger, "{}", "Acceleration filtering.");
  sysid::AccelFilter(&preparedData);

  WPI_INFO(logger, "{}", "Storing datasets.");
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
  auto rawSlowForward = DataConcat(rawSlowForwardLeft, rawSlowForwardRight);
  auto rawSlowBackward = DataConcat(rawSlowBackwardLeft, rawSlowBackwardRight);
  auto rawFastForward = DataConcat(rawFastForwardLeft, rawFastForwardRight);
  auto rawFastBackward = DataConcat(rawFastBackwardLeft, rawFastBackwardRight);

  StoreDatasets(&rawDatasets, rawSlowForward, rawSlowBackward, rawFastForward,
                rawFastBackward);
  StoreDatasets(&rawDatasets, rawSlowForwardLeft, rawSlowBackwardLeft,
                rawFastForwardLeft, rawFastBackwardLeft, "Left");
  StoreDatasets(&rawDatasets, rawSlowForwardRight, rawSlowBackwardRight,
                rawFastForwardRight, rawFastBackwardRight, "Right");

  // Create the distinct filtered datasets and store them in our StringMap.
  StoreDatasets(&filteredDatasets, slowForward, slowBackward, fastForward,
                fastBackward);
  StoreDatasets(&filteredDatasets, slowForwardLeft, slowBackwardLeft,
                fastForwardLeft, fastBackwardLeft, "Left");
  StoreDatasets(&filteredDatasets, slowForwardRight, slowBackwardRight,
                fastForwardRight, fastBackwardRight, "Right");

  startTimes = {
      rawSlowForward.front().timestamp, rawSlowBackward.front().timestamp,
      rawFastForward.front().timestamp, rawFastBackward.front().timestamp};
}

AnalysisManager::AnalysisManager(std::string_view path, Settings& settings,
                                 wpi::Logger& logger)
    : m_settings(settings), m_logger(logger) {
  {
    // Read JSON from the specified path
    std::error_code ec;
    wpi::raw_fd_istream is{path, ec};

    if (ec) {
      throw std::runtime_error(fmt::format("Unable to read: {}", path));
    }

    is >> m_json;

    WPI_INFO(m_logger, "Read {}", path);
  }

  // Check that we have a sysid JSON
  if (m_json.find("sysid") == m_json.end()) {
    // If it's not a sysid JSON, try converting it from frc-char format
    std::string newPath = sysid::ConvertJSON(path, logger);

    // Read JSON from the specified path
    std::error_code ec;
    wpi::raw_fd_istream is{newPath, ec};

    if (ec) {
      throw std::runtime_error(fmt::format("Unable to read: {}", newPath));
    }

    is >> m_json;

    WPI_INFO(m_logger, "Read {}", newPath);
  }

  WPI_INFO(m_logger, "Parsing initial data of {}", path);
  // Get the analysis type from the JSON.
  m_type = sysid::analysis::FromName(m_json.at("test").get<std::string>());

  // Get the rotation -> output units factor from the JSON.
  m_unit = m_json.at("units").get<std::string>();
  m_factor = m_json.at("unitsPerRotation").get<double>();
  WPI_DEBUG(m_logger, "Parsing units per rotation as {} {} per rotation",
            m_factor, m_unit);

  // Reset settings for Dynamic Test Limits
  m_settings.stepTestDuration = units::second_t{0.0};
  m_minDuration = units::second_t{100000};
}

void AnalysisManager::PrepareData() {
  WPI_INFO(m_logger, "Preparing {} data", m_type.name);
  if (m_type == analysis::kDrivetrain) {
    PrepareLinearDrivetrainData(m_json, m_settings, m_factor,
                                m_originalDatasets, m_rawDatasets,
                                m_filteredDatasets, m_startTimes, m_minDuration,
                                m_maxDuration, m_logger);
  } else if (m_type == analysis::kDrivetrainAngular) {
    PrepareAngularDrivetrainData(m_json, m_settings, m_factor, m_trackWidth,
                                 m_originalDatasets, m_rawDatasets,
                                 m_filteredDatasets, m_startTimes,
                                 m_minDuration, m_maxDuration, m_logger);
  } else {
    PrepareGeneralData(m_json, m_settings, m_factor, m_unit, m_originalDatasets,
                       m_rawDatasets, m_filteredDatasets, m_startTimes,
                       m_minDuration, m_maxDuration, m_logger);
  }
  WPI_INFO(m_logger, "{}", "Finished Preparing Data");
}

AnalysisManager::Gains AnalysisManager::Calculate() {
  WPI_INFO(m_logger, "{}", "Calculating Gains");
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

void AnalysisManager::OverrideUnits(std::string_view unit,
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
