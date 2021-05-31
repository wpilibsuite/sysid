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
#include <stdexcept>
#include <string_view>
#include <vector>

#include <fmt/format.h>
#include <units/angle.h>
#include <units/math.h>
#include <wpi/StringMap.h>
#include <wpi/json.h>
#include <wpi/numbers>
#include <wpi/raw_istream.h>

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

template <size_t S>
static void CopyRawData(
    wpi::StringMap<std::vector<std::array<double, S>>>* dataset) {
  using Data = std::array<double, S>;
  auto& data = *dataset;
  // Loads the Raw Data
  sysid::ApplyToData(
      data,
      [&](std::string_view key) {
        data[fmt::format("raw-{}", key)] = std::vector<Data>(data[key]);
      },
      [](std::string_view key) {
        return key.find("raw") == std::string_view::npos;
      });
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
      Storage{Concatenate(slowForward, {&slowBackward}),
              Concatenate(fastForward, {&fastBackward})};
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
                               double factor, std::string_view unit,
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
  CopyRawData(&data);

  // Convert data to PreparedData structs
  sysid::ApplyToData(data, [&](std::string_view key) {
    preparedData[key] =
        ConvertToPrepared<4, kTimeCol, kVoltageCol, kPosCol, kVelCol>(
            data[key]);
  });

  sysid::InitialTrimAndFilter(&preparedData, settings, minStepTime, maxStepTime,
                              unit);
  // Compute mean dt
  auto& slowForward = preparedData["slow-forward"];
  auto& slowBackward = preparedData["slow-backward"];
  auto& fastForward = preparedData["fast-forward"];
  auto& fastBackward = preparedData["fast-backward"];
  Storage tempCombined{Concatenate(slowForward, {&slowBackward}),
                       Concatenate(fastForward, {&fastBackward})};

  sysid::AccelAndTimeFilter(&preparedData, tempCombined);

  // Store the raw datasets
  StoreDatasets(&rawDatasets, preparedData["raw-slow-forward"],
                preparedData["raw-slow-backward"],
                preparedData["raw-fast-forward"],
                preparedData["raw-fast-backward"]);

  // Store the filtered datasets
  StoreDatasets(&filteredDatasets, slowForward, slowBackward, fastForward,
                fastBackward);

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
  static constexpr size_t kLVoltageCol = 1;
  static constexpr size_t kRVoltageCol = 2;
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
      pt[kLPosCol] *= factor;
      pt[kRPosCol] *= factor;

      // Store the summarized average voltages in the left voltage column
      pt[kLVoltageCol] =
          (std::copysign(pt[kLVoltageCol], pt[kAngularRateCol]) +
           std::copysign(pt[kRVoltageCol], pt[kAngularRateCol])) /
          2;
    }
  }

  // Load Raw Data
  CopyRawData(&data);

  // Convert raw data to prepared data
  sysid::ApplyToData(data, [&](std::string_view key) {
    preparedData[key] = ConvertToPrepared<9, kTimeCol, kLVoltageCol, kAngleCol,
                                          kAngularRateCol>(data[key]);
  });

  sysid::InitialTrimAndFilter(&preparedData, settings, minStepTime,
                              maxStepTime);
  // Compute mean dt
  auto& slowForward = preparedData["slow-forward"];
  auto& slowBackward = preparedData["slow-backward"];
  auto& fastForward = preparedData["fast-forward"];
  auto& fastBackward = preparedData["fast-backward"];
  Storage tempCombined{Concatenate(slowForward, {&slowBackward}),
                       Concatenate(fastForward, {&fastBackward})};

  sysid::AccelAndTimeFilter(&preparedData, tempCombined);

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

  // To convert the angular rates into translational velocities (to work with
  // the model + OLS), v = ωr => v = ω * trackwidth / 2
  double translationalFactor = trackWidth.value() / 2.0;

  // Scale Angular Rate Data
  sysid::ApplyToData(preparedData, [&](std::string_view key) {
    for (auto& pt : preparedData[key]) {
      pt.velocity *= translationalFactor;
      pt.nextVelocity *= translationalFactor;
      pt.acceleration *= translationalFactor;
    }
  });

  // Create the distinct datasets and store them in our StringMap.
  StoreDatasets(&rawDatasets, preparedData["raw-slow-forward"],
                preparedData["raw-slow-backward"],
                preparedData["raw-fast-forward"],
                preparedData["raw-fast-backward"]);
  StoreDatasets(&filteredDatasets, slowForward, slowBackward, fastForward,
                fastBackward);

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
  CopyRawData(&data);

  // Convert data to PreparedData
  sysid::ApplyToData(data, [&](std::string_view key) {
    preparedData[fmt::format("left-{}", key)] =
        ConvertToPrepared<9, kTimeCol, kLVoltageCol, kLPosCol, kLVelCol>(
            data[key]);
    preparedData[fmt::format("right-{}", key)] =
        ConvertToPrepared<9, kTimeCol, kRVoltageCol, kRPosCol, kRVelCol>(
            data[key]);
  });

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

  auto slowForward = Concatenate(slowForwardLeft, {&slowForwardRight});
  auto slowBackward = Concatenate(slowBackwardLeft, {&slowBackwardRight});
  auto fastForward = Concatenate(fastForwardLeft, {&fastForwardRight});
  auto fastBackward = Concatenate(fastBackwardLeft, {&fastBackwardRight});

  // Compute mean dt
  Storage tempCombined{Concatenate(slowForward, {&slowBackward}),
                       Concatenate(fastForward, {&fastBackward})};

  sysid::AccelAndTimeFilter(&preparedData, tempCombined);

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
  // Read JSON from the specified path.
  std::error_code ec;
  wpi::raw_fd_istream is{path, ec};

  if (ec) {
    throw std::runtime_error(fmt::format("Unable to read: {}", path));
  }

  is >> m_json;
  WPI_INFO(m_logger, "Read {}", path);

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
