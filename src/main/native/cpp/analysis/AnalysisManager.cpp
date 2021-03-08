// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/AnalysisManager.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <system_error>

#include <units/angle.h>
#include <wpi/StringMap.h>
#include <wpi/json.h>
#include <wpi/math>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/JSONConverter.h"
#include "sysid/analysis/TrackWidthAnalysis.h"

using namespace sysid;

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
 * Trims quasistatic data so that no point has a voltage of zero or a velocity
 * less than the motion threshold.
 *
 * @tparam S        The size of the raw data array.
 * @tparam Voltage  The index of the voltage entry in the raw data.
 * @tparam Velocity The index of the velocity entry in the raw data.
 *
 * @param data            A pointer to the vector of raw data.
 * @param motionThreshold The velocity threshold under which to delete data.
 */
template <size_t S, size_t Voltage, size_t Velocity>
void TrimQuasistaticData(std::vector<std::array<double, S>>* data,
                         double motionThreshold) {
  data->erase(std::remove_if(data->begin(), data->end(),
                             [motionThreshold](const auto& pt) {
                               return std::abs(pt[Voltage]) <= 0 ||
                                      std::abs(pt[Velocity]) < motionThreshold;
                             }),
              data->end());
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
  prepared.reserve(data.size() - window);

  // Compute acceleration and add it to the vector.
  for (size_t i = step; i < data.size() - step; ++i) {
    const auto& pt = data[i];
    double acc = (data[i + step][Velocity] - data[i - step][Velocity]) /
                 (data[i + step][0] - data[i - step][0]);

    // Sometimes, if the encoder velocities are the same, it will register zero
    // acceleration. Do not include these values.
    if (acc != 0) {
      prepared.push_back(PreparedData{pt[0], pt[Voltage], pt[Position],
                                      pt[Velocity], acc, 0.0});
    }
  }
  return prepared;
}

/**
 * Trims the step voltage data to discard all points before the maximum
 * acceleration.
 *
 * @param data A pointer to the step voltage data.
 */
void TrimStepVoltageData(std::vector<PreparedData>* data) {
  // We want to find the point where the acceleration data roughly stops
  // decreasing at the beginning.
  size_t idx = 0;

  // We will use this to make sure that the acceleration is decreasing for 3
  // consecutive entries in a row. This will help avoid false positives from
  // bad data.
  bool caution = false;

  for (size_t i = 0; i < data->size(); ++i) {
    // Get the current acceleration.
    double acc = data->at(i).acceleration;

    // If we are not in caution, the acceleration values are still increasing.
    if (!caution) {
      if (acc < std::abs(data->at(idx).acceleration)) {
        // We found a potential candidate. Let's mark the flag and continue
        // checking...
        caution = true;
      } else {
        // Set the current acceleration to be the highest so far.
        idx = i;
      }
    } else {
      // Check to make sure the acceleration value is still smaller. If it
      // isn't, break out of caution.
      if (acc >= std::abs(data->at(idx).acceleration)) {
        caution = false;
        idx = i;
      }
    }

    // If we were in caution for three iterations, we can exit.
    if (caution && (i - idx) == 3) {
      break;
    }
  }
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
                               const AnalysisManager::Settings& settings,
                               double factor, wpi::StringRef unit,
                               wpi::StringMap<Storage>& datasets) {
  using Data = std::array<double, 4>;
  wpi::StringMap<std::vector<Data>> data;

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

  // Trim quasistatic test data to remove all points where voltage is zero or
  // velocity < motion threshold.
  TrimQuasistaticData<4, kVoltageCol, kVelCol>(&data["slow-forward"],
                                               settings.motionThreshold);
  TrimQuasistaticData<4, kVoltageCol, kVelCol>(&data["slow-backward"],
                                               settings.motionThreshold);

  // Compute acceleration on all data sets.
  auto sf = ComputeAcceleration<4, kVoltageCol, kPosCol, kVelCol>(
      data["slow-forward"], settings.windowSize);
  auto sb = ComputeAcceleration<4, kVoltageCol, kPosCol, kVelCol>(
      data["slow-backward"], settings.windowSize);
  auto ff = ComputeAcceleration<4, kVoltageCol, kPosCol, kVelCol>(
      data["fast-forward"], settings.windowSize);
  auto fb = ComputeAcceleration<4, kVoltageCol, kPosCol, kVelCol>(
      data["fast-backward"], settings.windowSize);

  // Calculate cosine of position data.
  CalculateCosine(&sf, unit);
  CalculateCosine(&sb, unit);
  CalculateCosine(&ff, unit);
  CalculateCosine(&fb, unit);

  // Trim the step voltage data.
  TrimStepVoltageData(&ff);
  TrimStepVoltageData(&fb);

  // Create the distinct datasets and store them in our StringMap.
  datasets["Forward"] = std::make_tuple(sf, ff);
  datasets["Backward"] = std::make_tuple(sb, fb);
  datasets["Combined"] =
      std::make_tuple(Concatenate(sf, {&sb}), Concatenate(ff, {&fb}));
}

/**
 * Prepares data for angular drivetrain test data and stores them in
 * the analysis manager dataset.
 *
 * @param json     A reference to the JSON containing all of the collected
 * data.
 * @param settings A reference to the settings being used by the analysis
 *                 manager instance.
 * @param factor   The units per rotation to multiply positions and velocities
 *                 by.
 * @param tw       A reference to the std::optional where the track width will
 *                 be stored.
 * @param datasets A reference to the datasets object of the relevant analysis
 *                 manager instance.
 */
static void PrepareAngularDrivetrainData(
    const wpi::json& json, const AnalysisManager::Settings& settings,
    double factor, std::optional<double>& tw,
    wpi::StringMap<Storage>& datasets) {
  using Data = std::array<double, 9>;
  wpi::StringMap<std::vector<Data>> data;

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
  TrimQuasistaticData<9, kVoltageCol, kAngularRateCol>(
      &data["slow-forward"], settings.motionThreshold);
  TrimQuasistaticData<9, kVoltageCol, kAngularRateCol>(
      &data["slow-backward"], settings.motionThreshold);

  // Compute acceleration on all data sets.
  auto sf = ComputeAcceleration<9, kVoltageCol, kAngleCol, kAngularRateCol>(
      data["slow-forward"], settings.windowSize);
  auto sb = ComputeAcceleration<9, kVoltageCol, kAngleCol, kAngularRateCol>(
      data["slow-backward"], settings.windowSize);
  auto ff = ComputeAcceleration<9, kVoltageCol, kAngleCol, kAngularRateCol>(
      data["fast-forward"], settings.windowSize);
  auto fb = ComputeAcceleration<9, kVoltageCol, kAngleCol, kAngularRateCol>(
      data["fast-backward"], settings.windowSize);

  // Trim the step voltage data.
  TrimStepVoltageData(&ff);
  TrimStepVoltageData(&fb);

  // Calculate track width from the slow-forward raw data.
  auto& twd = data["slow-forward"];
  double l = twd.back()[kLPosCol] - twd.front()[kLPosCol];
  double r = twd.back()[kRPosCol] - twd.front()[kRPosCol];
  double a = twd.back()[kAngleCol] - twd.front()[kAngleCol];
  tw = sysid::CalculateTrackWidth(l, r, units::radian_t(a));

  // Create the distinct datasets and store them in our StringMap.
  datasets["Forward"] = std::make_tuple(sf, ff);
  datasets["Backward"] = std::make_tuple(sb, fb);
  datasets["Combined"] =
      std::make_tuple(Concatenate(sf, {&sb}), Concatenate(ff, {&fb}));
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
    const wpi::json& json, const AnalysisManager::Settings& settings,
    double factor, wpi::StringMap<Storage>& datasets) {
  using Data = std::array<double, 9>;
  wpi::StringMap<std::vector<Data>> data;

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

  // Trim quasistatic test data to remove all points where voltage is zero or
  // velocity < motion threshold.
  TrimQuasistaticData<9, kLVoltageCol, kLVelCol>(&data["slow-forward"],
                                                 settings.motionThreshold);
  TrimQuasistaticData<9, kLVoltageCol, kLVelCol>(&data["slow-backward"],
                                                 settings.motionThreshold);
  TrimQuasistaticData<9, kRVoltageCol, kRVelCol>(&data["slow-forward"],
                                                 settings.motionThreshold);
  TrimQuasistaticData<9, kRVoltageCol, kRVelCol>(&data["slow-backward"],
                                                 settings.motionThreshold);

  // Compute acceleration on all data sets.
  auto sfl = ComputeAcceleration<9, kLVoltageCol, kLPosCol, kLVelCol>(
      data["slow-forward"], settings.windowSize);
  auto sbl = ComputeAcceleration<9, kLVoltageCol, kLPosCol, kLVelCol>(
      data["slow-backward"], settings.windowSize);
  auto ffl = ComputeAcceleration<9, kLVoltageCol, kLPosCol, kLVelCol>(
      data["fast-forward"], settings.windowSize);
  auto fbl = ComputeAcceleration<9, kLVoltageCol, kLPosCol, kLVelCol>(
      data["fast-backward"], settings.windowSize);
  auto sfr = ComputeAcceleration<9, kRVoltageCol, kRPosCol, kRVelCol>(
      data["slow-forward"], settings.windowSize);
  auto sbr = ComputeAcceleration<9, kRVoltageCol, kRPosCol, kRVelCol>(
      data["slow-backward"], settings.windowSize);
  auto ffr = ComputeAcceleration<9, kRVoltageCol, kRPosCol, kRVelCol>(
      data["fast-forward"], settings.windowSize);
  auto fbr = ComputeAcceleration<9, kRVoltageCol, kRPosCol, kRVelCol>(
      data["fast-backward"], settings.windowSize);

  // Trim the step voltage data.
  TrimStepVoltageData(&ffl);
  TrimStepVoltageData(&ffr);
  TrimStepVoltageData(&fbl);
  TrimStepVoltageData(&fbr);

  // Create the distinct datasets and store them in our StringMap.
  auto sf = Concatenate(sfl, {&sfr});
  auto sb = Concatenate(sbl, {&sbr});
  auto ff = Concatenate(ffl, {&ffr});
  auto fb = Concatenate(fbl, {&fbr});

  datasets["Forward"] = std::make_tuple(sf, ff);
  datasets["Backward"] = std::make_tuple(sb, fb);
  datasets["Combined"] =
      std::make_tuple(Concatenate(sf, {&sb}), Concatenate(ff, {&fb}));

  datasets["Left Forward"] = std::make_tuple(sfl, ffl);
  datasets["Left Backward"] = std::make_tuple(sbl, fbl);
  datasets["Left Combined"] =
      std::make_tuple(Concatenate(sfl, {&sbl}), Concatenate(ffl, {&fbl}));

  datasets["Right Forward"] = std::make_tuple(sfr, ffr);
  datasets["Right Backward"] = std::make_tuple(sbr, fbr);
  datasets["Right Combined"] =
      std::make_tuple(Concatenate(sfr, {&sbr}), Concatenate(ffr, {&fbr}));
}

AnalysisManager::AnalysisManager(wpi::StringRef path, const Settings& settings,
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

    // Prepare data.
    PrepareData();
  }
}

void AnalysisManager::PrepareData() {
  if (m_type == analysis::kDrivetrain) {
    PrepareLinearDrivetrainData(m_json, m_settings, m_factor, m_datasets);
  } else if (m_type == analysis::kDrivetrainAngular) {
    PrepareAngularDrivetrainData(m_json, m_settings, m_factor, m_trackWidth,
                                 m_datasets);
  } else {
    PrepareGeneralData(m_json, m_settings, m_factor, m_unit, m_datasets);
  }
}

AnalysisManager::Gains AnalysisManager::Calculate() {
  // Calculate feedforward gains from the data.
  auto ff = sysid::CalculateFeedforwardGains(
      m_datasets[kDatasets[m_settings.dataset]], m_type);

  // Create the struct that we need for feedback analysis.
  auto& f = std::get<0>(ff);
  FeedforwardGains gains = {f[0], f[1], f[2]};

  // Calculate the appropriate gains.
  std::tuple<double, double> fb;
  if (m_settings.type == FeedbackControllerLoopType::kPosition) {
    fb = sysid::CalculatePositionFeedbackGains(
        m_settings.preset, m_settings.lqr, gains,
        m_settings.convertGainsToEncTicks
            ? m_settings.gearing * m_settings.cpr * m_factor
            : 1);
  } else {
    fb = sysid::CalculateVelocityFeedbackGains(
        m_settings.preset, m_settings.lqr, gains,
        m_settings.convertGainsToEncTicks
            ? m_settings.gearing * m_settings.cpr * m_factor
            : 1);
  }
  return {ff, fb, m_trackWidth};
}

void AnalysisManager::OverrideUnits(const std::string& unit,
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
