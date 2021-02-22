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
#include "sysid/analysis/TrackWidthAnalysis.h"

using namespace sysid;

/**
 * Concatenates a list of vectors to the end of a vector. The contents of the
 * source vectors are copied (not moved) into the new vector.
 */
std::vector<PreparedData> Concatenate(
    std::vector<PreparedData> dest,
    std::initializer_list<const std::vector<PreparedData>*> srcs) {
  // Copy the contents of the source vectors into the dest vector.
  for (auto ptr : srcs) {
    dest.insert(dest.end(), ptr->cbegin(), ptr->cend());
  }

  // Return the dest vector.
  return dest;
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

  // Get the analysis type from the JSON.
  m_type = sysid::analysis::FromName(m_json.at("test").get<std::string>());

  // Get the rotation -> output units factor from the JSON.
  m_unit = m_json.at("units").get<std::string>();
  m_factor = m_json.at("unitsPerRotation").get<double>();

  // Check if we have a track width value.
  m_hasTrackWidth = m_json.find("track-width") != m_json.end();

  // Prepare data.
  PrepareData();
}

// Helper function that prepares data for drivetrain.
void AnalysisManager::PrepareDataDrivetrain(
    wpi::StringMap<std::vector<RawData>>&& data) {
  // Compute acceleration on all datasets.
  int window = m_settings.windowSize;
  auto sfl = ComputeAcceleration(data["slow-forward"], window);
  auto sfr = ComputeAcceleration(data["slow-forward"], window, true);
  auto sbl = ComputeAcceleration(data["slow-backward"], window);
  auto sbr = ComputeAcceleration(data["slow-backward"], window, true);
  auto ffl = ComputeAcceleration(data["fast-forward"], window);
  auto ffr = ComputeAcceleration(data["fast-forward"], window, true);
  auto fbl = ComputeAcceleration(data["fast-backward"], window);
  auto fbr = ComputeAcceleration(data["fast-backward"], window, true);

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

  m_datasets["Forward"] = std::make_tuple(sf, ff);
  m_datasets["Backward"] = std::make_tuple(sb, fb);
  m_datasets["Combined"] =
      std::make_tuple(Concatenate(sf, {&sb}), Concatenate(ff, {&fb}));

  m_datasets["Left Forward"] = std::make_tuple(sfl, ffl);
  m_datasets["Left Backward"] = std::make_tuple(sbl, fbl);
  m_datasets["Left Combined"] =
      std::make_tuple(Concatenate(sfl, {&sbl}), Concatenate(ffl, {&fbl}));

  m_datasets["Right Forward"] = std::make_tuple(sfr, ffr);
  m_datasets["Right Backward"] = std::make_tuple(sbr, fbr);
  m_datasets["Right Combined"] =
      std::make_tuple(Concatenate(sfr, {&sbr}), Concatenate(ffr, {&fbr}));
}

void AnalysisManager::PrepareData() {
  // Get the major components from the JSON and store them inside a StringMap.
  wpi::StringMap<std::vector<RawData>> data;
  for (auto&& key : kJsonDataKeys) {
    data[key] = m_json.at(key).get<std::vector<RawData>>();
  }

  // Ensure that voltage and velocity have the same sign; apply conversion
  // factor.
  for (auto it = data.begin(); it != data.end(); ++it) {
    for (auto&& pt : it->second) {
      pt[3] = std::copysign(pt[Cols::kLVolts], pt[Cols::kLVel]);
      pt[4] = std::copysign(pt[Cols::kRVolts], pt[Cols::kRVel]);
      pt[5] *= m_factor;
      pt[6] *= m_factor;
      pt[7] *= m_factor;
      pt[8] *= m_factor;
    }
  }

  // Trim quasistatic test data to remove all points where voltage == 0 or
  // velocity < threshold.
  TrimQuasistaticData(&data["slow-forward"], m_settings.motionThreshold);
  TrimQuasistaticData(&data["slow-backward"], m_settings.motionThreshold);

  // If the type is drivetrain, we can use our special function from here.
  if (m_type == analysis::kDrivetrain) {
    PrepareDataDrivetrain(std::move(data));
    return;
  }

  // Compute acceleration on all datasets.
  auto sf = ComputeAcceleration(data["slow-forward"], m_settings.windowSize);
  auto sb = ComputeAcceleration(data["slow-backward"], m_settings.windowSize);
  auto ff = ComputeAcceleration(data["fast-forward"], m_settings.windowSize);
  auto fb = ComputeAcceleration(data["fast-backward"], m_settings.windowSize);

  // Trim the step voltage data.
  TrimStepVoltageData(&ff);
  TrimStepVoltageData(&fb);

  // If the analysis type is kArm, then we need to calculate the cosine of the
  // positions.
  if (m_type == analysis::kArm) {
    CalculateCosine(&sf, m_unit);
    CalculateCosine(&sb, m_unit);
    CalculateCosine(&ff, m_unit);
    CalculateCosine(&fb, m_unit);
  }

  // Create the distinct datasets and store them in our StringMap.
  m_datasets["Forward"] = std::make_tuple(sf, ff);
  m_datasets["Backward"] = std::make_tuple(sb, fb);
  m_datasets["Combined"] =
      std::make_tuple(Concatenate(sf, {&sb}), Concatenate(ff, {&fb}));
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
            ? m_settings.gearing * m_settings.epr * m_factor
            : 1);
  } else {
    fb = sysid::CalculateVelocityFeedbackGains(
        m_settings.preset, m_settings.lqr, gains,
        m_settings.convertGainsToEncTicks
            ? m_settings.gearing * m_settings.epr * m_factor
            : 1);
  }

  // Calculate track width if applicable.
  if (m_hasTrackWidth) {
    auto data = m_json.at("track-width").get<std::vector<RawData>>();

    double l =
        (data.back()[Cols::kLPos] - data.front()[Cols::kLPos]) * m_factor;
    double r =
        (data.back()[Cols::kRPos] - data.front()[Cols::kRPos]) * m_factor;
    double a = (data.back()[Cols::kGyro] - data.front()[Cols::kGyro]);

    return {ff, fb,
            std::make_optional(
                sysid::CalculateTrackWidth(l, r, units::radian_t(a)))};
  }

  return {ff, fb};
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

void AnalysisManager::TrimQuasistaticData(std::vector<RawData>* data,
                                          double threshold, bool drivetrain) {
  data->erase(
      std::remove_if(data->begin(), data->end(),
                     [drivetrain, threshold](const auto& pt) {
                       // Calculate abs value of voltages.
                       double lvolts = std::abs(pt[Cols::kLVolts]);
                       double rvolts = std::abs(pt[Cols::kRVolts]);

                       // Calculate abs value of velocities.
                       double lvelocity = std::abs(pt[Cols::kLVel]);
                       double rvelocity = std::abs(pt[Cols::kRVel]);

                       // Calculate primary and secondary conditions (secondary
                       // is for drivetrain).
                       bool primary = lvolts <= 0 || lvelocity <= threshold;
                       bool secondary = rvolts <= 0 || rvelocity <= threshold;

                       // Return the condition, depending on whether we want
                       // secondary or not.
                       return primary || (drivetrain ? secondary : false);
                     }),
      data->end());
}

std::vector<PreparedData> AnalysisManager::ComputeAcceleration(
    const std::vector<RawData>& data, int window, bool right) {
  size_t step = window / 2;
  std::vector<PreparedData> prepared;

  if (data.size() <= window) {
    throw std::runtime_error(
        "The data collected is too small! This can be caused by too high of a "
        "motion threshold or bad data collection.");
  }

  prepared.reserve(data.size() - window);

  if (data.size() < static_cast<size_t>(window)) {
    throw std::runtime_error("The size of the data is too small.");
  }

  size_t volts = right ? Cols::kRVolts : Cols::kLVolts;
  size_t pos = right ? Cols::kRPos : Cols::kLPos;
  size_t vel = right ? Cols::kRVel : Cols::kLVel;

  // Compute acceleration and add it to the vector.
  for (size_t i = step; i < data.size() - step; ++i) {
    auto& pt = data[i];
    double acc =
        (data[i + step][vel] - data[i - step][vel]) /
        (data[i + step][Cols::kTimestamp] - data[i - step][Cols::kTimestamp]);

    // Sometimes, if the encoder velocities are the same, it will register
    // zero acceleration. Do not include these values.
    if (acc != 0) {
      prepared.push_back(
          {pt[Cols::kTimestamp], pt[volts], pt[pos], pt[vel], acc, 0.0});
    }
  }

  return prepared;
}

void AnalysisManager::TrimStepVoltageData(std::vector<PreparedData>* data) {
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

void AnalysisManager::CalculateCosine(std::vector<PreparedData>* data,
                                      const std::string& unit) {
  for (auto&& pt : *data) {
    if (unit == "Radians") {
      pt.cos = std::cos(pt.position);
    } else if (unit == "Degrees") {
      pt.cos = std::cos(pt.position * wpi::math::pi / 180.0);
    } else if (unit == "Rotations") {
      pt.cos = std::cos(pt.position * 2 * wpi::math::pi);
    } else {
      throw std::runtime_error("The unit is not supported");
    }
  }
}
