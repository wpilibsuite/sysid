// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

#include <frc/MedianFilter.h>
#include <units/time.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/Storage.h"

namespace sysid {

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
 * Calculates the expected acceleration noise to be used as the floor of the
 * Voltage Trim. This is done by taking the standard deviation from the moving
 * average values of each point.
 *
 * @param data the prepared data vector containing acceleration data
 * @param window the size of the window for the moving average
 */
double GetAccelNoiseFloor(const std::vector<PreparedData>& data, int window);

/**
 * Reduces noise in velocity data by applying a median filter.
 *
 * @tparam S The size of the raw data array
 * @tparam Velocity The index of the velocity entry in the raw data.
 *
 * @param data the vector of arrays representing sysid data (must contain
 * velocity data)
 * @param window the size of the window of the median filter (must be odd)
 */
template <size_t S, size_t Velocity>
void ApplyMedianFilter(std::vector<std::array<double, S>>* data, int window) {
  size_t step = window / 2;
  std::vector<std::array<double, S>> prepared;
  frc::MedianFilter<double> medianFilter(window);
  for (size_t i = 0; i < data->size(); i++) {
    double median = medianFilter.Calculate(data->at(i)[Velocity]);
    if (i > step) {
      std::array<double, S> updateData{data->at(i - step)};
      updateData[Velocity] = median;
      prepared.push_back(updateData);
    }
  }
  *data = std::move(prepared);
}

/**
 * Trims the step voltage data to discard all points before the maximum
 * acceleration and after reaching stead-state velocity. Also trims the end of
 * the test based off of user specified test durations, but it will determine a
 * default duration if the requested duration is less than the minimum step test
 * duration.
 *
 * @param data A pointer to the step voltage data.
 * @param settings A pointer to the settings of an analysis manager object.
 * @param minStepTime The current minimum step test duration.
 * @param maxStepTime The maximum step test duration.
 *
 * @return The updated minimum step test duration.
 */
units::second_t TrimStepVoltageData(std::vector<PreparedData>* data,
                                    AnalysisManager::Settings* settings,
                                    units::second_t minStepTime,
                                    units::second_t maxStepTime);

/**
 * Compute the mean time delta of the given data.
 */
units::second_t GetMeanTimeDelta(const Storage& data);

}  // namespace sysid
