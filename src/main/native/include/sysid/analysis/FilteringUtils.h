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

#include <units/time.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/Storage.h"

namespace sysid {

/**
 * Trims quasistatic data so that no point has a voltage of zero or a velocity
 * less than the motion threshold.
 *
 * @param data            A pointer to the vector of the prepared data.
 * @param motionThreshold The velocity threshold under which to delete data.
 */
void TrimQuasistaticData(std::vector<PreparedData>* data,
                         double motionThreshold);

/**
 * Calculates the expected acceleration noise to be used as the floor of the
 * Voltage Trim. This is done by taking the standard deviation from the moving
 * average values of each point.
 *
 * @param data the prepared data vector containing acceleration data
 * @param window the size of the window for the moving average
 *
 * @return The expected acceleration noise
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
void ApplyMedianFilter(std::vector<PreparedData>* data, int window);

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
 *
 * @param data A reference to all of the collected PreparedData
 *
 * @return The mean time delta for all the data points
 */
units::second_t GetMeanTimeDelta(const Storage& data);

/**
 * Filters out data with acceleration = 0
 *
 * @param data A pointer to a PreparedData vector that needs acceleration
 *             filtering.
 */
void FilterAccelData(std::vector<PreparedData>* data);

}  // namespace sysid
