// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string_view>
#include <utility>
#include <vector>

#include <units/time.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/Storage.h"

namespace sysid {

/**
 * Calculates the expected acceleration noise to be used as the floor of the
 * Voltage Trim. This is done by taking the standard deviation from the moving
 * average values of each point.
 *
 * @param data the prepared data vector containing acceleration data
 * @param window the size of the window for the moving average
 * @return The expected acceleration noise
 */
double GetAccelNoiseFloor(const std::vector<PreparedData>& data, int window);

/**
 * Trims the step voltage data to discard all points before the maximum
 * acceleration and after reaching stead-state velocity. Also trims the end of
 * the test based off of user specified test durations, but it will determine a
 * default duration if the requested duration is less than the minimum step test
 * duration.
 *
 * @param data A pointer to the step voltage data.
 * @param settings A pointer to the settings of an analysis manager object.
 * @param minStepTime The current minimum step test duration as one of the
 *                    trimming procedures will remove this amount from the start
 *                    of the test.
 * @param maxStepTime The maximum step test duration.
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
 * @return The mean time delta for all the data points
 */
units::second_t GetMeanTimeDelta(const std::vector<PreparedData>& data);

/**
 * Compute the mean time delta of the given data.
 *
 * @param data A reference to all of the collected PreparedData
 * @return The mean time delta for all the data points
 */
units::second_t GetMeanTimeDelta(const Storage& data);

/**
 * Computes the first derivative of f(x_i) via central finite difference.
 *
 * @tparam order The order of accuracy. The window size is order + 1.
 * @param f      The function for which to compute the finite difference.
 * @param i      The point around which to compute the finite difference.
 * @param h      The grid spacing (e.g., x_{i+1} - x_i for all i).
 */
template <size_t Order, typename F>
constexpr double CentralFiniteDifference(F&& f, size_t i, double h) {
  static_assert(Order % 2 == 0 && Order >= 2 && Order <= 8,
                "Central finite difference order not supported");

  double result = 0.0;

  // See the following link for coefficients:
  // https://en.wikipedia.org/wiki/Finite_difference_coefficient#Central_finite_difference
  if constexpr (Order == 2) {
    constexpr std::array kA{0.5};
    for (int j = 0; j < kA.size(); ++j) {
      result += kA[j] * f(i + (j + 1)) - kA[j] * f(i - (j + 1));
    }
  } else if constexpr (Order == 4) {
    constexpr std::array kA{2.0 / 3.0, -1.0 / 12.0};
    for (int j = 0; j < kA.size(); ++j) {
      result += kA[j] * f(i + (j + 1)) - kA[j] * f(i - (j + 1));
    }
  } else if constexpr (Order == 6) {
    constexpr std::array kA{3.0 / 4.0, -3.0 / 20.0, 1.0 / 60.0};
    for (int j = 0; j < kA.size(); ++j) {
      result += kA[j] * f(i + (j + 1)) - kA[j] * f(i - (j + 1));
    }
  } else if constexpr (Order == 8) {
    constexpr std::array kA{4.0 / 5.0, -1.0 / 5.0, 4.0 / 105.0, -1.0 / 280.0};
    for (int j = 0; j < kA.size(); ++j) {
      result += kA[j] * f(i + (j + 1)) - kA[j] * f(i - (j + 1));
    }
  }

  return result / h;
}

/**
 * Trims the quasistatic tests, applies a median filter to the velocity data,
 * calculates acceleration and cosine (arm only) data, and trims the dynamic
 * tests.
 *
 * @param data A pointer to a data vector recently created by the
 *             ConvertToPrepared method
 * @param settings A reference to the analysis settings
 * @param minStepTime A reference to the minimum dynamic test duration as one of
 *                    the trimming procedures will remove this amount from the
 *                    start of the test.
 * @param maxStepTime A reference to the maximum dynamic test duration
 * @param unit The angular unit that the arm test is in (only for calculating
 *             cosine data)
 */
void InitialTrimAndFilter(wpi::StringMap<std::vector<PreparedData>>* data,
                          AnalysisManager::Settings& settings,
                          units::second_t& minStepTime,
                          units::second_t& maxStepTime,
                          std::string_view unit = "");

/**
 * Removes all points with acceleration = 0.
 *
 * @param data A pointer to a PreparedData vector
 */
void AccelFilter(wpi::StringMap<std::vector<PreparedData>>* data);

}  // namespace sysid
