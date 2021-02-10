// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <tuple>
#include <vector>

#include "sysid/analysis/AnalysisType.h"

namespace sysid {
// The prepared data from the analysis manager.
struct PreparedData;

// The type of storage used by the analysis manger.
using Storage =
    std::tuple<std::vector<PreparedData>, std::vector<PreparedData>>;

/**
 * Calculates feedforward gains given the data and the type of analysis to
 * perform.
 *
 * @return Tuple containing the coefficients of the analysis along with the
 *         r-squared (coefficient of determination) of the fit.
 */
std::tuple<std::vector<double>, double> CalculateFeedforwardGains(
    const Storage& data, const AnalysisType& type);
}  // namespace sysid
