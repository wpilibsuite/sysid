// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <tuple>
#include <vector>

namespace sysid {
/**
 * Performs ordinary least squares multiple regression on the provided data and
 * returns a vector of coefficients along with the r-squared (coefficient of
 * determination) and RMSE (stardard deviation of the residuals) of the fit.
 *
 * @param data                 The data to perform the regression on. The data
 *                             must be formatted as y, x₀, x₁, x₂, ..., y, ...
 *                             in the vector.
 * @param independentVariables The number of independent variables (x values).
 */
std::tuple<std::vector<double>, double, double> OLS(
    const std::vector<double>& data, size_t independentVariables);
}  // namespace sysid
