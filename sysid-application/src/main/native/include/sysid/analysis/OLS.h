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
 * @param XData The independent data in y = Xβ. The data must be formatted as
 *   x₀, x₁, x₂, ..., in the vector.
 * @param independentVariables The number of independent variables (x values).
 * @param yData The dependent data in y = Xβ.
 */
std::tuple<std::vector<double>, double, double> OLS(
    const std::vector<double>& XData, size_t independentVariables,
    const std::vector<double>& yData);
}  // namespace sysid
