// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/OLS.h"

#include <tuple>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

using namespace sysid;

std::tuple<std::vector<double>, double> sysid::OLS(
    const std::vector<double>& data, size_t independentVariables) {
  // Perform some quick sanity checks regarding the size of the vector.
  assert(data.size() % (independentVariables + 1) == 0);

  // The linear model can be written as follows:
  // y = Xβ + u, where y is the dependent observed variable, X is the matrix
  // of independent variables, β is a vector of coefficients, and u is a
  // vector of residuals.

  // We want to minimize u^2 = u'u = (y - Xβ)'(y - Xβ).
  // β = (X'X)^-1 (X'y)

  // Get the number of elements.
  size_t n = data.size() / (independentVariables + 1);

  // Create new variables to make things more readable.
  size_t rows = n;
  size_t cols = independentVariables;  // X
  size_t strd = independentVariables + 1;

  // Create y and X matrices.
  Eigen::Map<const Eigen::MatrixXd, 0, Eigen::Stride<1, Eigen::Dynamic>> y(
      data.data() + 0, rows, 1, Eigen::Stride<1, Eigen::Dynamic>(1, strd));

  Eigen::Map<const Eigen::MatrixXd, 0, Eigen::Stride<1, Eigen::Dynamic>> X(
      data.data() + 1, rows, cols, Eigen::Stride<1, Eigen::Dynamic>(1, strd));

  // Calculate b = β that minimizes u'u.
  Eigen::MatrixXd b = (X.transpose() * X).llt().solve(X.transpose() * y);

  // We will now calculate r^2 or the coefficient of determination, which
  // tells us how much of the total variation (variation in y) can be
  // explained by the regression model.

  // We will first calculate the sum of the squares of the error, or the
  // variation in error (SSE).
  double SSE = (y - X * b).squaredNorm();

  // Now we will calculate the total variation in y, known as SSTO.
  double SSTO = ((y.transpose() * y) - (1 / n) * (y.transpose() * y)).value();

  double rSquared = (SSTO - SSE) / SSTO;
  double adjRSquared = 1 - (1 - rSquared) * ((n - 1.0) / (n - 3));

  return {{b.data(), b.data() + b.rows()}, adjRSquared};
}
