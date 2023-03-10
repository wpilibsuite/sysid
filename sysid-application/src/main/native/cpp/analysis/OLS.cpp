// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/OLS.h"

#include <tuple>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

using namespace sysid;

std::tuple<std::vector<double>, double, double> sysid::OLS(
    const std::vector<double>& XData, size_t independentVariables,
    const std::vector<double>& yData) {
  // Perform some quick sanity checks regarding the size of the vector.
  assert(XData.size() % independentVariables == 0);
  assert(yData.size() % independentVariables == 0);

  // The linear model can be written as follows:
  // y = Xβ + u, where y is the dependent observed variable, X is the matrix
  // of independent variables, β is a vector of coefficients, and u is a
  // vector of residuals.

  // We want to minimize u² = uᵀu = (y - Xβ)ᵀ(y - Xβ).
  // β = (XᵀX)⁻¹Xᵀy

  // Create X matrix and y vector.
  Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
      X(XData.data(), XData.size() / independentVariables,
        independentVariables);
  Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
      y(yData.data(), yData.size(), 1);

  // Calculate β that minimizes uᵀu.
  Eigen::MatrixXd beta = (X.transpose() * X).llt().solve(X.transpose() * y);

  // We will now calculate R² or the coefficient of determination, which
  // tells us how much of the total variation (variation in y) can be
  // explained by the regression model.

  // We will first calculate the sum of the squares of the error, or the
  // variation in error (SSE).
  double SSE = (y - X * beta).squaredNorm();

  int n = X.cols();

  // Now we will calculate the total variation in y, known as SSTO.
  double SSTO = ((y.transpose() * y) - (1.0 / n) * (y.transpose() * y)).value();

  double rSquared = (SSTO - SSE) / SSTO;
  double adjRSquared = 1.0 - (1.0 - rSquared) * ((n - 1.0) / (n - 3.0));
  double RMSE = std::sqrt(SSE / n);

  return {{beta.data(), beta.data() + beta.rows()}, adjRSquared, RMSE};
}
