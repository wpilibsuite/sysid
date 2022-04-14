// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FeedforwardAnalysis.h"

#include <cmath>

#include <units/math.h>
#include <units/time.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/FilteringUtils.h"
#include "sysid/analysis/OLS.h"

using namespace sysid;

/**
 * Populates OLS vector for x_k+1 - x_k / tau = alpha x_k + beta u_k + gamma
 * sgn(x_k).
 *
 * @param d       List of characterization data.
 * @param type    Type of system being identified.
 * @param olsData Vector of OLS data.
 */
static void PopulateOLSVector(const std::vector<PreparedData>& d,
                              const AnalysisType& type,
                              std::vector<double>& olsData) {
  for (const auto& pt : d) {
    // Add the dependent variable (acceleration)
    olsData.push_back(pt.acceleration);

    // Add the velocity term (for alpha)
    olsData.push_back(pt.velocity);

    // Add the voltage term (for beta)
    olsData.push_back(pt.voltage);

    // Add the intercept term (for gamma)
    olsData.push_back(std::copysign(1, pt.velocity));

    // Add test-specific variables
    if (type == analysis::kElevator) {
      // Add the gravity term (for Kg)
      olsData.push_back(1.0);
    } else if (type == analysis::kArm) {
      // Add the cosine and sine terms (for Kcos)
      olsData.push_back(pt.cos);
      olsData.push_back(pt.sin);
    }
  }
}

std::tuple<std::vector<double>, double, double>
sysid::CalculateFeedforwardGains(const Storage& data,
                                 const AnalysisType& type) {
  // Create a raw vector of doubles with our data in it.
  std::vector<double> olsData;

  // Iterate through the data and add it to our raw vector.
  const auto& [slowForward, slowBackward, fastForward, fastBackward] = data;

  const auto size = slowForward.size() + slowBackward.size() +
                    fastForward.size() + fastBackward.size();

  // 1 dependent variable, n independent variables in each observation
  // Observations are stored serially
  olsData.reserve(1 + type.independentVariables * size);

  // Perform OLS with accel = alpha*vel + beta*voltage + gamma*signum(vel)
  // OLS performs best with the noisiest variable as the dependent var,
  // so we regress accel in terms of the other variables.
  PopulateOLSVector(slowForward, type, olsData);
  PopulateOLSVector(slowBackward, type, olsData);
  PopulateOLSVector(fastForward, type, olsData);
  PopulateOLSVector(fastBackward, type, olsData);

  auto ols = sysid::OLS(olsData, type.independentVariables);
  double alpha = std::get<0>(ols)[0];  // -kv/ka
  double beta = std::get<0>(ols)[1];   // 1/ka
  double gamma = std::get<0>(ols)[2];  // -ks/ka

  // Initialize gains list with Ks, Kv, and Ka
  std::vector<double> gains{-gamma / beta, -alpha / beta, 1 / beta};

  if (type == analysis::kElevator) {
    // Add Kg to gains list
    double delta = std::get<0>(ols)[3];  // -kg/ka
    gains.emplace_back(-delta / beta);
  }

  if (type == analysis::kArm) {
    double delta = std::get<0>(ols)[3];    // -kcos/ka cos(offset)
    double epsilon = std::get<0>(ols)[4];  // kcos/ka sin(offset)

    // Add Kcos to gains list
    gains.emplace_back(std::hypot(delta, epsilon) / beta);

    // Add offset to gains list
    gains.emplace_back(std::atan2(epsilon, -delta));
  }

  // Gains are Ks, Kv, Ka, Kg (elevator only), Kcos (arm only), offset (arm
  // only)
  return std::tuple{gains, std::get<1>(ols), std::get<2>(ols)};
}
