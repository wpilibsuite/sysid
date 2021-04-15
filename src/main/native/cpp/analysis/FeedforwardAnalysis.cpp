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
 * Populates OLS vector for u = Ks + Kv v + Ka a.
 *
 * @param d        List of characterization data.
 * @param type     Type of system being identified.
 * @param olsData  Vector of OLS data.
 * @param dtMean   Mean period of data.
 */
static void PopulateAccelOLSVector(const std::vector<PreparedData>& d,
                                   const AnalysisType& type,
                                   std::vector<double>& olsData,
                                   units::second_t dtMean) {
  auto PushData = [&](int i) {
    // u = Ks sgn(v) + K_v v + Ka a

    // Add the dependent variable (voltage)
    olsData.push_back(d[i].voltage);

    // Add the intercept term (for Ks)
    olsData.push_back(std::copysign(1, d[i].velocity));

    // Add the velocity term (for Kv)
    olsData.push_back(d[i].velocity);

    // Add the acceleration term (for Ka)
    olsData.push_back(d[i].acceleration);

    if (type == analysis::kArm) {
      // Add the cosine term (for Kcos)
      olsData.push_back(d[i].cos);
    }
  };

  for (size_t i = 0; i < d.size() - 1; ++i) {
    // If dt is isn't in tolerance, ignore the data point. This gives a better
    // OLS fit.
    auto dt = d[i + 1].timestamp - d[i].timestamp;
    if (units::math::abs(dt - dtMean) < 1_ms) {
      PushData(i);
    }
  }
}

/**
 * Populates OLS vector for x_k+1 = alpha x_k + beta u_k + gamma sgn(x_k).
 *
 * @param d        List of characterization data.
 * @param type     Type of system being identified.
 * @param olsData  Vector of OLS data.
 * @param dtMean   Mean period of data.
 */
static void PopulateNextVelOLSVector(const std::vector<PreparedData>& d,
                                     const AnalysisType& type,
                                     std::vector<double>& olsData,
                                     units::second_t dtMean) {
  auto PushData = [&](int i) {
    // x_k+1 = alpha x_k + beta u_k + gamma sgn(x_k)

    // Add the dependent variable (next velocity)
    olsData.push_back(d[i + 1].velocity);

    // Add the velocity term (for alpha)
    olsData.push_back(d[i].velocity);

    // Add the voltage term (for beta)
    olsData.push_back(d[i].voltage);

    // Add the intercept term (for gamma)
    olsData.push_back(std::copysign(1, d[i].velocity));

    // Add test-specific variables
    if (type == analysis::kElevator) {
      // Add the gravity term (for Kg)
      olsData.push_back(1.0);
    }
  };

  for (size_t i = 0; i < d.size() - 1; ++i) {
    // If dt is isn't in tolerance, ignore the data point. This gives a better
    // OLS fit.
    auto dt = d[i + 1].timestamp - d[i].timestamp;
    if (units::math::abs(dt - dtMean) < 1_ms) {
      PushData(i);
    }
  }
}

std::tuple<std::vector<double>, double> sysid::CalculateFeedforwardGains(
    const Storage& data, const AnalysisType& type) {
  units::second_t dtMean = GetMeanTimeDelta(data);

  // Create a raw vector of doubles with our data in it.
  std::vector<double> olsData;
  olsData.reserve((1 + type.independentVariables) *
                  (data.slow.size() + data.fast.size()));

  // Iterate through the data and add it to our raw vector.
  const auto& [slow, fast] = data;
  if (type == analysis::kArm) {
    PopulateAccelOLSVector(slow, type, olsData, dtMean);
    PopulateAccelOLSVector(fast, type, olsData, dtMean);

    // Gains are Ks, Kv, Ka, Kcos
    return sysid::OLS(olsData, type.independentVariables);
  } else {
    // This implements the OLS algorithm defined in
    // https://file.tavsys.net/control/sysid-ols.pdf.

    PopulateNextVelOLSVector(slow, type, olsData, dtMean);
    PopulateNextVelOLSVector(fast, type, olsData, dtMean);

    auto ols = sysid::OLS(olsData, type.independentVariables);
    double alpha = std::get<0>(ols)[0];
    double beta = std::get<0>(ols)[1];
    double gamma = std::get<0>(ols)[2];

    // Initialize gains list with Ks, Kv, and Ka
    std::vector<double> gains{
        {-gamma / beta, (1 - alpha) / beta,
         dtMean.to<double>() * (alpha - 1) / (beta * std::log(alpha))}};

    if (type == analysis::kElevator) {
      // Add Kg to gains list
      double delta = std::get<0>(ols)[3];
      gains.emplace_back(-delta / beta);
    }

    // Gains are Ks, Kv, Ka, Kg (elevator only)
    return std::tuple{gains, std::get<1>(ols)};
  }
}
