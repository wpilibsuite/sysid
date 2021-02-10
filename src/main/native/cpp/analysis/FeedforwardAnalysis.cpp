// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FeedforwardAnalysis.h"

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/OLS.h"

using namespace sysid;

std::tuple<std::vector<double>, double> sysid::CalculateFeedforwardGains(
    const Storage& data, const AnalysisType& type) {
  // Create a raw vector of doubles with our data in it.
  std::vector<double> olsData;
  olsData.reserve((1 + type.independentVariables) *
                  (std::get<0>(data).size() + std::get<1>(data).size()));

  // Iterate through the data and add it to our raw vector.
  auto PopulateVector = [&](const std::vector<PreparedData>& d) {
    for (auto&& pt : d) {
      // Add the dependent variable (voltage).
      olsData.push_back(pt.voltage);

      // Add the intercept term (for Ks).
      olsData.push_back(std::copysign(1, pt.velocity));

      // Add the velocity term (for Kv).
      olsData.push_back(pt.velocity);

      // Add the acceleration term (for Ka).
      olsData.push_back(pt.acceleration);

      // Add test-specific variables.
      if (type.mechanism == Mechanism::kElevator) {
        // Add the gravity term (for Kg)
        olsData.push_back(1.0);
      } else if (type.mechanism == Mechanism::kArm) {
        // Add the cosine term (for Kcos)
        olsData.push_back(pt.cos);
      }
    }
  };
  PopulateVector(std::get<0>(data));
  PopulateVector(std::get<1>(data));

  return sysid::OLS(olsData, type.independentVariables);
}
