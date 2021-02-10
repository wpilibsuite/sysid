// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstddef>

namespace wpi {
class StringRef;
}  // namespace wpi

namespace sysid {

enum class Mechanism { kDrivetrain, kElevator, kArm, kSimple };

struct AnalysisType {
  /** The mechanism for which analysis is being performed. */
  Mechanism mechanism;

  /** The number of independent variables for feedforward analysis */
  size_t independentVariables;

  /** Display name for the analysis type. */
  const char* name;

  /** Compares equality of two AnalysisType structs. */
  constexpr bool operator==(const AnalysisType& rhs) const {
    return mechanism == rhs.mechanism &&
           independentVariables == rhs.independentVariables;
  }

  /** Compares inequality of two AnalysisType structs. */
  constexpr bool operator!=(const AnalysisType& rhs) const {
    return !operator==(rhs);
  }
};

namespace analysis {
constexpr AnalysisType kDrivetrain{Mechanism::kDrivetrain, 3, "Drivetrain"};
constexpr AnalysisType kElevator{Mechanism::kElevator, 4, "Elevator"};
constexpr AnalysisType kArm{Mechanism::kArm, 4, "Arm"};
constexpr AnalysisType kSimple{Mechanism::kSimple, 3, "Simple"};

AnalysisType FromName(wpi::StringRef name);
}  // namespace analysis
}  // namespace sysid
