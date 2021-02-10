// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/AnalysisType.h"

#include <wpi/StringRef.h>

using namespace sysid;

AnalysisType sysid::analysis::FromName(wpi::StringRef name) {
  if (name == "Drivetrain") {
    return sysid::analysis::kDrivetrain;
  }
  if (name == "Elevator") {
    return sysid::analysis::kElevator;
  }
  if (name == "Arm") {
    return sysid::analysis::kArm;
  }
  return sysid::analysis::kSimple;
}
