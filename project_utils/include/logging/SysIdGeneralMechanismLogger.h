// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "logging/SysIdLogger.h"

// TODO Set to proper number after telemetry data refactor
class SysIdGeneralMechanismLogger : public SysIdLogger {
 public:
  /**
   * The users should set their motors to what this returns AFTER calling log.
   */
  units::volt_t GetMotorVoltage();

  /**
   * Logs data for a single-sided mechanism (Elevator, Simple, Arm).
   *
   * Outputs data in form: timestamp, voltage, position, velocity.
   *
   * @param measuredPosition the recorded rotations of the shaft
   * @param measureVelocity the recorded rotations per second of the shaft
   */
  void Log(double measuredPosition, double measuredVelocity);
};
