// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>

#include "sysid/logging/SysIdLogger.h"

namespace sysid {

/**
 * Serves to provide methods for robot projects seeking to send and receive data
 * in occurdence to the SysId general mechanism protocols.
 */
class SysIdGeneralMechanismLogger : public SysIdLogger {
 public:
  /**
   * The users should set their motors to what this returns AFTER calling log.
   *
   * @returns The voltage that the mechanism motor(s) should be set to.
   */
  units::volt_t GetMotorVoltage() const;

  /**
   * Logs data for a single-sided mechanism (Elevator, Simple, Arm).
   *
   * When SendData() is called it outputs data in the form: timestamp, voltage,
   * position, velocity.
   *
   * @param voltage the recorded voltage of the motors
   * @param measuredPosition the recorded rotations of the shaft
   * @param measuredVelocity the recorded rotations per second of the shaft
   */
  void Log(double voltage, double measuredPosition, double measuredVelocity);

  void Reset() override;

  bool IsWrongMechanism() const override;

 private:
  units::volt_t m_primaryMotorVoltage = 0_V;
};

}  // namespace sysid
