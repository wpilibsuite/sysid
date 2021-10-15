// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>

#include "logging/SysIdLogger.h"

// TODO Set to proper number after telemetry data refactor
class SysIdDrivetrainLogger : public SysIdLogger {
 public:
  /**
   * The users should their left motors to what this returns AFTER calling log.
   */
  units::volt_t GetLeftMotorVoltage() const;

  /**
   * The users should set their right motors to what this returns AFTER calling
   * log.
   */
  units::volt_t GetRightMotorVoltage() const;

  /**
   * Logs data for a drivetrain mechanism.
   *
   * When SendData() is called it outputs data in the form: timestamp, l
   * voltage, r voltage, l position, r position, l velocity, r velocity, angle,
   * angular rate
   *
   * @param leftPosition the recorded rotations of the left shaft
   * @param rightPosition the recorded rotations of the right shaft
   * @param leftVelocity the recorded rotations per second of the left shaft
   * @param rightVelocity the recorded rotations per second or the right shaft
   * @param measuredAngle the recorded angle of they gyro
   * @param angularRate the recorded angular rate of the gyro in radians per
   * second
   */
  void Log(double leftPosition, double rightPosition, double leftVelocity,
           double rightVelocity, double measuredAngle, double angularRate);

  void Reset() override;

  bool IsWrongMechanism() const override;

 private:
  units::volt_t m_primaryMotorVoltage = 0_V;
  units::volt_t m_secondaryMotorVoltage = 0_V;
};
