// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "logging/SysIdDrivetrainLogger.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

units::volt_t SysIdDrivetrainLogger::GetLeftMotorVoltage() {
  return m_primaryMotorVoltage;
}

units::volt_t SysIdDrivetrainLogger::GetRightMotorVoltage() {
  return m_secondaryMotorVoltage;
}

void SysIdDrivetrainLogger::Log(double leftPosition, double rightPosition,
                                double leftVelocity, double rightVelocity,
                                double measuredAngle, double angularRate) {
  UpdateData();
  std::array<double, 9> arr = {m_timestamp,
                               m_primaryMotorVoltage.to<double>(),
                               m_secondaryMotorVoltage.to<double>(),
                               leftPosition,
                               rightPosition,
                               leftVelocity,
                               rightVelocity,
                               measuredAngle,
                               angularRate};
  m_data.insert(m_data.end(), arr.cbegin(), arr.cend());

  m_primaryMotorVoltage = units::volt_t{(m_rotate ? -1 : 1) * m_motorVoltage};
  m_secondaryMotorVoltage = units::volt_t{m_motorVoltage};
}
