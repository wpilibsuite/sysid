// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/logging/SysIdDrivetrainLogger.h"

#include <array>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace sysid;

units::volt_t SysIdDrivetrainLogger::GetLeftMotorVoltage() const {
  return m_primaryMotorVoltage;
}

units::volt_t SysIdDrivetrainLogger::GetRightMotorVoltage() const {
  return m_secondaryMotorVoltage;
}

void SysIdDrivetrainLogger::Log(double leftVoltage, double rightVoltage,
                                double leftPosition, double rightPosition,
                                double leftVelocity, double rightVelocity,
                                double measuredAngle, double angularRate) {
  UpdateData();
  if (m_data.size() < kDataVectorSize) {
    std::array<double, 9> arr{m_timestamp,   leftVoltage,   rightVoltage,
                              leftPosition,  rightPosition, leftVelocity,
                              rightVelocity, measuredAngle, angularRate};
    m_data.insert(m_data.end(), arr.cbegin(), arr.cend());
  }

  m_primaryMotorVoltage = units::volt_t{(m_rotate ? -1 : 1) * m_motorVoltage};
  m_secondaryMotorVoltage = units::volt_t{m_motorVoltage};
}

void SysIdDrivetrainLogger::Reset() {
  SysIdLogger::Reset();
  m_primaryMotorVoltage = 0_V;
  m_secondaryMotorVoltage = 0_V;
}

bool SysIdDrivetrainLogger::IsWrongMechanism() const {
  return m_mechanism != "Drivetrain" && m_mechanism != "Drivetrain (Angular)";
}
