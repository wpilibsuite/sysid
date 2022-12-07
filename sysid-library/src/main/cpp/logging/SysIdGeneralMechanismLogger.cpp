// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/logging/SysIdGeneralMechanismLogger.h"

#include <array>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace sysid;

units::volt_t SysIdGeneralMechanismLogger::GetMotorVoltage() const {
  return m_primaryMotorVoltage;
}

void SysIdGeneralMechanismLogger::Log(double voltage, double measuredPosition,
                                      double measuredVelocity) {
  UpdateData();
  if (m_data.size() < kDataVectorSize) {
    std::array<double, 4> arr{m_timestamp, voltage, measuredPosition,
                              measuredVelocity};
    m_data.insert(m_data.end(), arr.cbegin(), arr.cend());
  }

  m_primaryMotorVoltage = units::volt_t{m_motorVoltage};
}

void SysIdGeneralMechanismLogger::Reset() {
  SysIdLogger::Reset();
  m_primaryMotorVoltage = 0_V;
}

bool SysIdGeneralMechanismLogger::IsWrongMechanism() const {
  return m_mechanism != "Arm" && m_mechanism != "Elevator" &&
         m_mechanism != "Simple";
}
