// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "logging/SysIdGeneralMechanismLogger.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

units::volt_t SysIdGeneralMechanismLogger::GetMotorVoltage() {
  return m_primaryMotorVoltage;
}

void SysIdGeneralMechanismLogger::Log(double measuredPosition,
                                      double measuredVelocity) {
  UpdateData();
  std::array<double, 4> arr = {m_timestamp, m_primaryMotorVoltage.to<double>(),
                               measuredPosition, measuredVelocity};
  m_data.insert(m_data.end(), arr.cbegin(), arr.cend());
  m_primaryMotorVoltage = units::volt_t{m_motorVoltage};
}
