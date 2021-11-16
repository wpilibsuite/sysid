// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SimpleMotor.h"

#include <frc/RobotController.h>

void SimpleMotor::SimulationPeriodic() {
  m_flywheelSimulator.SetInputVoltage(units::volt_t{m_leader.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_flywheelSimulator.Update(5_ms);

  auto delta = m_flywheelSimulator.GetAngularVelocity().value() * 5_ms;
  distance += delta.value();

  m_encoderSim.SetDistance(distance);
  m_encoderSim.SetRate(m_flywheelSimulator.GetAngularVelocity().value());
}
