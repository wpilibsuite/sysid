// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Elevator.h"

#include <frc/RobotController.h>

void Elevator::SimulationPeriodic() {
  m_elevatorSimulator.SetInputVoltage(units::volt_t{m_leader.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_elevatorSimulator.Update(5_ms);

  m_encoderSim.SetDistance(m_elevatorSimulator.GetPosition().value());
  m_encoderSim.SetRate(m_elevatorSimulator.GetVelocity().value());
}
