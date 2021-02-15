// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Arm.h"

#include <frc/RobotController.h>

void Arm::SimulationPeriodic() {
  m_armSimulator.SetInput(frc::MakeMatrix<1, 1>(
      m_leader.Get() * frc::RobotController::GetInputVoltage()));
  m_armSimulator.Update(5_ms);

  m_encoderSim.SetDistance(m_armSimulator.GetAngle().to<double>());
  m_encoderSim.SetRate(m_armSimulator.GetVelocity().to<double>());
}