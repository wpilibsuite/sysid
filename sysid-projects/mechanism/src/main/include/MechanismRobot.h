// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/CANSparkMax.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>

#include "sysid/generation/SysIdSetup.h"
#include "sysid/logging/SysIdGeneralMechanismLogger.h"

/* Keep CTRE includes below other headers to allow Windows to build */
#include <ctre/Phoenix.h>
#include <ctre/phoenixpro/CANcoder.hpp>

class MechanismRobot : public frc::TimedRobot {
 public:
  MechanismRobot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void SimulationPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

  void PushNTDiagnostics();

 private:
  std::vector<std::unique_ptr<frc::MotorController>> m_controllers;
  std::vector<std::string> m_controllerNames;
  std::function<double()> m_position;
  std::function<double()> m_rate;
  wpi::json m_json;
  std::unique_ptr<rev::SparkMaxRelativeEncoder> m_revEncoderPort;
  std::unique_ptr<rev::SparkMaxAlternateEncoder> m_revDataPort;
  std::unique_ptr<CANCoder> m_cancoder;
  std::unique_ptr<ctre::phoenixpro::hardware::CANcoder> m_cancoderPro;
  std::unique_ptr<frc::Encoder> m_encoder;
  sysid::SysIdGeneralMechanismLogger m_logger;
};
