// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #ifdef __FRC_ROBORIO__
// #include <ctre/Phoenix.h>
// #endif

#include <functional>
#include <memory>
#include <vector>

#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <frc/interfaces/Gyro.h>
#include <frc/motorcontrol/MotorController.h>
#include <rev/CANSparkMax.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>

#include "sysid/logging/SysIdDrivetrainLogger.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
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

 private:
  std::vector<std::unique_ptr<frc::MotorController>> m_rightControllers;
  std::vector<std::unique_ptr<frc::MotorController>> m_leftControllers;
  std::function<double()> m_leftPosition;
  std::function<double()> m_leftRate;
  std::function<double()> m_rightPosition;
  std::function<double()> m_rightRate;
  std::function<double()> m_gyroPosition;
  std::function<double()> m_gyroRate;
  wpi::json m_json;
  std::unique_ptr<rev::SparkMaxRelativeEncoder> m_leftRevEncoderPort;
  std::unique_ptr<rev::SparkMaxAlternateEncoder> m_leftRevDataPort;
  std::unique_ptr<rev::SparkMaxRelativeEncoder> m_rightRevEncoderPort;
  std::unique_ptr<rev::SparkMaxAlternateEncoder> m_rightRevDataPort;
  std::unique_ptr<frc::Encoder> m_leftEncoder;
  std::unique_ptr<frc::Encoder> m_rightEncoder;
  std::unique_ptr<frc::Gyro> m_gyro;

  // #ifdef __FRC_ROBORIO__
  //   std::unique_ptr<CANCoder> m_leftCancoder;
  //   std::unique_ptr<CANCoder> m_rightCancoder;
  //   std::unique_ptr<PigeonIMU> m_pigeon;
  // #endif

  sysid::SysIdDrivetrainLogger m_logger;
};
