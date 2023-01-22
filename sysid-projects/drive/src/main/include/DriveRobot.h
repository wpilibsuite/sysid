// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <frc/ADIS16448_IMU.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <frc/interfaces/Gyro.h>
#include <frc/motorcontrol/MotorController.h>
#include <rev/CANSparkMax.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>

/* Keep CTRE includes below ADIS16448_IMU header to allow Windows to build */
#include <ctre/Phoenix.h>
#include <ctre/phoenixpro/Pigeon2.hpp>
#include <ctre/phoenixpro/CANcoder.hpp>

#include "sysid/logging/SysIdDrivetrainLogger.h"

class DriveRobot : public frc::TimedRobot {
 public:
  DriveRobot();
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
  std::vector<std::unique_ptr<frc::MotorController>> m_rightControllers;
  std::vector<std::unique_ptr<frc::MotorController>> m_leftControllers;
  std::vector<std::string> m_controllerNames;
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
  std::unique_ptr<frc::ADIS16448_IMU> m_ADIS16448Gyro;
  std::unique_ptr<frc::ADIS16470_IMU> m_ADIS16470Gyro;

  std::unique_ptr<CANCoder> m_leftCancoder;
  std::unique_ptr<CANCoder> m_rightCancoder;
  std::unique_ptr<ctre::phoenixpro::hardware::CANcoder> m_leftCancoderPro;
  std::unique_ptr<ctre::phoenixpro::hardware::CANcoder> m_rightCancoderPro;
  std::unique_ptr<WPI_TalonSRX> m_tempTalon;
  std::unique_ptr<BasePigeon> m_pigeon;
  std::unique_ptr<ctre::phoenixpro::hardware::Pigeon2> m_pigeonPro;

  sysid::SysIdDrivetrainLogger m_logger;
};
