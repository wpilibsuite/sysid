// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <string>
#include <string_view>

#include <fmt/format.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/StringExtras.h>

#include "sysid/generation/SysIdSetup.h"

Robot::Robot() : frc::TimedRobot(5_ms) {
  m_json = sysid::GetConfigJson();

  try {
    fmt::print("Read JSON\n");

    std::vector<int> leftPorts =
        m_json.at("primary motor ports").get<std::vector<int>>();
    m_controllerNames =
        m_json.at("motor controllers").get<std::vector<std::string>>();
    std::vector<int> leftEncoderPorts =
        m_json.at("primary encoder ports").get<std::vector<int>>();
    std::vector<bool> leftMotorsInverted =
        m_json.at("primary motors inverted").get<std::vector<bool>>();

    std::vector<int> rightPorts =
        m_json.at("secondary motor ports").get<std::vector<int>>();
    std::vector<int> rightEncoderPorts =
        m_json.at("secondary encoder ports").get<std::vector<int>>();
    std::vector<bool> rightMotorsInverted =
        m_json.at("secondary motors inverted").get<std::vector<bool>>();

    std::string encoderType = m_json.at("encoder type").get<std::string>();
    bool leftEncoderInverted =
        m_json.at("primary encoder inverted").get<bool>();
    bool rightEncoderInverted =
        m_json.at("secondary encoder inverted").get<bool>();
    double cpr = m_json.at("counts per rotation").get<double>();
    double gearingNumerator = m_json.at("gearing numerator").get<double>();
    double gearingDenominator = m_json.at("gearing denominator").get<double>();
    double gearing = gearingNumerator / gearingDenominator;

    std::string gyroType = m_json.at("gyro").get<std::string>();
    std::string gyroCtor = m_json.at("gyro ctor").get<std::string>();

    bool isEncoding = m_json.at("encoding").get<bool>();

    int numSamples = m_json.at("number of samples per average").get<int>();

    int period = m_json.at("velocity measurement period").get<int>();

    fmt::print("Setup motors\n");
    for (size_t i = 0; i < leftPorts.size(); i++) {
      sysid::AddMotorController(leftPorts[i], m_controllerNames[i],
                                leftMotorsInverted[i], &m_leftControllers);
      sysid::AddMotorController(rightPorts[i], m_controllerNames[i],
                                rightMotorsInverted[i], &m_rightControllers);
    }

    fmt::print("Setup encoders\n");
    sysid::SetupEncoders(encoderType, isEncoding, period, cpr, gearing,
                         numSamples, m_controllerNames[0],
                         m_leftControllers.at(0).get(), leftEncoderInverted,
                         leftEncoderPorts, m_leftCancoder, m_leftRevEncoderPort,
                         m_leftRevDataPort, m_leftEncoder, m_leftPosition,
                         m_leftRate);
    sysid::SetupEncoders(encoderType, isEncoding, period, cpr, gearing,
                         numSamples, m_controllerNames[0],
                         m_rightControllers.at(0).get(), rightEncoderInverted,
                         rightEncoderPorts, m_rightCancoder,
                         m_rightRevEncoderPort, m_rightRevDataPort,
                         m_rightEncoder, m_rightPosition, m_rightRate);

    fmt::print("Setup gyro\n");
    sysid::SetupGyro(gyroType, gyroCtor, leftPorts, rightPorts,
                     m_controllerNames, m_leftControllers, m_rightControllers,
                     m_gyro, m_ADIS16448Gyro, m_ADIS16470Gyro, m_pigeon,
                     m_tempTalon, m_gyroPosition, m_gyroRate);
  } catch (std::exception& e) {
    fmt::print("Project failed: {}\n", e.what());
    std::exit(-1);
  }
  m_logger.UpdateThreadPriority();
  std::fflush(stdout);

#ifdef INTEGRATION
  frc::SmartDashboard::PutBoolean("SysIdRun", false);
  // TODO use std::exit or EndCompetition once CTRE bug is fixed
  std::set_terminate([]() { std::_Exit(0); });
#endif
}

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable
 * chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
 * Dashboard, remove all of the chooser code and uncomment the GetString line
 * to get the auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the
 * SendableChooser make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_logger.InitLogging();
}

/**
 * Outputs data in the format: timestamp, l voltage, r voltage, l position, r
 * position, l velocity, r velocity, angle, angular rate
 */
void Robot::AutonomousPeriodic() {
  m_logger.Log(m_logger.MeasureVoltage(m_leftControllers, m_controllerNames),
               m_logger.MeasureVoltage(m_rightControllers, m_controllerNames),
               m_leftPosition(), m_rightPosition(), m_leftRate(), m_rightRate(),
               m_gyroPosition(), m_gyroRate());
  sysid::SetMotorControllers(m_logger.GetLeftMotorVoltage(), m_leftControllers);
  sysid::SetMotorControllers(m_logger.GetRightMotorVoltage(),
                             m_rightControllers);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  PushNTDiagnostics();
}

void Robot::DisabledInit() {
  sysid::SetMotorControllers(0_V, m_leftControllers);
  sysid::SetMotorControllers(0_V, m_rightControllers);
  fmt::print("Robot disabled\n");
  m_logger.SendData();
}

void Robot::SimulationPeriodic() {
#ifdef INTEGRATION

  bool enable = frc::SmartDashboard::GetBoolean("SysIdRun", false);
  frc::sim::DriverStationSim::SetAutonomous(enable);
  frc::sim::DriverStationSim::SetEnabled(enable);
  frc::sim::DriverStationSim::NotifyNewData();

  if (frc::SmartDashboard::GetBoolean("SysIdKill", false)) {
    // TODO use std::exit or EndCompetition once CTRE bug is fixed
    std::terminate();
  }
#endif
}

void Robot::DisabledPeriodic() {
  PushNTDiagnostics();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
  PushNTDiagnostics();
}

void Robot::PushNTDiagnostics() {
  try {
    frc::SmartDashboard::PutNumber(
        "Left Voltage",
        m_logger.MeasureVoltage(m_leftControllers, m_controllerNames));
    frc::SmartDashboard::PutNumber(
        "Right Voltage",
        m_logger.MeasureVoltage(m_rightControllers, m_controllerNames));

    frc::SmartDashboard::PutNumber("Left Position", m_leftPosition());
    frc::SmartDashboard::PutNumber("Right Position", m_rightPosition());
    frc::SmartDashboard::PutNumber("Left Velocity", m_leftRate());
    frc::SmartDashboard::PutNumber("Right Velocity", m_rightRate());

    frc::SmartDashboard::PutNumber("Gyro Reading", m_gyroPosition());
    frc::SmartDashboard::PutNumber("Gyro Rate", m_gyroRate());
  } catch (std::exception& e) {
    fmt::print("Project failed: {}\n", e.what());
    std::exit(-1);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
