// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <cstddef>
#include <cstdio>
#include <exception>
#include <string>

#include <fmt/format.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

#include "sysid/generation/SysIdSetup.h"

Robot::Robot() : frc::TimedRobot(5_ms) {
  try {
    m_json = sysid::GetConfigJson();
    fmt::print("Reading JSON\n");
    std::vector<int> ports =
        m_json.at("primary motor ports").get<std::vector<int>>();
    m_controllerNames =
        m_json.at("motor controllers").get<std::vector<std::string>>();
    std::vector<int> encoderPorts =
        m_json.at("primary encoder ports").get<std::vector<int>>();
    std::vector<bool> motorsInverted =
        m_json.at("primary motors inverted").get<std::vector<bool>>();

    std::string encoderType = m_json.at("encoder type").get<std::string>();
    bool encoderInverted = m_json.at("primary encoder inverted").get<bool>();

    double cpr = m_json.at("counts per rotation").get<double>();
    double gearingNumerator = m_json.at("gearing numerator").get<double>();
    double gearingDenominator = m_json.at("gearing denominator").get<double>();
    double gearing = gearingNumerator / gearingDenominator;

    bool isEncoding = m_json.at("encoding").get<bool>();

    int numSamples = m_json.at("number of samples per average").get<int>();

    int period = m_json.at("velocity measurement period").get<int>();

    fmt::print("Initializing Motors\n");
    for (size_t i = 0; i < ports.size(); i++) {
      sysid::AddMotorController(ports[i], m_controllerNames[i],
                                motorsInverted[i], &m_controllers);
    }

    fmt::print("Initializing encoder\n");
    sysid::SetupEncoders(encoderType, isEncoding, period, cpr, gearing,
                         numSamples, m_controllerNames[0],
                         m_controllers.front().get(), encoderInverted,
                         encoderPorts, m_cancoder, m_revEncoderPort,
                         m_revDataPort, m_encoder, m_position, m_rate);
  } catch (std::exception& e) {
    fmt::print("Project failed: {}\n", e.what());
    std::exit(-1);
  }
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
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_logger.InitLogging();
}

/**
 * Outputs data in the format: timestamp, voltage, position, velocity
 */
void Robot::AutonomousPeriodic() {
  m_logger.Log(m_logger.MeasureVoltage(m_controllers, m_controllerNames),
               m_position(), m_rate());
  sysid::SetMotorControllers(m_logger.GetMotorVoltage(), m_controllers);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  PushNTDiagnostics();
}

void Robot::DisabledInit() {
  sysid::SetMotorControllers(0_V, m_controllers);
  fmt::print("Robot Disabled\n");
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
        "Voltage", m_logger.MeasureVoltage(m_controllers, m_controllerNames));
    frc::SmartDashboard::PutNumber("Position", m_position());
    frc::SmartDashboard::PutNumber("Rate", m_rate());
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
