// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <hal/HAL.h>
#include <rev/CANSparkMax.h>
#include <signal.h>

#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <frc/Spark.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/FileSystem.h>
#include <wpi/SmallString.h>
#include <wpi/StringMap.h>
#include <wpi/raw_ostream.h>

#include "generation/SysIdSetup.h"
#include "rev/CANEncoder.h"

Robot::Robot() : frc::TimedRobot(5_ms) {
  m_json = GetConfigJson();

  try {
    wpi::outs() << "reading json \n";
    wpi::outs().flush();
    std::vector<int> ports =
        m_json.at("primary motor ports").get<std::vector<int>>();
    std::vector<std::string> controllerNames =
        m_json.at("motor controllers").get<std::vector<std::string>>();
    std::vector<int> encoderPorts =
        m_json.at("primary encoder ports").get<std::vector<int>>();
    std::vector<bool> motorsInverted =
        m_json.at("primary motors inverted").get<std::vector<bool>>();

    std::string encoderType = m_json.at("encoder type").get<std::string>();
    bool encoderInverted = m_json.at("primary encoder inverted").get<bool>();

    double cpr = m_json.at("counts per shaft revolution").get<double>();

    bool isEncoding = m_json.at("encoding").get<bool>();

    int numSamples = m_json.at("number of samples per average").get<int>();

    int period = m_json.at("velocity measurement period").get<int>();

    wpi::outs() << "Initializing motors \n";
    wpi::outs().flush();
    for (size_t i = 0; i < ports.size(); i++) {
      AddMotorController(ports[i], controllerNames[i], motorsInverted[i],
                         &m_controllers);
    }

    wpi::outs() << "Initializing encoder\n";
    wpi::outs().flush();
    SetupEncoders(encoderType, isEncoding, period, cpr, numSamples,
                  controllerNames[0], m_controllers.front().get(),
                  encoderInverted, encoderPorts, m_cancoder, m_encoder,
                  m_position, m_rate);
  } catch (std::exception& e) {
    wpi::outs() << "Project failed: " << e.what() << "\n";
    wpi::outs().flush();
    std::exit(-1);
  }
#ifdef INTEGRATION
  // TODO use std::exit or EndCompetition once CTRE bug is fixed
  std::set_terminate([]() { std::_Exit(0); });
#endif
}

void Robot::RobotInit() {}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  try {
    frc::SmartDashboard::PutNumber("Position", m_position());
    frc::SmartDashboard::PutNumber("Rate", m_rate());
  } catch (std::exception& e) {
    wpi::outs() << "Project failed: " << e.what() << "\n";
    wpi::outs().flush();
    std::exit(-1);
  }
  // TODO Put actual readings once supported
}

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
  m_logger.Log(m_position(), m_rate());
  SetMotorControllers(m_logger.GetMotorVoltage(), m_controllers);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  SetMotorControllers(units::volt_t{0}, m_controllers);
  wpi::outs() << "Robot disabled\n";
  m_logger.SendData();
}

void Robot::SimulationPeriodic() {
#ifdef INTEGRATION
  if (frc::SmartDashboard::GetBoolean("SysIdKill", false)) {
    // TODO use std::exit or EndCompetition once CTRE bug is fixed
    std::terminate();
  }
#endif
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
