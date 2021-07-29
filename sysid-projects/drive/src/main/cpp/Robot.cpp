// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <string>
#include <string_view>

#include <fmt/format.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
// #include <frc/romi/RomiGyro.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

// #include "AHRS.h"
#include "generation/SysIdSetup.h"

Robot::Robot() : frc::TimedRobot(5_ms) {
  m_json = GetConfigJson();

  try {
    fmt::print("Read JSON\n");

    std::vector<int> leftPorts =
        m_json.at("primary motor ports").get<std::vector<int>>();
    std::vector<std::string> controllerNames =
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
    double gearing = m_json.at("gearing").get<double>();

    std::string gyroType = m_json.at("gyro").get<std::string>();
    std::string gyroCtor = m_json.at("gyro ctor").get<std::string>();

    bool isEncoding = m_json.at("encoding").get<bool>();

    int numSamples = m_json.at("number of samples per average").get<int>();

    int period = m_json.at("velocity measurement period").get<int>();

    fmt::print("Setup motors\n");
    for (size_t i = 0; i < leftPorts.size(); i++) {
      AddMotorController(leftPorts[i], controllerNames[i],
                         leftMotorsInverted[i], &m_leftControllers);
      AddMotorController(rightPorts[i], controllerNames[i],
                         rightMotorsInverted[i], &m_rightControllers);
    }

    fmt::print("Setup encoders\n");
    SetupEncoders(encoderType, isEncoding, period, cpr * gearing, numSamples,
                  controllerNames[0], m_leftControllers.at(0).get(),
                  leftEncoderInverted, leftEncoderPorts, m_leftCancoder,
                  m_leftEncoder, m_leftPosition, m_leftRate);
    SetupEncoders(encoderType, isEncoding, period, cpr * gearing, numSamples,
                  controllerNames[0], m_rightControllers.at(0).get(),
                  rightEncoderInverted, rightEncoderPorts, m_rightCancoder,
                  m_rightEncoder, m_rightPosition, m_rightRate);

    fmt::print("Setup gyro\n");
    if (gyroType == "Pigeon") {
      std::string portStr = "";
      if (gyroCtor.find("WPI_TalonSRX") != std::string_view::npos) {
        portStr = gyroCtor[gyroCtor.find("-") + 1];
      } else {
        portStr = gyroCtor;
      }

      fmt::print("Port String: {}\n", portStr);
      // converts gyroCtor into port #
      int srxPort = std::stoi(portStr);
      if (gyroCtor.find("WPI_TalonSRX") != std::string_view::npos) {
        bool talonDeclared = false;
        // Check if there is a Talon Port in Left Ports
        auto findPort = std::find(leftPorts.begin(), leftPorts.end(), srxPort);

        // Check Right Ports if not found
        if (findPort == leftPorts.end()) {
          findPort = std::find(rightPorts.begin(), rightPorts.end(), srxPort);
          if (findPort != rightPorts.end() &&
              controllerNames[findPort - rightPorts.begin()] == "TalonSRX") {
            m_pigeon = std::make_unique<PigeonIMU>(dynamic_cast<WPI_TalonSRX*>(
                m_rightControllers.at(findPort - rightPorts.begin()).get()));
            talonDeclared = true;
          }
        } else if (controllerNames[findPort - leftPorts.begin()] ==
                   "TalonSRX") {
          m_pigeon = std::make_unique<PigeonIMU>(dynamic_cast<WPI_TalonSRX*>(
              m_leftControllers.at(findPort - leftPorts.begin()).get()));
          talonDeclared = true;
        }

        // If it isn't tied to an existing Talon, create a new object
        if (!talonDeclared) {
          m_pigeon = std::make_unique<PigeonIMU>(new WPI_TalonSRX(srxPort));
        }
      } else {
        m_pigeon = std::make_unique<PigeonIMU>(srxPort);
      }

      // setup functions
      m_gyroPosition = [&, this] {
        double xyz[3];
        m_pigeon->GetAccumGyro(xyz);
        return xyz[2];
      };

      m_gyroRate = [&, this] {
        double xyz_dps[3];
        m_pigeon->GetRawGyro(xyz_dps);
        units::degrees_per_second_t rate{xyz_dps[2]};
        return units::radians_per_second_t{rate}.to<double>();
      };

      // } else if (gyroType != "None") {
      //   if (gyroType == "ADXRS450") {
      //     if (gyroCtor == "SPI.kMXP") {
      //       m_gyro =
      //       std::make_unique<frc::ADXRS450_Gyro>(frc::SPI::Port::kMXP);
      //     } else {
      //       m_gyro =
      //           std::make_unique<frc::ADXRS450_Gyro>(frc::SPI::Port::kOnboardCS0);
      //     }
      //     // FIXME: waiting on Linux and macOS builds for navX AHRS
      //     // } else if (gyroType == "NavX") {
      //     //   if (gyroCtor == "SerialPort.kUSB") {
      //     //     m_gyro =
      //     std::make_unique<AHRS>(frc::SerialPort::Port::kUSB);
      //     //   } else if (gyroCtor == "I2C") {
      //     //     m_gyro = std::make_unique<AHRS>(frc::I2C::Port::kMXP);
      //     //   } else if (gyroCtor == "SerialPort.kMXP") {
      //     //     m_gyro =
      //     std::make_unique<AHRS>(frc::SerialPort::Port::kMXP);
      //     //   } else {
      //     //     m_gyro = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
      //     //   }
      //     // FIXME: Update Romi Gyro once vendordep is out
      //     // } else if (gyroType == "Romi") {
      //     //   m_gyro = std::make_unique<frc::RomiGyro>();
      //   } else {
      //     try {
      //       m_gyro = std::make_unique<frc::AnalogGyro>(std::stoi(gyroCtor));
      //     } catch (std::invalid_argument& e) {
      //       m_gyro = std::make_unique<frc::AnalogGyro>(0);
      //     }
      //   }

      //   m_gyroPosition = [&, this] {
      //     return units::radian_t(units::degree_t{m_gyro->GetAngle()})
      //         .to<double>();
      //   };

      //   m_gyroRate = [&, this] {
      //     return units::radians_per_second_t(
      //                units::degrees_per_second_t{m_gyro->GetAngle()})
      //         .to<double>();
      //   };
      // } else {
      // Default behaviour is to make the gyro functions return zero
    } else {
      m_gyroPosition = [&] { return 0.0; };
      m_gyroRate = [&] { return 0.0; };
    }

    // }
  } catch (std::exception& e) {
    fmt::print("Project failed: {}\n", e.what());
    std::exit(-1);
  }
  m_logger.UpdateThreadPriority();

#ifdef INTEGRATION
  frc::SmartDashboard::PutBoolean("SysIdRun", false);
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
    frc::SmartDashboard::PutNumber("Left Position", m_leftPosition());
    frc::SmartDashboard::PutNumber("Right Position", m_rightPosition());
    frc::SmartDashboard::PutNumber("Left Velocity", m_leftRate());
    frc::SmartDashboard::PutNumber("Right Position", m_rightRate());

    frc::SmartDashboard::PutNumber("Gyro Reading", m_gyroPosition());
    frc::SmartDashboard::PutNumber("Gyro Rate", m_gyroRate());
  } catch (std::exception& e) {
    fmt::print("Project failed: {}\n", e.what());
    std::exit(-1);
  }
}

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
  m_logger.Log(m_leftPosition(), m_rightPosition(), m_leftRate(), m_rightRate(),
               m_gyroPosition(), m_gyroRate());
  SetMotorControllers(m_logger.GetLeftMotorVoltage(), m_leftControllers);
  SetMotorControllers(m_logger.GetRightMotorVoltage(), m_rightControllers);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  SetMotorControllers(0_V, m_leftControllers);
  SetMotorControllers(0_V, m_rightControllers);
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

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
