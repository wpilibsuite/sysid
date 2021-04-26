// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <rev/CANSparkMax.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <sstream>

#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <frc/Spark.h>
#include <frc/TimedRobot.h>
// #include <frc/romi/RomiGyro.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <wpi/FileSystem.h>
#include <wpi/SmallString.h>
#include <wpi/StringMap.h>
#include <wpi/StringRef.h>

// #include "AHRS.h"
#include "generation/SysIdSetup.h"
#include "rev/CANEncoder.h"

Robot::Robot() : frc::TimedRobot(5_ms) {
  m_json = GetConfigJson();

  try {
    wpi::outs() << "Read JSON\n";
    wpi::outs().flush();
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
    double cpr = m_json.at("counts per shaft revolution").get<double>();

    std::string gyroType = m_json.at("gyro").get<std::string>();
    std::string gyroCtorString = m_json.at("gyro ctor").get<std::string>();
    wpi::StringRef gyroCtor{gyroCtorString};

    bool isEncoding = m_json.at("encoding").get<bool>();

    int numSamples = m_json.at("number of samples per average").get<int>();

    int period = m_json.at("velocity measurement period").get<int>();

    wpi::outs() << "Setup motors\n";
    wpi::outs().flush();
    for (size_t i = 0; i < leftPorts.size(); i++) {
      AddMotorController(leftPorts[i], controllerNames[i],
                         leftMotorsInverted[i], &m_leftControllers);
      AddMotorController(rightPorts[i], controllerNames[i],
                         rightMotorsInverted[i], &m_rightControllers);
    }

    wpi::outs() << "Setup encoders\n";
    wpi::outs().flush();
    SetupEncoders(encoderType, isEncoding, period, cpr, numSamples,
                  controllerNames[0], m_leftControllers.at(0).get(),
                  leftEncoderInverted, leftEncoderPorts, m_leftCancoder,
                  m_leftEncoder, m_leftPosition, m_leftRate);
    SetupEncoders(encoderType, isEncoding, period, cpr, numSamples,
                  controllerNames[0], m_rightControllers.at(0).get(),
                  rightEncoderInverted, rightEncoderPorts, m_rightCancoder,
                  m_rightEncoder, m_rightPosition, m_rightRate);

    wpi::outs() << "setup gyro\n";
    wpi::outs().flush();
    if (gyroType == "Pigeon") {
      std::string port_str = "";
      if (gyroCtor.contains("WPI_TalonSRX")) {
        port_str = std::string{gyroCtor[gyroCtor.find("-") + 1]};
      } else {
        port_str = std::string{gyroCtor};
      }

      wpi::outs() << port_str << "\n";
      wpi::outs().flush();
      // converts gyroCtor into port #
      int srx_port = std::stoi(port_str);
      if (gyroCtor.contains("WPI_TalonSRX")) {
        bool talonDeclared = false;
        // Check if there is a Talon Port in Left Ports
        auto find_port =
            std::find(leftPorts.begin(), leftPorts.end(), srx_port);

        // Check Right Ports if not found
        if (find_port == leftPorts.end()) {
          find_port = std::find(rightPorts.begin(), rightPorts.end(), srx_port);
          if (find_port != rightPorts.end() &&
              controllerNames[find_port - rightPorts.begin()] == "TalonSRX") {
            m_pigeon = std::make_unique<PigeonIMU>(dynamic_cast<WPI_TalonSRX*>(
                m_rightControllers.at(find_port - rightPorts.begin()).get()));
            talonDeclared = true;
          }
        } else if (controllerNames[find_port - leftPorts.begin()] ==
                   "TalonSRX") {
          m_pigeon = std::make_unique<PigeonIMU>(dynamic_cast<WPI_TalonSRX*>(
              m_leftControllers.at(find_port - leftPorts.begin()).get()));
          talonDeclared = true;
        }

        // If it isn't tied to an existing Talon, create a new object
        if (!talonDeclared) {
          m_pigeon = std::make_unique<PigeonIMU>(new WPI_TalonSRX(srx_port));
        }
      } else {
        m_pigeon = std::make_unique<PigeonIMU>(srx_port);
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

    } else if (gyroType != "None") {
      if (gyroType == "ADXRS450") {
        if (gyroCtor == "SPI.kMXP") {
          m_gyro = std::make_unique<frc::ADXRS450_Gyro>(frc::SPI::Port::kMXP);
        } else {
          m_gyro =
              std::make_unique<frc::ADXRS450_Gyro>(frc::SPI::Port::kOnboardCS0);
        }
        // FIXME: waiting on Linux and macOS builds for navX AHRS
        // } else if (gyroType == "NavX") {
        //   if (gyroCtor == "SerialPort.kUSB") {
        //     m_gyro = std::make_unique<AHRS>(frc::SerialPort::Port::kUSB);
        //   } else if (gyroCtor == "I2C") {
        //     m_gyro = std::make_unique<AHRS>(frc::I2C::Port::kMXP);
        //   } else if (gyroCtor == "SerialPort.kMXP") {
        //     m_gyro = std::make_unique<AHRS>(frc::SerialPort::Port::kMXP);
        //   } else {
        //     m_gyro = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
        //   }
        // FIXME: Update Romi Gyro once vendordep is out
        // } else if (gyroType == "Romi") {
        //   m_gyro = std::make_unique<frc::RomiGyro>();
      } else {
        std::stringstream ss(gyroCtor);
        int port = 0;
        ss >> port;
        m_gyro = std::make_unique<frc::AnalogGyro>(port);
      }

      m_gyroPosition = [&, this] {
        return units::radian_t(units::degree_t{m_gyro->GetAngle()})
            .to<double>();
      };

      m_gyroRate = [&, this] {
        return units::radians_per_second_t(
                   units::degrees_per_second_t{m_gyro->GetAngle()})
            .to<double>();
      };
    } else {
      // Default behaviour is to make the gyro functions return zero
      m_gyroPosition = [&, this] { return 0.0; };
      m_gyroRate = [&, this] { return 0.0; };
    }
  } catch (std::exception& e) {
    wpi::outs() << "Project failed: " << e.what() << "\n";
    wpi::outs().flush();
    std::exit(-1);
  }
  m_logger.UpdateThreadPriority();

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
  // TODO Enable once sim is fixed
  // try {
  frc::SmartDashboard::PutNumber("Left Position", m_leftPosition());
  frc::SmartDashboard::PutNumber("Right Position", m_rightPosition());
  frc::SmartDashboard::PutNumber("Left Velocity", m_leftRate());
  frc::SmartDashboard::PutNumber("Right Position", m_rightRate());

  frc::SmartDashboard::PutNumber("Gyro Reading", m_gyroPosition());
  frc::SmartDashboard::PutNumber("Gyro Rate", m_gyroRate());

  // } catch (std::exception& e) {
  //   wpi::outs() << "Project failed: " << e.what() << "\n";
  //   wpi::outs().flush();
  //   std::exit(-1);
  // }
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
  SetMotorControllers(units::volt_t{0}, m_leftControllers);
  SetMotorControllers(units::volt_t{0}, m_rightControllers);
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
