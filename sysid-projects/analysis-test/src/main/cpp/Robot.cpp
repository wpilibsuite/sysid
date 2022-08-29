// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>
#include <vector>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

#include "Arm.h"
#include "Drivetrain.h"
#include "Elevator.h"
#include "SimpleMotor.h"
#include "interface/SysIdDrivetrain.h"
#include "interface/SysIdGeneralMechanism.h"
#include "sysid/logging/SysIdDrivetrainLogger.h"
#include "sysid/logging/SysIdGeneralMechanismLogger.h"

class Robot : public frc::TimedRobot {
 public:
  Robot() : frc::TimedRobot(5_ms) { m_driveLogger.UpdateThreadPriority(); }
  void RobotInit() override {
    // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    SetNetworkTablesFlushEnabled(true);
    frc::SmartDashboard::PutString("SysIdTest", "Drivetrain");
  }

  void DisabledInit() override {
    m_mechanism->SetMotor(0_V);

    m_driveMechanism->SetLMotor(0_V);
    m_driveMechanism->SetRMotor(0_V);

    m_arm.ResetReadings();
    if (m_test == "Drivetrain" || m_test == "Drivetrain (Angular)") {
      m_driveLogger.SendData();
    } else {
      m_generalLogger.SendData();
    }
  }

  void DisabledPeriodic() override {
    m_arm.ResetReadings();
    if (m_test == "Drivetrain" || m_test == "Drivetrain (Angular)") {
      m_driveLogger.ClearWhenReceived();
    } else {
      m_generalLogger.ClearWhenReceived();
    }
  }

  void RobotPeriodic() override {
    m_drive.Periodic();
    m_flywheel.Periodic();
    m_arm.Periodic();
  }

  void AutonomousInit() override {
    m_test = frc::SmartDashboard::GetString("SysIdTest", "Simple");
    if (m_test == "Drivetrain" || m_test == "Drivetrain (Angular)") {
      m_driveLogger.InitLogging();
    } else {
      m_generalLogger.InitLogging();
    }

    m_elevator.UpdateInitialSpeed();
    m_arm.ResetReadings();

    if (m_test == "Simple") {
      m_mechanism = &m_flywheel;
    } else if (m_test == "Elevator") {
      m_mechanism = &m_elevator;
    } else if (m_test == "Arm") {
      m_mechanism = &m_arm;
    }
  }

  void AutonomousPeriodic() override {
    if (m_test == "Drivetrain" || m_test == "Drivetrain (Angular)") {
      m_driveLogger.Log(m_driveLogger.GetLeftMotorVoltage().value(),
                        m_driveLogger.GetRightMotorVoltage().value(),
                        m_driveMechanism->GetLEncDistance(),
                        m_driveMechanism->GetREncDistance(),
                        m_driveMechanism->GetLEncVelocity(),
                        m_driveMechanism->GetREncVelocity(),
                        m_driveMechanism->GetGyroAngle(),
                        m_driveMechanism->GetGyroAngularRate());
      m_driveMechanism->SetLMotor(m_driveLogger.GetLeftMotorVoltage());
      m_driveMechanism->SetRMotor(m_driveLogger.GetRightMotorVoltage());
    } else {
      m_generalLogger.Log(m_generalLogger.GetMotorVoltage().value(),
                          m_mechanism->GetPosition(),
                          m_mechanism->GetVelocity());
      m_mechanism->SetMotor(m_generalLogger.GetMotorVoltage());
    }
  }

  void TeleopPeriodic() override {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_speedLimiter.Calculate(m_controller.GetLeftY()) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rot = -m_rotLimiter.Calculate(m_controller.GetRightX()) *
               Drivetrain::kMaxAngularSpeed;

    m_drive.Drive(xSpeed, rot);
  }

  void SimulationPeriodic() override {
    m_drive.SimulationPeriodic();
    m_flywheel.SimulationPeriodic();
    m_elevator.SimulationPeriodic();
    m_arm.SimulationPeriodic();

    frc::SmartDashboard::PutString("ServerTest", m_test);

#ifdef INTEGRATION
    bool enable = frc::SmartDashboard::GetBoolean("SysIdRun", false);
    frc::sim::DriverStationSim::SetAutonomous(enable);
    frc::sim::DriverStationSim::SetEnabled(enable);
    frc::sim::DriverStationSim::NotifyNewData();
#endif

    if (frc::SmartDashboard::GetBoolean("SysIdKill", false)) {
      EndCompetition();
    }
  }

 private:
  frc::XboxController m_controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  Drivetrain m_drive;
  SimpleMotor m_flywheel;
  Elevator m_elevator;
  Arm m_arm;

  SysIdGeneralMechanism* m_mechanism = &m_flywheel;
  SysIdDrivetrain* m_driveMechanism = &m_drive;
  sysid::SysIdGeneralMechanismLogger m_generalLogger;
  sysid::SysIdDrivetrainLogger m_driveLogger;

  std::string m_test = "Drivetrain";
  std::vector<double> m_data;
};

#ifndef RUNNING_FRC_TESTS
int main() {  // NOLINT (bugprone-exception-escape)
  return frc::StartRobot<Robot>();
}
#endif
