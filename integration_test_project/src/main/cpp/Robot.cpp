// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <sstream>
#include <vector>

#include <frc/RobotController.h>
#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/RamseteController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/Timer.h>
#include <wpi/StringRef.h>

#include "Arm.h"
#include "Drivetrain.h"
#include "Elevator.h"
#include "SimpleMotor.h"
#include "interface/SysIdDrivetrain.h"
#include "interface/SysIdGeneralMechanism.h"

class Robot : public frc::TimedRobot {
 public:
  Robot() : frc::TimedRobot(5_ms) {
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();
  }
  void RobotInit() override {
    // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    SetNetworkTablesFlushEnabled(true);
    frc::SmartDashboard::PutString("SysIdTest", "Drivetrain");
    frc::SmartDashboard::PutNumber("SysIdAutoSpeed", 0.0);
  }

  void DisabledInit() override {
    m_mechanism->SetMotor(0);

    m_driveMechanism->SetLMotor(0);
    m_driveMechanism->SetRMotor(0);

    m_arm.ResetReadings();

    if (m_counter > 0) {
      wpi::outs() << "Collected " << m_counter << " data points.\n";

      std::stringstream ss;
      std::for_each(m_data.begin(), m_data.end(),
                    [&ss](auto& pt) { ss << std::to_string(pt) << ", "; });

      frc::SmartDashboard::PutString("SysIdTelemetry", ss.str());

      m_counter = 0;
      m_data.clear();
    }
  }

  void DisabledPeriodic() override { m_arm.ResetReadings(); }

  void RobotPeriodic() override {
    m_drive.Periodic();
    m_flywheel.Periodic();
    m_arm.Periodic();
  }

  void AutonomousInit() override {
    m_elevator.UpdateInitialSpeed();
    m_arm.ResetReadings();

    frc::SmartDashboard::PutString("SysIdTelemetry", "");

    if (m_test == "Simple") {
      m_mechanism = &m_flywheel;
    } else if (m_test == "Elevator") {
      m_mechanism = &m_elevator;
    } else if (m_test == "Arm") {
      m_mechanism = &m_arm;
    }
  }

  void AutonomousPeriodic() override {
    double speed = frc::SmartDashboard::GetNumber("SysIdAutoSpeed", 0.0);
    bool rotate = frc::SmartDashboard::GetBoolean("SysIdRotate", false);
    double voltage = frc::RobotController::GetInputVoltage();

    if (m_test == "Drivetrain" || m_test == "Drivetrain (Angular)") {
      std::array<double, 9> arr = {frc2::Timer::GetFPGATimestamp().to<double>(),
                                   speed * voltage,
                                   speed * voltage,
                                   m_driveMechanism->GetLEncDistance(),
                                   m_driveMechanism->GetREncDistance(),
                                   m_driveMechanism->GetLEncVelocity(),
                                   m_driveMechanism->GetREncVelocity(),
                                   m_driveMechanism->GetGyroAngle(),
                                   m_driveMechanism->GetGyroAngularRate()};

      m_driveMechanism->SetLMotor(speed);
      m_driveMechanism->SetRMotor(speed * (rotate ? 1 : -1));

      m_data.insert(m_data.end(), arr.cbegin(), arr.cend());
      m_counter++;
    } else {
      std::array<double, 4> arr = {frc2::Timer::GetFPGATimestamp().to<double>(),
                                   speed * voltage, m_mechanism->GetPosition(),
                                   m_mechanism->GetVelocity()};

      m_mechanism->SetMotor(speed);
      m_data.insert(m_data.end(), arr.cbegin(), arr.cend());
      m_counter++;
    }
  }

  void TeleopPeriodic() override {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_speedLimiter.Calculate(
                            m_controller.GetY(frc::GenericHID::kLeftHand)) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rot = -m_rotLimiter.Calculate(
                   m_controller.GetX(frc::GenericHID::kRightHand)) *
               Drivetrain::kMaxAngularSpeed;

    m_drive.Drive(xSpeed, rot);
  }

  void SimulationPeriodic() override {
    m_drive.SimulationPeriodic();
    m_flywheel.SimulationPeriodic();
    m_elevator.SimulationPeriodic();
    m_arm.SimulationPeriodic();

    m_test = frc::SmartDashboard::GetString("SysIdTest", "Simple");
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

  std::string m_test = "Drivetrain";
  std::vector<double> m_data;
  size_t m_counter = 0;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
