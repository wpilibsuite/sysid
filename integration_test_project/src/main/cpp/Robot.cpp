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
#include <frc/simulation/DriverStationSim.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/Timer.h>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  Robot() : frc::TimedRobot(5_ms) {}
  void RobotInit() override {
    // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    SetNetworkTablesFlushEnabled(true);
  }

  void DisabledInit() override {
    m_drive.SetPercent(0, 0);

    if (m_counter > 0) {
      wpi::outs() << "Collected " << m_counter << " data points.\n";

      std::stringstream ss;
      std::for_each(m_data.begin(), m_data.end() - 1,
                    [&ss](auto& pt) { ss << std::to_string(pt) << ","; });
      ss << m_data.back();

      frc::SmartDashboard::PutString("SysIdTelemetry", ss.str());

      m_counter = 0;
      m_data.clear();
    }
  }

  void RobotPeriodic() override { m_drive.Periodic(); }

  void AutonomousPeriodic() override {
    double speed = frc::SmartDashboard::GetNumber("SysIdAutoSpeed", 0.0);
    bool rotate = frc::SmartDashboard::GetBoolean("SysIdRotate", false);

    m_drive.SetPercent((rotate ? -1 : 1) * speed, speed);
    m_drive.UpdateOdometry();

    double voltage = frc::RobotController::GetInputVoltage();

    std::array<double, 10> arr{frc2::Timer::GetFPGATimestamp().to<double>(),
                               voltage,
                               speed,
                               speed * voltage,
                               speed * voltage,
                               m_drive.GetLEnc().GetDistance(),
                               m_drive.GetREnc().GetDistance(),
                               m_drive.GetLEnc().GetRate(),
                               m_drive.GetREnc().GetRate(),
                               m_drive.GetGyro().to<double>()};

    frc::SmartDashboard::PutNumber("Speed", speed);

    m_data.insert(m_data.end(), arr.cbegin(), arr.cend());
    m_counter++;
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

    bool enable = frc::SmartDashboard::GetBoolean("SysIdRun", false);
    frc::sim::DriverStationSim::SetAutonomous(enable);
    frc::sim::DriverStationSim::SetEnabled(enable);
    frc::sim::DriverStationSim::NotifyNewData();

    if (frc::SmartDashboard::GetBoolean("SysIdKill", false)) {
      std::exit(0);
    }
  }

 private:
  frc::XboxController m_controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  Drivetrain m_drive;

  std::vector<double> m_data;
  size_t m_counter = 0;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
