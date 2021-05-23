// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/math>

#include "Constants.h"
#include "interface/SysIdDrivetrain.h"

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain : public SysIdDrivetrain {
 public:
  Drivetrain() {
    m_gyro.Reset();
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.SetDistancePerPulse(
        2 * wpi::math::pi * Constants::Drivetrain::kWheelRadius /
        Constants::Drivetrain::kEncoderResolution);
    m_rightEncoder.SetDistancePerPulse(
        2 * wpi::math::pi * Constants::Drivetrain::kWheelRadius /
        Constants::Drivetrain::kEncoderResolution);

    m_leftEncoder.Reset();
    m_rightEncoder.Reset();

    frc::SmartDashboard::PutData("Field", &m_fieldSim);
  }

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi};  // 1/2 rotation per second

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::radians_per_second_t rot);
  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d& pose);

  double GetLEncDistance() override { return m_leftEncoder.GetDistance(); }
  double GetLEncVelocity() override { return m_leftEncoder.GetRate(); }
  void SetLMotor(units::volt_t value) override {
    m_leftGroup.SetVoltage(value);
  }

  void SetRMotor(units::volt_t value) override {
    m_rightGroup.SetVoltage(value);
  }

  double GetREncDistance() override { return m_rightEncoder.GetDistance(); }
  double GetREncVelocity() override { return m_rightEncoder.GetRate(); }

  double GetGyroAngle() override { return GetGyro().to<double>(); }
  double GetGyroAngularRate() override {
    return -m_gyro.GetRate() * wpi::math::pi / 180;
  }

  double GetSpeed() { return (m_leftGroup.Get() + m_rightGroup.Get()) / 2; }

  units::radian_t GetGyro() { return units::degree_t(-m_gyro.GetAngle()); }

  frc::Pose2d GetPose() const { return m_odometry.GetPose(); }

  void SimulationPeriodic();
  void Periodic();

 private:
  frc::PWMVictorSPX m_leftLeader{Constants::Drivetrain::kLeftLeaderPort};
  frc::PWMVictorSPX m_leftFollower{Constants::Drivetrain::kLeftFollowerPort};
  frc::PWMVictorSPX m_rightLeader{Constants::Drivetrain::kRightLeaderPort};
  frc::PWMVictorSPX m_rightFollower{Constants::Drivetrain::kRightFollowerPort};

  frc::SpeedControllerGroup m_leftGroup{m_leftLeader, m_leftFollower};
  frc::SpeedControllerGroup m_rightGroup{m_rightLeader, m_rightFollower};

  frc::Encoder m_leftEncoder{Constants::Drivetrain::kLeftEncoderPorts[0],
                             Constants::Drivetrain::kLeftEncoderPorts[1]};
  frc::Encoder m_rightEncoder{Constants::Drivetrain::kRightEncoderPorts[0],
                              Constants::Drivetrain::kRightEncoderPorts[0]};

  frc2::PIDController m_leftPIDController{8.5, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{8.5, 0.0, 0.0};

  frc::AnalogGyro m_gyro{Constants::Drivetrain::kGyroPort};

  frc::DifferentialDriveKinematics m_kinematics{
      Constants::Drivetrain::kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{1_V, 3_V / 1_mps};

  // Simulation classes help us simulate our robot
  frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
  frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
  frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
  frc::Field2d m_fieldSim;
  frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
      frc::LinearSystemId::IdentifyDrivetrainSystem(
          Constants::Drivetrain::kV, Constants::Drivetrain::kA,
          Constants::Drivetrain::kAngularKV, Constants::Drivetrain::kAngularKA);
  frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
      m_drivetrainSystem, Constants::Drivetrain::kTrackWidth,
      frc::DCMotor::CIM(2), 8, 2_in};

  frc::Rotation2d m_prevAngle = 0_rad;
};
