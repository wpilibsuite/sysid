// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

#include <frc/RobotController.h>

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  double leftOutput = m_leftPIDController.Calculate(m_leftEncoder.GetRate(),
                                                    speeds.left.value());
  double rightOutput = m_rightPIDController.Calculate(m_rightEncoder.GetRate(),
                                                      speeds.right.value());

  m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t(m_rightEncoder.GetDistance()));
}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(pose, m_gyro.GetRotation2d());
}

void Drivetrain::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  m_drivetrainSimulator.SetInputs(units::volt_t{m_leftLeader.Get()} *
                                      frc::RobotController::GetInputVoltage(),
                                  units::volt_t{m_rightLeader.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(5_ms);

  m_leftEncoderSim.SetDistance(m_drivetrainSimulator.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_drivetrainSimulator.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(
      m_drivetrainSimulator.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_drivetrainSimulator.GetRightVelocity().value());

  frc::Rotation2d angle = m_drivetrainSimulator.GetHeading();

  m_gyroSim.SetAngle(-angle.Degrees().value());
  m_gyroSim.SetRate((m_drivetrainSimulator.GetRightVelocity().value() -
                     m_drivetrainSimulator.GetLeftVelocity().value()) /
                    Constants::Drivetrain::kTrackWidth.value());
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
}
