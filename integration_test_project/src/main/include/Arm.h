// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/RobotController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <wpi/math>

#include "Constants.h"

/**
 * Represents a differential drive style drivetrain.
 */
class Arm {
 public:
  Arm() {
    // Set the distance per pulse for the flywheel encoders. We can simply use
    // the 1 divided by the resolution as that denotes one rotation of the
    // flywheel.
    m_encoder.SetDistancePerPulse(2 * wpi::math::pi / kEncoderResolution);

    m_encoder.Reset();
  }

  void SetPercent(double power) { m_group.Set(power); }

  frc::Encoder& GetEnc() { return m_encoder; }

  double GetSpeed() { return m_group.Get(); }

  void SimulationPeriodic();

  void ResetReadings() {
    m_armSimulator.SetState(frc::MakeMatrix<2, 1>(0.0, 0.0));
  }

  void Periodic() {
    frc::SmartDashboard::PutNumber("Arm Speed", m_encoder.GetRate());
    frc::SmartDashboard::PutNumber("Arm Position", m_encoder.GetDistance());
    frc::SmartDashboard::PutNumber(
        "Current Draw", m_armSimulator.GetCurrentDraw().to<double>());
    frc::SmartDashboard::PutBoolean("Hit Top Limit",
                                    m_armSimulator.HasHitUpperLimit());
    frc::SmartDashboard::PutBoolean("Hit bottom Limit",
                                    m_armSimulator.HasHitLowerLimit());
  }

 private:
  static constexpr int kEncoderResolution = 4096;
  double distance = 0;

  frc::PWMVictorSPX m_leader{Constants::Arm::kLeaderPort};
  frc::PWMVictorSPX m_follower{Constants::Arm::kFollowerPort};

  frc::SpeedControllerGroup m_group{m_leader, m_follower};

  frc::Encoder m_encoder{Constants::Arm::kEncoderPorts[0],
                         Constants::Arm::kEncoderPorts[1]};

  // Simulation classes help us simulate our robot
  frc::sim::EncoderSim m_encoderSim{m_encoder};
  frc::LinearSystem<2, 1, 1> m_armSystem =
      frc::LinearSystemId::IdentifyPositionSystem<units::radians>(
          Constants::Arm::kV, Constants::Arm::kA);
  frc::sim::SingleJointedArmSim m_armSimulator{m_armSystem,
                                               frc::DCMotor::Vex775Pro(4),
                                               .1,
                                               30_in,
                                               -Constants::Arm::kAngle,
                                               Constants::Arm::kAngle,
                                               Constants::Arm::kMass,
                                               true};
};
