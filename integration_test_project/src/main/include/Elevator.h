// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/RobotController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <wpi/math>

#include "Constants.h"

/**
 * Represents a differential drive style drivetrain.
 */
class Elevator {
 public:
  Elevator() {
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
    m_elevatorSimulator.SetState(frc::MakeMatrix<2, 1>(0.0, 0.0));
  }

  void Periodic() {
    frc::SmartDashboard::PutNumber("Elevator Speed", m_encoder.GetRate());
  }

 private:
  static constexpr int kEncoderResolution = 4096;
  double distance = 0;

  frc::PWMVictorSPX m_leader{Constants::Elevator::kLeaderPort};
  frc::PWMVictorSPX m_follower{Constants::Elevator::kFollowerPort};

  frc::SpeedControllerGroup m_group{m_leader, m_follower};

  frc::Encoder m_encoder{Constants::Elevator::kEncoderPorts[0],
                         Constants::Elevator::kEncoderPorts[1]};

  // Simulation classes help us simulate our robot
  frc::sim::EncoderSim m_encoderSim{m_encoder};
  frc::LinearSystem<2, 1, 1> m_elevatorSystem =
      frc::LinearSystemId::IdentifyPositionSystem<units::radians>(
          Constants::Elevator::kV, Constants::Elevator::kA);
  frc::sim::ElevatorSim m_elevatorSimulator{m_elevatorSystem,
                                            frc::DCMotor::Vex775Pro(2),
                                            1.0,
                                            1_m,
                                            -Constants::Elevator::kHeight,
                                            Constants::Elevator::kHeight};
};
