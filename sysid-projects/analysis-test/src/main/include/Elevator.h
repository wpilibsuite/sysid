// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/RobotController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <wpi/numbers>

#include "Constants.h"
#include "interface/SysIdGeneralMechanism.h"

/**
 * Represents an elevator mechanism.
 */
class Elevator : public SysIdGeneralMechanism {
 public:
  Elevator() {
    // Set the distance per pulse for the flywheel encoders. We can simply use
    // the 1 divided by the resolution as that denotes one rotation of the
    // flywheel.
    m_encoder.SetDistancePerPulse(2 * wpi::numbers::pi / kEncoderResolution);

    m_encoder.Reset();
  }

  void SetMotor(units::volt_t value) override { m_group.SetVoltage(value); }

  double GetPosition() override { return m_encoder.GetDistance(); }
  double GetVelocity() override { return m_encoder.GetRate() - m_initSpeed; }

  void SimulationPeriodic();

  void ResetReadings() {
    m_elevatorSimulator.SetState(Eigen::Matrix<double, 2, 1>{0.0, 0.0});
    m_encoderSim.SetRate(0);
    m_encoderSim.SetDistance(0);
  }

  void Periodic() {
    frc::SmartDashboard::PutNumber("Elevator Speed", m_encoder.GetRate());
    frc::SmartDashboard::PutNumber("Elevator Position",
                                   m_encoder.GetDistance());
  }

  void UpdateInitialSpeed() { m_initSpeed = m_encoder.GetRate(); }

 private:
  static constexpr int kEncoderResolution = 4096;
  double distance = 0;

  double m_initSpeed = 0;

  frc::PWMVictorSPX m_leader{Constants::Elevator::kLeaderPort};
  frc::PWMVictorSPX m_follower{Constants::Elevator::kFollowerPort};

  frc::MotorControllerGroup m_group{m_leader, m_follower};

  frc::Encoder m_encoder{Constants::Elevator::kEncoderPorts[0],
                         Constants::Elevator::kEncoderPorts[1]};

  // Simulation classes help us simulate our robot
  frc::sim::EncoderSim m_encoderSim{m_encoder};
  frc::LinearSystem<2, 1, 1> m_elevatorSystem =
      frc::LinearSystemId::IdentifyPositionSystem<units::radians>(
          Constants::Elevator::kV, Constants::Elevator::kA);
  frc::sim::ElevatorSim m_elevatorSimulator{m_elevatorSystem,
                                            frc::DCMotor::Vex775Pro(4),
                                            1000,
                                            2_in,
                                            -Constants::Elevator::kHeight,
                                            Constants::Elevator::kHeight};
};
