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
#include <frc/simulation/FlywheelSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <wpi/math>

#include "Constants.h"
#include "interface/SysIdGeneralMechanism.h"

/**
 * Represents a flywheel mechanism.
 */
class SimpleMotor : public SysIdGeneralMechanism {
 public:
  SimpleMotor() {
    // Set the distance per pulse for the flywheel encoders. We can simply use
    // the 1 divided by the resolution as that denotes one rotation of the
    // flywheel.
    m_encoder.SetDistancePerPulse(2 * wpi::math::pi / kEncoderResolution);

    m_encoder.Reset();
  }

  void SetMotor(units::volt_t value) override { m_group.SetVoltage(value); }

  double GetPosition() override { return m_encoder.GetDistance(); }
  double GetVelocity() override { return m_encoder.GetRate(); }

  void SimulationPeriodic();

  void Periodic() {
    frc::SmartDashboard::PutNumber("Flywheel Speed", m_encoder.GetRate());
  }

 private:
  static constexpr int kEncoderResolution = 4096;
  double distance = 0;

  frc::PWMVictorSPX m_leader{Constants::SimpleMotor::kLeaderPort};
  frc::PWMVictorSPX m_follower{Constants::SimpleMotor::kFollowerPort};

  frc::SpeedControllerGroup m_group{m_leader, m_follower};

  frc::Encoder m_encoder{Constants::SimpleMotor::kEncoderPorts[0],
                         Constants::SimpleMotor::kEncoderPorts[1]};

  // Simulation classes help us simulate our robot
  frc::sim::EncoderSim m_encoderSim{m_encoder};
  frc::LinearSystem<1, 1, 1> m_flywheelSystem =
      frc::LinearSystemId::IdentifyVelocitySystem<units::radians>(
          Constants::SimpleMotor::kV, Constants::SimpleMotor::kA);
  frc::sim::FlywheelSim m_flywheelSimulator{m_flywheelSystem,
                                            frc::DCMotor::Vex775Pro(2), 1.0};
};
