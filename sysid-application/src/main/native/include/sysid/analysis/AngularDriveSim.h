// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Core>
#include <units/time.h>
#include <units/voltage.h>

namespace sysid {
/**
 * Simulates an angular drivetrain. This is primarily intended for unit testing
 * purposes.
 */
class AngularDriveSim {
 public:
  /**
   * @param Ks              Static friction gain in linear units.
   * @param Kv              Velocity gain in linear units.
   * @param Ka              Acceleration gain in linear units.
   * @param initialPosition Initial angle in radians.
   * @param initialVelocity Initial angular velocity in radians per second.
   */
  AngularDriveSim(double kS, double kV, double kA, double trackwidth,
                  double initialPosition = 0.0, double initialVelocity = 0.0);

  /**
   * Simulates affine state-space system dx/dt = Ax + Bu + c sgn(x) forward dt
   * seconds.
   *
   * @param voltage Voltage to apply over the timestep.
   * @param dt      Sammple period.
   */
  void Update(units::volt_t voltage, units::second_t dt);

  /**
   * Returns the position in the units of the trackwidth.
   */
  double GetPosition() const;

  /**
   * Returns the velocity in the units of the trackwidth.
   */
  double GetVelocity() const;

  /**
   * Returns the acceleration for the current state and given input in the units
   * of the trackwidth.
   */
  double GetAcceleration(units::volt_t voltage) const;

  /**
   * Resets model position and velocity.
   */
  void Reset(double position = 0.0, double velocity = 0.0);

 private:
  Eigen::Matrix<double, 2, 2> m_A;
  Eigen::Matrix<double, 2, 1> m_B;
  Eigen::Vector<double, 2> m_c;
  Eigen::Vector<double, 2> m_x;
  double m_trackwidth;
};

}  // namespace sysid
