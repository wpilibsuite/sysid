// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

class SysIdMechanism {
 public:
  /**
   * Sets the primary (left motor for drivetrain) percentage.
   */
  virtual void SetPMotor(double value) = 0;

  /**
   * Sets the secondary motor (right for drivetrain) percentage. This method
   * does not need to be implemented for other mechanisms.
   */
  virtual void SetSMotor(double value) {}

  /**
   * Gets the primary encoder (left encoder for drivetrain) distance.
   */
  virtual double GetPEncDistance() = 0;

  /**
   * Gets the secondary encoder (right encoder for drivetrain) distance. This
   * method does not need to be implemented for other mechanisms.
   */
  virtual double GetSEncDistance() { return 0.0; }

  /**
   * Gets the primary encoder (left encoder for drivetrain) velocity.
   */
  virtual double GetPEncVelocity() = 0;

  /**
   * Gets the secondary encoder (right encoder for drivetrain) velocity. This
   * method does not need to be implemented for other mechanisms.
   */
  virtual double GetSEncVelocity() { return 0.0; }

  /**
   * Returns the unbounded gyro angle. This method does not need to be
   * implemented if the mechanism is not a drivetrain.
   */
  virtual double GetGyroAngle() { return 0.0; }
};
