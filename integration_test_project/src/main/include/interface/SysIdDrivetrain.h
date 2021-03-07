// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

class SysIdDrivetrain {
 public:
  virtual void SetLMotor(double value) = 0;
  virtual void SetRMotor(double value) = 0;

  virtual double GetLEncDistance() = 0;
  virtual double GetREncDistance() = 0;

  virtual double GetLEncVelocity() = 0;
  virtual double GetREncVelocity() = 0;

  virtual double GetGyroAngle() = 0;
  virtual double GetGyroAngularRate() = 0;
};
