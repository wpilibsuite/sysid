// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>

class SysIdDrivetrain {
 public:
  virtual void SetLMotor(units::volt_t value) = 0;
  virtual void SetRMotor(units::volt_t value) = 0;

  virtual double GetLEncDistance() = 0;
  virtual double GetREncDistance() = 0;

  virtual double GetLEncVelocity() = 0;
  virtual double GetREncVelocity() = 0;

  virtual double GetGyroAngle() = 0;
  virtual double GetGyroAngularRate() = 0;
};
