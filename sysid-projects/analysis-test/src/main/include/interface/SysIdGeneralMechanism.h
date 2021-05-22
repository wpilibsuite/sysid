// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>

class SysIdGeneralMechanism {
 public:
  virtual double GetPosition() = 0;
  virtual double GetVelocity() = 0;

  virtual void SetMotor(units::volt_t value) = 0;
};
