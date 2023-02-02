// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generation/HardwareType.h"

#include <exception>
#include <stdexcept>

#include <fmt/format.h>

using namespace sysid;

HardwareType sysid::motorcontroller::FromMotorControllerName(
    std::string_view name) {
  if (name == "PWM") {
    return sysid::motorcontroller::kPWM;
  }
  if (name == "TalonSRX") {
    return sysid::motorcontroller::kTalonSRX;
  }
  if (name == "VictorSPX") {
    return sysid::motorcontroller::kVictorSPX;
  }
  if (name == "TalonFX") {
    return sysid::motorcontroller::kTalonFX;
  }
  if (name == "TalonFX (Pro)") {
    return sysid::motorcontroller::kTalonFXPro;
  }
  if (name == "SPARK MAX (Brushless)") {
    return sysid::motorcontroller::kSPARKMAXBrushless;
  }
  if (name == "SPARK MAX (Brushed)") {
    return sysid::motorcontroller::kSPARKMAXBrushed;
  }
  if (name == "Venom") {
    return sysid::motorcontroller::kVenom;
  }
  throw std::runtime_error(
      fmt::format("Unsupported Motor Controller Name: {}", name));
}

HardwareType sysid::encoder::FromEncoderName(std::string_view name) {
  if (name == "roboRIO quadrature") {
    return sysid::encoder::kRoboRIO;
  }
  if (name == "CANCoder") {
    return sysid::encoder::kCANCoder;
  }
  if (name == "CANcoder (Pro)") {
    return sysid::encoder::kCANcoderPro;
  }
  if (name == "Built-in") {
    return sysid::encoder::kBuiltInSetting;
  }
  if (name == "Tachometer") {
    return sysid::encoder::kCTRETachometer;
  }
  if (name == "Encoder Port") {
    return sysid::encoder::kSMaxEncoderPort;
  }
  if (name == "Data Port") {
    return sysid::encoder::kSMaxDataPort;
  }
  throw std::runtime_error(fmt::format("Unsupported Encoder Name: {}", name));
}

HardwareType sysid::gyro::FromGyroName(std::string_view name) {
  if (name == "Analog Gyro") {
    return sysid::gyro::kAnalogGyro;
  }
  if (name == "ADXRS450") {
    return sysid::gyro::kADXRS450;
  }
  if (name == "NavX") {
    return sysid::gyro::kNavX;
  }
  if (name == "Pigeon") {
    return sysid::gyro::kPigeon;
  }

  if (name == "Pigeon2") {
    return sysid::gyro::kPigeon2;
  }

  if (name == "ADIS16470") {
    return sysid::gyro::kADIS16470;
  }

  if (name == "ADIS16448") {
    return sysid::gyro::kADIS16448;
  }

  if (name == "Romi") {
    return sysid::gyro::kRomiGyro;
  }

  if (name == "None") {
    return sysid::gyro::kNoGyroOption;
  }
  throw std::runtime_error(fmt::format("Unsupported Gyro Name: {}", name));
}
