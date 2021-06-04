// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>

#include <functional>
#include <memory>
#include <string_view>
#include <vector>

#include <frc/Encoder.h>
#include <frc/SpeedController.h>
#include <units/voltage.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>

wpi::json GetConfigJson();

void AddMotorController(
    int port, std::string_view controller, bool inverted,
    std::vector<std::unique_ptr<frc::SpeedController>>* controllers);

void SetMotorControllers(
    units::volt_t motorVoltage,
    const std::vector<std::unique_ptr<frc::SpeedController>>& controllers);

void SetupEncoders(std::string_view encoderType, bool isEncoding, int period,
                   double cpr, int numSamples, std::string_view controllerName,
                   frc::SpeedController* controller, bool encoderInverted,
                   const std::vector<int>& encoderPorts,
                   std::unique_ptr<CANCoder>& cancoder,
                   std::unique_ptr<frc::Encoder>& encoder,
                   std::function<double()>& position,
                   std::function<double()>& rate);
