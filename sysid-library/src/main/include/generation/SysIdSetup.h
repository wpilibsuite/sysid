// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include <ctre/Phoenix.h>

#include <functional>
#include <memory>
#include <string_view>
#include <vector>

#include <frc/Encoder.h>
#include <frc/motorcontrol/MotorController.h>
#include <units/voltage.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>

wpi::json GetConfigJson();

/**
 * Instantiates and adds a motor controller to a vector containing speed
 * controller pointers.
 *
 * @param port The port number that the motor controller is plugged into.
 * @param controller The type of motor controller, should be one of these:
 *                   "PWM", "TalonSRX", "VictorSPX", "TalonFX",
 *                   "SPARK MAX (Brushless)", "SPARK AX (Brushed)", "Venom"
 * @param inverted True if the motor controller should be inverted, false if
 *                 not.
 * @param controllers A reference to the vector storing the motor controller
 *                    objects
 */
void AddMotorController(
    int port, std::string_view controller, bool inverted,
    std::vector<std::unique_ptr<frc::MotorController>>* controllers);

/**
 * Sets all the motor controllers stored to a certain voltage
 *
 * @param motorVoltage The voltage the motors should be set to.
 * @param controllers The vector that the motor controllers are being stored in.
 */
void SetMotorControllers(
    units::volt_t motorVoltage,
    const std::vector<std::unique_ptr<frc::MotorController>>& controllers);

/**
 * Sets up an encoder for data collection by settings a position and rate
 * function to report the right encoder values.
 *
 * @param encoderType The type of encoder used, should be one of these:
 *                    "Built-in", "Tachometer", "CANCoder",
 *                    "roboRIO quadrature", "Encoder Port", "Data Port"
 * @param isEncoding True if the encoder should be in an encoding setting (only
 *                   applies to Encoders plugged into a roboRIO)
 * @param period The measurement period used to calculate velocity of the
 *               encoder (doesn't apply to roboRIO encoders)
 * @param cpr The counts per revolution of the encoder
 * @param numSamples The number of samples that should be taken for a velocity
 *                   measurement
 * @param controllerName The main motor controller being used in the setup.
 * @param controller A pointer to the main motor controller object being used in
 *                   the setup.
 * @param encoderInverted True if the encoder is supposed to be inverted
 *                        (doesn't apply to NEO Integrated Encoders)
 * @param encoderPorts Port number for the encoder if its not plugged into a
 *                     motor controller. 2 ports should be used for roboRIO
 *                     encoders, 1 port should be used for CANCoder.
 * @param cancoder A reference to a CANCoder object
 * @param encoder A reference to a roboRIO encoder object
 * @param position A reference to a function that is supposed to return the
 *                 encoder position
 * @param rate A reference to a function that is supposed to return the encoder
 *             rate
 */
void SetupEncoders(std::string_view encoderType, bool isEncoding, int period,
                   double cpr, int numSamples, std::string_view controllerName,
                   frc::MotorController* controller, bool encoderInverted,
                   const std::vector<int>& encoderPorts,
                   // std::unique_ptr<CANCoder>& cancoder,
                   std::unique_ptr<frc::Encoder>& encoder,
                   std::function<double()>& position,
                   std::function<double()>& rate);
