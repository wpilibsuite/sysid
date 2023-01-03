// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <ctre/Phoenix.h>
#include <frc/ADIS16448_IMU.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/Encoder.h>
#include <frc/interfaces/Gyro.h>
#include <frc/motorcontrol/MotorController.h>
#include <rev/CANSparkMax.h>
#include <units/voltage.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>

namespace sysid {

wpi::json GetConfigJson();

/**
 * Instantiates and adds a motor controller to a vector containing speed
 * controller pointers.
 *
 * @param[in] port The port number that the motor controller is plugged into.
 * @param[in] controller The type of motor controller, should be one of these:
 *                       "PWM", "TalonSRX", "VictorSPX", "TalonFX",
 *                       "SPARK MAX (Brushless)", "SPARK AX (Brushed)", "Venom"
 * @param[in] inverted True if the motor controller should be inverted, false if
 *                     not.
 * @param[in, out] controllers A reference to the vector storing the motor
 *                             controller objects
 */
void AddMotorController(
    int port, std::string_view controller, bool inverted,
    std::vector<std::unique_ptr<frc::MotorController>>* controllers);

/**
 * Sets all the motor controllers stored to a certain voltage
 *
 * @param[in] motorVoltage The voltage the motors should be set to.
 * @param[in] controllers The vector that the motor controllers are being stored
 *                        in.
 */
void SetMotorControllers(
    units::volt_t motorVoltage,
    const std::vector<std::unique_ptr<frc::MotorController>>& controllers);

/**
 * Sets up an encoder for data collection by settings a position and rate
 * function to report the right encoder values.
 *
 * @param[in] encoderType The type of encoder used, should be one of these:
 *                        "Built-in", "Tachometer", "CANCoder",
 *                        "roboRIO quadrature", "Encoder Port", "Data Port"
 * @param[in] isEncoding True if the encoder should be in an encoding setting
 *                       (only applies to Encoders plugged into a roboRIO)
 * @param[in] period The measurement period used to calculate velocity of the
 *                   encoder (doesn't apply to roboRIO encoders)
 * @param[in] cpr The counts per revolution of the encoder
 * @param[in] gearing The gear ratio from the encoder shaft to the wheel shaft.
 * @param[in] numSamples The number of samples that should be taken for a
 *                       velocity measurement
 * @param[in] controllerName The main motor controller being used in the setup.
 * @param[in] controller A pointer to the main motor controller object being
 *                       used in the setup.
 * @param[in] encoderInverted True if the encoder is supposed to be inverted
 *                            (doesn't apply to NEO Integrated Encoders)
 * @param[in] encoderPorts Port number for the encoder if its not plugged into a
 *                         motor controller. 2 ports should be used for roboRIO
 *                         encoders, 1 port should be used for CANCoder.
 * @param[in, out] cancoder A reference to a CANCoder object
 * @param[in, out] revEncoderPort A reference to a REV Encoder Port object
 * @param[in, out] revDataPort A reference to a REV Data Port object
 * @param[in, out] encoder A reference to a roboRIO encoder object
 * @param[out] position A reference to a function that is supposed to return the
 *                      encoder position
 * @param[out] rate A reference to a function that is supposed to return the
 *                  encoder rate
 */
void SetupEncoders(
    std::string_view encoderType, bool isEncoding, int period, double cpr,
    double gearing, int numSamples, std::string_view controllerName,
    frc::MotorController* controller, bool encoderInverted,
    const std::vector<int>& encoderPorts, std::unique_ptr<CANCoder>& cancoder,
    std::unique_ptr<rev::SparkMaxRelativeEncoder>& revEncoderPort,
    std::unique_ptr<rev::SparkMaxAlternateEncoder>& revDataPort,
    std::unique_ptr<frc::Encoder>& encoder, std::function<double()>& position,
    std::function<double()>& rate);

/**
 * Sets up an encoder for data collection by settings a position and rate
 * function to report the right encoder values.
 *
 * @param[in] gyroType The type of gyro to configure. Options include: "Analog
 *                     Gyro", "ADXRS450", "NavX", "Pigeon", "Romi", "None"
 * @param[in] gyroCtor The type of constructor to use with the gyro. This varies
 *                     based on the gyro.
 * @param[in] leftPorts All of the specified ports on the left side of the
 *                      drivetrain. Intended to be used when there's a Pigeon
 *                      IMU plugged into a drive motor controller.
 * @param[in] rightPorts All of the specified ports on the right side of the
 *                       drivetrain. Intended to be used when there's a Pigeon
 *                       IMU plugged into a drive motor controller.
 * @param[in] controllerNames The motor controllers being used in the setup.
 * @param[in] leftControllers A vector of stored motor controller objects for
 *                            the left side of the drivetrain. Intended to be
 *                            used when there's a Pigeon IMU plugged into a
 *                            drive motor controller.
 * @param[in] rightControllers A vector of stored motor controller objects for
 *                             the right side of the drivetrain. Intended to be
 *                             used when there's a Pigeon IMU plugged into a
 *                             drive motor controller.
 * @param[in, out] gyro A pointer to a WPILib Gyro object.
 * @param[in, out] ADIS16448Gyro A pointer to an ADIS16448_IMU object.
 * @param[in, out] ADIS16470Gyro A pointer to an ADIS16470_IMU object.
 * @param[in, out] pigeon A pointer to a Pigeon IMU Object
 * @param[in, out] tempTalon A pointer to a TalonSRX object mean to store a
 *                           Talon that the Pigeon IMU is plugged into.
 * @param[out] gyroPosition A reference to a function that is supposed to return
 *                          the gyro position
 * @param[out] gyroRate A reference to a function that is supposed to return the
 *                      gyro rate
 */
void SetupGyro(
    std::string_view gyroType, std::string_view gyroCtor,
    const std::vector<int>& leftPorts, const std::vector<int>& rightPorts,
    const std::vector<std::string>& controllerNames,
    const std::vector<std::unique_ptr<frc::MotorController>>& leftControllers,
    const std::vector<std::unique_ptr<frc::MotorController>>& rightControllers,
    std::unique_ptr<frc::Gyro>& gyro,
    std::unique_ptr<frc::ADIS16448_IMU>& ADIS16448Gyro,
    std::unique_ptr<frc::ADIS16470_IMU>& ADIS16470Gyro,
    std::unique_ptr<BasePigeon>& pigeon,
    std::unique_ptr<WPI_TalonSRX>& tempTalon,
    std::function<double()>& gyroPosition, std::function<double()>& gyroRate);

/**
 * Sets specified data collection functions to return zero. This is to avoid
 * runtime crashes for when the functions aren't defined.
 *
 * @param[out] position The position data collection function
 * @param[out] rate The rate data collection function
 */
void SetDefaultDataCollection(std::function<double()>& position,
                              std::function<double()>& rate);
}  // namespace sysid
