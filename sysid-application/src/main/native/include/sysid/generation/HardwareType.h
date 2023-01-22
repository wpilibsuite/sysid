// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <string_view>

namespace sysid {
/**
 * Stores information of a specific hardware device that sysid supports so that
 * it can be easily identified, used, and displayed.
 */
struct HardwareType {
  /**
   * The name of the hardware device to facillitate comparisons
   */
  std::string_view name;

  /**
   * The display name of the hardware device to facilitate GUI display
   */
  const char* displayName;

  /**
   * Creates a HardwareType object from a device name.
   *
   * @param deviceName The name of the hardware device.
   */
  constexpr explicit HardwareType(std::string_view deviceName)
      : name{deviceName}, displayName{name.data()} {}

  /**
   * Equality operator between HardwareTypes
   *
   * @param rhs Another HardwareType
   * @returns True if they are equal
   */
  constexpr bool operator==(const HardwareType& rhs) const {
    return name == rhs.name;
  }

  /**
   * Inequality operator between HardwareTypes
   *
   * @param rhs Another HardwareType
   * @returns True if they are not equal
   */
  constexpr bool operator!=(const HardwareType& rhs) const {
    return !operator==(rhs);
  }
};

namespace motorcontroller {

constexpr HardwareType kPWM{"PWM"};
constexpr HardwareType kTalonSRX{"TalonSRX"};
constexpr HardwareType kVictorSPX{"VictorSPX"};
constexpr HardwareType kTalonFX{"TalonFX"};
constexpr HardwareType kTalonFXPro{"TalonFX (Pro)"};
constexpr HardwareType kSPARKMAXBrushless{"SPARK MAX (Brushless)"};
constexpr HardwareType kSPARKMAXBrushed{"SPARK MAX (Brushed)"};
constexpr HardwareType kVenom{"Venom"};

constexpr std::array<HardwareType, 8> kMotorControllers = {
    kPWM,        kTalonSRX,          kVictorSPX,       kTalonFX,
    kTalonFXPro, kSPARKMAXBrushless, kSPARKMAXBrushed, kVenom};

/**
 * Returns an existing motor controller HardwareType from a string_view. Throws
 * if the passed name doesn't exist.
 *
 * @param name The name of the motor controller.
 * @return The motor controller HardwareType associated with the inputted name.
 */
HardwareType FromMotorControllerName(std::string_view name);
}  // namespace motorcontroller

namespace encoder {
constexpr HardwareType kRoboRIO{"roboRIO quadrature"};
constexpr HardwareType kCANCoder{"CANCoder"};
constexpr HardwareType kCANcoderPro{"CANcoder (Pro)"};
constexpr HardwareType kBuiltInSetting{"Built-in"};
constexpr HardwareType kCTRETachometer{"Tachometer"};
constexpr HardwareType kSMaxEncoderPort{"Encoder Port"};
constexpr HardwareType kSMaxDataPort{"Data Port"};

constexpr std::array<HardwareType, 7> kEncoders = {
    kRoboRIO,        kCANCoder,        kCANcoderPro, kBuiltInSetting,
    kCTRETachometer, kSMaxEncoderPort, kSMaxDataPort};

/**
 * Returns an existing encoder HardwareType from a string_view. Throws if the
 * passed name doesn't exist.
 *
 * @param name The name of the encoder.
 * @return The encoder HardwareType associated with the inputted name.
 */
HardwareType FromEncoderName(std::string_view name);
}  // namespace encoder

namespace gyro {
constexpr HardwareType kAnalogGyro{"Analog Gyro"};
constexpr HardwareType kADXRS450{"ADXRS450"};
constexpr HardwareType kADIS16448{"ADIS16448"};
constexpr HardwareType kADIS16470{"ADIS16470"};
constexpr HardwareType kNavX{"NavX"};
constexpr HardwareType kPigeon{"Pigeon"};
constexpr HardwareType kPigeon2{"Pigeon2"};
constexpr HardwareType kPigeon2Pro{"Pigeon2 (Pro)"};
constexpr HardwareType kRomiGyro{"Romi"};
constexpr HardwareType kNoGyroOption{"None"};

constexpr std::array<HardwareType, 9> kGyros = {
    kAnalogGyro, kADXRS450,  kNavX,     kPigeon,      kPigeon2,
    kADIS16448,  kADIS16470, kRomiGyro, kNoGyroOption};

/**
 * Returns an existing gyro HardwareType from a string_view. Throws if the
 * passed name doesn't exist.
 *
 * @param name The name of the gyro.
 * @return The gyro HardwareType associated with the inputted name.
 */
HardwareType FromGyroName(std::string_view name);
}  // namespace gyro
}  // namespace sysid
