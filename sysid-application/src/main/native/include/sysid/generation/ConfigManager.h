// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include <wpi/Logger.h>
#include <wpi/SmallVector.h>
#include <wpi/json.h>

#include "sysid/generation/HardwareType.h"

namespace sysid {
/**
 * This represents the settings for configuring a robot project -- including
 * hardware information such as motor ports, controllers, encoder ports,
 * inversions, etc. It also includes numerical data such as EPR.
 */
struct ConfigSettings {
  /**
   * Maximum length of a CANivore name is 32 characters
   */
  static constexpr int kMaxCANivoreNameLength = 32;

  /**
   * Ports for the primary motor controllers. For general mechanisms this is the
   * motor ports that will be used. For drivetrains these are the left-side
   * motor controllers.
   */
  wpi::SmallVector<int, 3> primaryMotorPorts = {0, 1};

  /**
   * Ports for the secondary motor controllers. For general mechanisms this
   * won't be used. For drivetrains these are the right-side motor controllers.
   */
  wpi::SmallVector<int, 3> secondaryMotorPorts = {2, 3};

  /**
   * The motor controller types that should be configured
   */
  wpi::SmallVector<HardwareType, 3> motorControllers = {
      sysid::motorcontroller::kPWM, sysid::motorcontroller::kPWM};

  /**
   * The CANivore name if it's on a Non-RIO bus
   */
  wpi::SmallVector<std::array<char, kMaxCANivoreNameLength>, 3> canivoreNames =
      {{'r', 'i', 'o', '\0'}, {'r', 'i', 'o', '\0'}};

  /**
   * If the primary motor controllers (general mechanism motors / left
   * drivetrain motors) should be inverted or not.
   */
  wpi::SmallVector<bool, 3> primaryMotorsInverted = {false, false};

  /**
   * If the secondary motor controllers (right drivetrain motors) should be
   * inverted or not.
   */
  wpi::SmallVector<bool, 3> secondaryMotorsInverted = {false, false};

  /**
   * The type of encoder that's used.
   */
  HardwareType encoderType = sysid::encoder::kRoboRIO;

  /**
   * The encoder ports that are used as the primary encoder (general mechanism
   * encoder / left drivetrain encoder). If they're plugged into the roboRIO it
   * stores both ports, if its a CANCoder the port number is the first index.
   */
  std::array<int, 2> primaryEncoderPorts = {0, 1};

  /**
   * The encoder ports that are used as the secondary encoder (right drivetrain
   * encoder). If they're plugged into the roboRIO it stores both ports, if its
   * a CANCoder the port number is the first index.
   */
  std::array<int, 2> secondaryEncoderPorts = {2, 3};

  /**
   * CANivore names for the CANcoder sensors if they're not on the RIO bus.
   */
  std::array<char, kMaxCANivoreNameLength> encoderCANivoreName = {'r', 'i', 'o',
                                                                  '\0'};

  /**
   * If the primary encoder (general mechanism motors / left drivetrain encoder)
   * should be inverted or not.
   */
  bool primaryEncoderInverted = false;

  /**
   * If the secondary encoder (right drivetrain encoder) should be inverted or
   * not.
   */
  bool secondaryEncoderInverted = false;

  /**
   * If using a CANcoder and it's using Pro.
   */
  bool cancoderUsingPro = false;

  /**
   * The counts per revolution of the encoder.
   */
  double cpr = 1.0;

  /**
   * The numerator for the gear ratio.
   */
  double gearingNumerator = 1.0;

  /**
   * The denominator for the gear ratio
   */
  double gearingDenominator = 1.0;

  /**
   * The type of gyro to configure.
   */
  HardwareType gyro = sysid::gyro::kAnalogGyro;

  /**
   * The gyro constructor to use.
   */
  std::string gyroCtor = "0";

  /**
   * The CANivore name of a Pigeon2 if it's on a Non-RIO bus
   */
  std::array<char, kMaxCANivoreNameLength> gyroCANivoreName = {'r', 'i', 'o',
                                                               '\0'};

  /**
   * If the encoder is plugged into the roboRIO, there's the option to set it's
   * encoding setting to reduce noise.
   */
  bool encoding = false;

  /**
   * The number of samples that should be used to calculate velocity.
   */
  int numSamples = 1;

  /**
   * The period of time in which the velocity is calculated.
   */
  int period = 1;

  /**
   * If the configuration is for a drivetrain.
   */
  bool isDrive = false;
};

// Pre-built configuration for the Romi -- all Romis have the same setup.
const ConfigSettings kRomiConfig{{0},
                                 {1},
                                 {sysid::motorcontroller::kPWM},
                                 {{'r', 'i', 'o', '\0'}},
                                 {true},
                                 {false},
                                 sysid::encoder::kRoboRIO,
                                 {4, 5},
                                 {6, 7},
                                 {{'r', 'i', 'o', '\0'}},
                                 false,
                                 false,
                                 false,
                                 1440.0,
                                 1.0,
                                 1.0,
                                 sysid::gyro::kRomiGyro,
                                 "",
                                 {'r', 'i', 'o', '\0'}};

/**
 * This class manages generating the JSON configuration from a reference to the
 * settings struct.
 */
class ConfigManager {
 public:
  /**
   * Constructs an instance of the ConfigManager that can generate config
   * settings to save to a robot project.
   *
   * @param settings The settings for this instance of a ConfigManager.
   * @param logger   A reference to the logger to which log messages should be
   *                 sent.
   */
  ConfigManager(ConfigSettings& settings, wpi::Logger& logger);

  /**
   * Generates the JSON from the settings struct provided in the constructor.
   *
   * @param occupied The number of occupied motor ports. Due to the way the GUI
   *                 works, the number of elements in the vector is not
   *                 necessarily how many ports the user actually wants -- this
   *                 is kept track of in a separate variable.
   * @return The generated JSON.
   */
  wpi::json Generate(size_t occupied);

  /**
   * Reads in a JSON and saves it as the stored settings
   *
   * @param path The path to the JSON
   */
  void ReadJSON(std::string_view path);

  /**
   * Writes the stored settings to a JSON
   *
   * @param path The path to save the JSON to
   * @param occupied The number of motor controllers that will be used
   */
  void SaveJSON(std::string_view path, size_t occupied);

 private:
  // Configuration settings for this instance.
  ConfigSettings& m_config;

  // Reference to logger for log messages.
  wpi::Logger& m_logger;

  /**
   * Returns a slice of a vector starting at its start element and ending at the
   * specified size.
   *
   * @param m_data The vector to be sliced.
   * @param size   The desired size of the vector.
   * @return A version of `m_data` that is `size` long.
   */
  template <class T>
  static std::vector<T> SliceVector(wpi::SmallVector<T, 3> m_data, size_t size);
};
}  // namespace sysid
