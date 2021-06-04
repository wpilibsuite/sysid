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

namespace sysid {
/**
 * This represents the settings for configuring a robot project -- including
 * hardware information such as motor ports, controllers, encoder ports,
 * inversions, etc. It also includes numerical data such as EPR.
 */
struct ConfigSettings {
  // Ports for motor controllers.
  wpi::SmallVector<int, 3> primaryMotorPorts = {0, 1};
  wpi::SmallVector<int, 3> secondaryMotorPorts = {2, 3};

  // Type of motor controller.
  wpi::SmallVector<std::string, 3> motorControllers = {"PWM", "PWM"};

  // Whether motors should be inverted.
  wpi::SmallVector<bool, 3> primaryMotorsInverted = {false, false};
  wpi::SmallVector<bool, 3> secondaryMotorsInverted = {false, false};

  // Encoder type and ports.
  std::string encoderType = "roboRIO";
  std::array<int, 2> primaryEncoderPorts = {0, 1};
  std::array<int, 2> secondaryEncoderPorts = {2, 3};

  // Whether encoders should be inverted.
  bool primaryEncoderInverted = false;
  bool secondaryEncoderInverted = false;

  // Constants for converting between encoder units and real-world units.
  double cpr = 1.0;
  double gearing = 1.0;

  // Gyro type and constructor.
  std::string gyro = "AnalogGyro";
  std::string gyroCtor = "0";

  // Advanced encoder settings.
  bool encoding = false;
  int numSamples = 1;
  int period = 5;
};

// Pre-built configuration for the Romi -- all Romis have the same setup.
const ConfigSettings kRomiConfig{{0},       {1},    {"PWM"}, {true}, {false},
                                 "roboRIO", {4, 5}, {6, 7},  false,  false,
                                 1440.0,    1.0,    "Romi",  ""};

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
   *
   * @return The generated JSON.
   */
  wpi::json Generate(size_t occupied);

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
   *
   * @return A version of `m_data` that is `size` long.
   */
  template <class T>
  static std::vector<T> SliceVector(wpi::SmallVector<T, 3> m_data, size_t size);
};
}  // namespace sysid
