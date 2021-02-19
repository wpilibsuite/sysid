// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <wpi/Logger.h>
#include <wpi/SmallVector.h>

namespace sysid {

/**
 * This represents the settings for configuring a robot project.
 * This includes hardware information such as motor ports, controllers, encoder
 * ports, inversions, etc.. It also includes numerical data such as EPR.
 */
struct ConfigSettings {
  wpi::SmallVector<int, 3> m_primaryMotorPorts = {0, 1};
  wpi::SmallVector<int, 3> m_secondaryMotorPorts = {2, 3};
  wpi::SmallVector<std::string, 3> m_motorControllers = {"PWM", "PWM"};
  wpi::SmallVector<bool, 3> m_primaryMotorsInverted = {false, false};
  wpi::SmallVector<bool, 3> m_secondaryMotorsInverted = {false, false};
  std::string m_encoderType = "roboRIO";
  std::array<int, 2> m_primaryEncoderPorts = {0, 1};
  std::array<int, 2> m_secondaryEncoderPorts = {2, 3};
  bool m_primaryEncoderInverted = false;
  bool m_secondaryEncoderInverted = false;
  double m_cpr = 1.0;
  double m_gearing = 1.0;
  std::string m_gyro = "AnalogGyro";
  std::string m_gyroCtor = "0";
  bool m_encoding = false;
  int m_numSamples = 1;
  int m_period = 5;
};

const ConfigSettings kRomiConfig{{0},       {1},    {"PWM"}, {true}, {false},
                                 "roboRIO", {4, 5}, {6, 7},  false,  false,
                                 1440.0,    1.0,    "Romi",  ""};

class ConfigManager {
 public:
  /**
   * Constructs an instance of the ConfigManager that can generate config
   * settings to save to a robot project.
   * @param settings The settings for this instance of a ConfigManager.
   */
  explicit ConfigManager(ConfigSettings& settings, wpi::Logger& logger);

  /**
   * Saves a config json to the specified path.
   * @param path The path that the json will be saved to
   * @param portCount How many ports the robot will be using. If it is a
   * drivetrain project, this is how many ports per drive side.
   */
  void SaveJSON(wpi::StringRef path, size_t portCount, bool isRomi = false);

 private:
  ConfigSettings& m_config;
  static constexpr const char* m_gradlePath = "src/main/native/build_files/";

  /**
   * Returns a slice of a vector starting at its start element and ending at the
   * specified size.
   * @param m_data The vector to be sliced.
   * @param size The desired size of the vector.
   * @return A version of `m_data` that is `size` long.
   */
  template <class T>
  std::vector<T> SliceVector(wpi::SmallVector<T, 3> m_data, size_t size);

  void SaveGradle(wpi::StringRef gradlePath, wpi::StringRef outputPath);

  wpi::Logger m_logger;
};

}  // namespace sysid
