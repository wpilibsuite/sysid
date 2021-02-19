// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generation/ConfigManager.h"

#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>

#include <wpi/Logger.h>
#include <wpi/SmallString.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_ostream.h>

#include "sysid/Util.h"

using namespace sysid;

ConfigManager::ConfigManager(ConfigSettings& config, wpi::Logger& logger)
    : m_config(config), m_logger(logger) {}

template <typename T>
std::vector<T> ConfigManager::SliceVector(wpi::SmallVector<T, 3> m_data,
                                          size_t size) {
  return std::vector<T>(m_data.begin(), m_data.begin() + size);
}

void ConfigManager::SaveGradle(wpi::StringRef gradlePath,
                               wpi::StringRef outputPath) {
  std::ifstream input_file{gradlePath};
  std::ofstream output_file{outputPath};

  if (input_file && output_file) {
    std::string line;
    while (std::getline(input_file, line)) {
      output_file << line << std::endl;
    }
  } else {
    if (!input_file && !output_file) {
      throw std::runtime_error("Failed to open both files\n");
    } else if (!input_file) {
      throw std::runtime_error("Failed to open input file\n");
    } else {
      throw std::runtime_error("Failed to open output files\n");
    }
  }

  input_file.close();
  output_file.close();
}

void ConfigManager::SaveJSON(wpi::StringRef path, size_t portCount,
                             bool isRomi) {
  wpi::json json;
  json["primary motor ports"] =
      SliceVector(m_config.m_primaryMotorPorts, portCount);
  json["secondary motor ports"] =
      SliceVector(m_config.m_secondaryMotorPorts, portCount);

  json["motor controllers"] =
      SliceVector(m_config.m_motorControllers, portCount);

  json["primary motors inverted"] =
      SliceVector(m_config.m_primaryMotorsInverted, portCount);
  json["secondary motors inverted"] =
      SliceVector(m_config.m_secondaryMotorsInverted, portCount);

  json["encoder type"] = m_config.m_encoderType;

  json["primary encoder ports"] = m_config.m_primaryEncoderPorts;

  json["secondary encoder ports"] = m_config.m_secondaryEncoderPorts;

  json["primary encoder inverted"] = m_config.m_primaryEncoderInverted;

  json["secondary encoder inverted"] = m_config.m_secondaryEncoderInverted;

  json["counts per shaft revolution"] = m_config.m_cpr * m_config.m_gearing;

  json["gyro"] = m_config.m_gyro;

  json["gyro ctor"] = m_config.m_gyroCtor;

  json["encoding"] = m_config.m_encoding;

  json["number of samples per average"] = m_config.m_numSamples;

  json["velocity measurement period"] = m_config.m_period;

  wpi::SmallString<128> jsonDirectory;
  wpi::raw_svector_ostream jsonOs{jsonDirectory};
  jsonOs << path << "src" << SYSID_PATH_SEPARATOR << "main"
         << SYSID_PATH_SEPARATOR << "deploy" << SYSID_PATH_SEPARATOR
         << "config.json";

  std::error_code ec;
  wpi::raw_fd_ostream os{jsonDirectory, ec};

  WPI_INFO(m_logger, "Writing JSON to: " << jsonDirectory.c_str());

  if (ec) {
    throw std::runtime_error("Cannot write to file");
  }

  os << json;
  os.flush();

  wpi::SmallString<128> gradleInPath;
  wpi::raw_svector_ostream gradleIn{gradleInPath};
  gradleIn << PROJECT_ROOT_DIR << SYSID_PATH_SEPARATOR << m_gradlePath
           << (isRomi ? "romi.txt" : "general.txt");

  wpi::SmallString<128> gradleOutPath;
  wpi::raw_svector_ostream gradleOut{gradleOutPath};

  gradleOut << path << "build.gradle";
  SaveGradle(gradleInPath, gradleOutPath);

  WPI_INFO(m_logger, "Updated build.gradle at: " << gradleOutPath.c_str());
}
