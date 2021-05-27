// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generation/ConfigManager.h"

#include <cstddef>
#include <stdexcept>

#include <fmt/format.h>
#include <wpi/Logger.h>
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

void ConfigManager::SaveJSON(std::string_view path, size_t portCount,
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

  std::string jsonDirectory = fmt::format("{}config.json", path);

  std::error_code ec;
  wpi::raw_fd_ostream os{jsonDirectory, ec};

  WPI_INFO(m_logger, "Writing JSON to: " << jsonDirectory);

  if (ec) {
    throw std::runtime_error("Cannot write to file");
  }

  os << json;
  os.flush();
}
