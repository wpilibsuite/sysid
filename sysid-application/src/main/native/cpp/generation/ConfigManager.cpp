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
    : m_config{config}, m_logger{logger} {}

wpi::json ConfigManager::Generate(size_t occupied) {
  // Create the JSON to return.
  wpi::json json;

  // Add motor ports.
  json["primary motor ports"] =
      SliceVector(m_config.primaryMotorPorts, occupied);
  json["secondary motor ports"] =
      SliceVector(m_config.secondaryMotorPorts, occupied);

  // Add motor types.
  json["motor controllers"] = SliceVector(m_config.motorControllers, occupied);

  // Add motor inversions.
  json["primary motors inverted"] =
      SliceVector(m_config.primaryMotorsInverted, occupied);
  json["secondary motors inverted"] =
      SliceVector(m_config.secondaryMotorsInverted, occupied);

  // Add encoder ports.
  json["primary encoder ports"] = m_config.primaryEncoderPorts;
  json["secondary encoder ports"] = m_config.secondaryEncoderPorts;

  // Add encoder type.
  json["encoder type"] = m_config.encoderType;

  // Add encoder inversions.
  json["primary encoder inverted"] = m_config.primaryEncoderInverted;
  json["secondary encoder inverted"] = m_config.secondaryEncoderInverted;

  // Add encoder units -> real world units conversion constant.
  json["counts per shaft revolution"] = m_config.cpr * m_config.gearing;

  // Add gyro type and constructor.
  json["gyro"] = m_config.gyro;
  json["gyro ctor"] = m_config.gyroCtor;

  // Add advanced encoder settings.
  json["encoding"] = m_config.encoding;
  json["number of samples per average"] = m_config.numSamples;
  json["velocity measurement period"] = m_config.period;

  // Return JSON.
  return json;
}

template <typename T>
std::vector<T> ConfigManager::SliceVector(wpi::SmallVector<T, 3> m_data,
                                          size_t size) {
  return std::vector<T>(m_data.begin(), m_data.begin() + size);
}
