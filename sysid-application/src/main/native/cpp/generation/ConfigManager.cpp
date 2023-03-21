// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generation/ConfigManager.h"

#include <cstddef>
#include <stdexcept>
#include <string_view>

#include <fmt/format.h>
#include <wpi/Logger.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "sysid/Util.h"

using namespace sysid;

ConfigManager::ConfigManager(ConfigSettings& config, wpi::Logger& logger)
    : m_config{config}, m_logger{logger} {}

wpi::json ConfigManager::Generate(size_t occupied) {
  // Create the JSON to return.
  wpi::json json;

  // Keep a CANivore name vector around that we can push our names into
  std::vector<std::string> canivoreNames;

  // Add motor ports.
  json["primary motor ports"] =
      SliceVector(m_config.primaryMotorPorts, occupied);
  json["secondary motor ports"] =
      SliceVector(m_config.secondaryMotorPorts, occupied);

  // Add motor types.
  std::vector<std::string_view> motorControllers;
  for (size_t i = 0; i < occupied; i++) {
    motorControllers.push_back(m_config.motorControllers[i].name);
  }
  json["motor controllers"] = motorControllers;

  // Add CANivore busses
  canivoreNames.clear();
  for (size_t i = 0; i < occupied; i++) {
    canivoreNames.push_back(std::string(m_config.canivoreNames[i].data()));
  }
  json["canivore names"] = canivoreNames;

  // Add motor inversions.
  json["primary motors inverted"] =
      SliceVector(m_config.primaryMotorsInverted, occupied);
  json["secondary motors inverted"] =
      SliceVector(m_config.secondaryMotorsInverted, occupied);

  // Add encoder ports.
  json["primary encoder ports"] = m_config.primaryEncoderPorts;
  json["secondary encoder ports"] = m_config.secondaryEncoderPorts;
  json["encoder canivore name"] =
      std::string(m_config.encoderCANivoreName.data());

  // Add encoder type.
  json["encoder type"] = m_config.encoderType.name;

  // Add encoder inversions.
  json["primary encoder inverted"] = m_config.primaryEncoderInverted;
  json["secondary encoder inverted"] = m_config.secondaryEncoderInverted;

  // Add encoder units -> real world units conversion constant.
  json["counts per rotation"] = m_config.cpr;
  json["gearing numerator"] = m_config.gearingNumerator;
  json["gearing denominator"] = m_config.gearingDenominator;

  // Add gyro type and constructor.
  json["gyro"] = m_config.gyro.name;
  json["gyro ctor"] = m_config.gyroCtor;
  json["gyro canivore name"] = std::string(m_config.gyroCANivoreName.data());

  // Add advanced encoder settings.
  json["encoding"] = m_config.encoding;
  json["number of samples per average"] = m_config.numSamples;
  json["velocity measurement period"] = m_config.period;

  json["is drivetrain"] = m_config.isDrive;

  // Return JSON.
  return json;
}

void ConfigManager::ReadJSON(std::string_view path) {
  constexpr const char* json_keys[] = {"primary motor ports",
                                       "secondary motor ports",
                                       "motor controllers",
                                       "canivore names",
                                       "primary motors inverted",
                                       "secondary motors inverted",
                                       "primary encoder ports",
                                       "secondary encoder ports",
                                       "encoder canivore name",
                                       "encoder type",
                                       "primary encoder inverted",
                                       "secondary encoder inverted",
                                       "counts per rotation",
                                       "gearing numerator",
                                       "gearing denominator",
                                       "gyro",
                                       "gyro ctor",
                                       "gyro canivore name",
                                       "encoding",
                                       "number of samples per average",
                                       "velocity measurement period",
                                       "is drivetrain"};

  // Read JSON from the specified path.
  std::error_code ec;
  wpi::raw_fd_istream is{path, ec};

  if (ec) {
    throw std::runtime_error(fmt::format("Unable to read: {}", path));
  }

  wpi::json json_file;
  is >> json_file;
  WPI_INFO(m_logger, "Read {}", path);

  for (auto&& key : json_keys) {
    if (json_file.find(std::string_view(key)) == json_file.end()) {
      throw std::runtime_error(fmt::format(
          "{} was not present in config file. Please check its formatting.",
          key));
    }
  }

  m_config.primaryMotorPorts =
      json_file.at("primary motor ports").get<wpi::SmallVector<int, 3>>();
  m_config.secondaryMotorPorts =
      json_file.at("secondary motor ports").get<wpi::SmallVector<int, 3>>();

  // Parse motor controllers
  auto motorControllers =
      json_file.at("motor controllers").get<wpi::SmallVector<std::string, 3>>();
  m_config.motorControllers.clear();
  for (auto&& controller : motorControllers) {
    m_config.motorControllers.push_back(
        sysid::motorcontroller::FromMotorControllerName(controller));
  }

  m_config.primaryMotorsInverted =
      json_file.at("primary motors inverted").get<wpi::SmallVector<bool, 3>>();
  m_config.secondaryMotorsInverted = json_file.at("secondary motors inverted")
                                         .get<wpi::SmallVector<bool, 3>>();

  m_config.encoderType = sysid::encoder::FromEncoderName(
      json_file.at("encoder type").get<std::string>());
  m_config.primaryEncoderPorts =
      json_file.at("primary encoder ports").get<std::array<int, 2>>();
  m_config.secondaryEncoderPorts =
      json_file.at("secondary encoder ports").get<std::array<int, 2>>();

  m_config.primaryEncoderInverted =
      json_file.at("primary encoder inverted").get<bool>();
  m_config.secondaryEncoderInverted =
      json_file.at("secondary encoder inverted").get<bool>();

  m_config.cpr = json_file.at("counts per rotation").get<double>();
  m_config.gearingNumerator = json_file.at("gearing numerator").get<double>();
  m_config.gearingDenominator =
      json_file.at("gearing denominator").get<double>();

  m_config.gyro =
      sysid::gyro::FromGyroName(json_file.at("gyro").get<std::string>());
  m_config.gyroCtor = json_file.at("gyro ctor").get<std::string>();

  m_config.encoding = json_file.at("encoding").get<bool>();
  m_config.numSamples =
      json_file.at("number of samples per average").get<int>();
  m_config.period = json_file.at("velocity measurement period").get<int>();

  m_config.isDrive = json_file.at("is drivetrain").get<bool>();
}

void ConfigManager::SaveJSON(std::string_view path, size_t occupied) {
  auto config_json = Generate(occupied);
  sysid::SaveFile(config_json.dump(2), std::filesystem::path{path});
  WPI_INFO(m_logger, "Wrote Config JSON to: {}", path);
}

template <typename T>
std::vector<T> ConfigManager::SliceVector(wpi::SmallVector<T, 3> m_data,
                                          size_t size) {
  return std::vector<T>(m_data.begin(), m_data.begin() + size);
}
