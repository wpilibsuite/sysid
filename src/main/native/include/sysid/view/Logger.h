// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string>

#include <glass/DataSource.h>
#include <glass/View.h>
#include <portable-file-dialogs.h>

#include "sysid/telemetry/TelemetryManager.h"

namespace sysid {
/**
 * The logger GUI takes care of running the system idenfitication tests over
 * NetworkTables and logging the data. This data is then stored in a JSON file
 * which can be used for analysis.
 */
class Logger : public glass::View {
 public:
  Logger();
  void Display() override;

  static constexpr const char* kTypes[] = {"Drivetrain", "Arm", "Elevator",
                                           "Simple"};

 private:
  void SelectDataFolder();
  void CheckNTReset();

  TelemetryManager::Settings m_settings;
  int m_selectedType = 0;

  std::unique_ptr<TelemetryManager> m_manager =
      std::make_unique<TelemetryManager>(m_settings);

  std::unique_ptr<pfd::select_folder> m_selector;
  std::string m_jsonLocation;

  bool m_ntConnected = false;
  bool m_ntReset = true;

  double m_primaryEncoder = 0.0;
  double m_secondaryEncoder = 0.0;
  double m_gyro = 0.0;

  int* m_team = nullptr;

  std::string m_opened;
  std::string m_exception;
};
}  // namespace sysid
