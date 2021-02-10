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

 private:
  void SelectDataFolder();
  void CheckNTReset();

  double m_quasistatic = 0.25;
  double m_step = 7.0;

  std::unique_ptr<TelemetryManager> m_manager =
      std::make_unique<TelemetryManager>(
          TelemetryManager::Settings{&m_quasistatic, &m_step});

  std::unique_ptr<pfd::select_folder> m_selector;
  std::string m_jsonLocation;

  bool m_ntConnected = false;
  bool m_ntReset = true;

  int* m_team = nullptr;

  std::string m_opened;
  std::string m_exception;
};
}  // namespace sysid
