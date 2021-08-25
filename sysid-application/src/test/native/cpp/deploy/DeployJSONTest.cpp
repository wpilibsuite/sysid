// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <wpi/Logger.h>
#include <wpi/json.h>

#include "gtest/gtest.h"
#include "sysid/deploy/DeploySession.h"
#include "sysid/generation/ConfigManager.h"

TEST(DeployTest, JSONGeneration) {
  wpi::Logger m_logger;
  sysid::ConfigSettings m_config;
  sysid::ConfigManager m_configManager{m_config, m_logger};
  sysid::DeploySession m_session{"0", false, m_configManager.Generate(1),
                                 m_logger};

  const auto& json = m_session.GetJSON();

  // Verify that it is valid JSON data
  try {
    wpi::json::parse(json.dump());
  } catch (wpi::json::parse_error err) {
    FAIL();
  }

  // Make sure its an object
  ASSERT_TRUE(json.is_object());
}
