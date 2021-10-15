// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntegrationUtils.h"

#include <string>

#include <fmt/core.h>
#include <wpi/timestamp.h>

#include "gtest/gtest.h"

void LaunchSim(std::string_view projectDirectory) {
  // Start the robot program.
  std::string cmd =
      fmt::format("cd {}/ && {} :{}:simulateCpp -Pintegration",
                  EXPAND_STRINGIZE(PROJECT_ROOT_DIR), LAUNCH, projectDirectory);

  fmt::print(stderr, "Executing: {}\n", cmd);

  int result = std::system(cmd.c_str());

  // Exit Test if Sim is unable to start
  if (result != 0) {
    fmt::print(stderr, "The robot program could not be started\n");
    std::exit(1);
  }
}

void Connect(NT_Inst nt, NT_Entry kill) {
  nt::StartClient(nt, "localhost", NT_DEFAULT_PORT);

  nt::SetEntryValue(kill, nt::Value::MakeBoolean(false));
  nt::Flush(nt);

  // Wait for NT to connect or fail it if it times out.
  auto time = wpi::Now();
  while (!nt::IsConnected(nt)) {
    if (wpi::Now() - time > 1.5E7) {
      fmt::print(stderr, "The robot program crashed\n");
      std::exit(1);
    }
  }
}

void KillNT(NT_Inst nt, NT_Entry kill) {
  fmt::print(stderr, "Killing program\n");
  auto time = wpi::Now();

  while (nt::IsConnected(nt)) {
    // Kill program
    nt::SetEntryValue(kill, nt::Value::MakeBoolean(true));
    nt::Flush(nt);
    if (wpi::Now() - time > 3E7) {
      FAIL();
      break;
    }
  }

  fmt::print(stderr, "Killed robot program\n");

  // Set kill entry to false for future tests
  nt::SetEntryValue(kill, nt::Value::MakeBoolean(false));
  // Stop NT Client.
  nt::StopClient(nt);
}
