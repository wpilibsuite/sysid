// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntegrationUtils.h"

#include <stdexcept>
#include <string>

#include <fmt/core.h>
#include <wpi/timestamp.h>

#include "gtest/gtest.h"

void LaunchSim(std::string_view projectDirectory) {
  // Install the robot program.
  std::string installCmd =
      fmt::format("cd {}/ && {} :{}:installSimulateNativeRelease -Pintegration",
                  EXPAND_STRINGIZE(PROJECT_ROOT_DIR), LAUNCH, projectDirectory);
  fmt::print(stderr, "Executing: {}\n", installCmd);

  int result = std::system(installCmd.c_str());

  // Exit the test if we could not install the robot program.
  if (result != 0) {
    fmt::print(stderr, "The robot program could not be installed.\n");
    std::exit(1);
  }

  // Run the robot program.
  std::string runCmd =
      fmt::format("cd {}/ && {} :{}:simulateNativeRelease -Pintegration {}",
                  EXPAND_STRINGIZE(PROJECT_ROOT_DIR), LAUNCH_DETACHED,
                  projectDirectory, DETACHED_SUFFIX);
  fmt::print(stderr, "Executing: {}\n", runCmd);

  // Start capturing console output before gradle command to capture program
  // data
  ::testing::internal::CaptureStdout();

  result = std::system(runCmd.c_str());

  // Exit the test if we could not run the robot program.
  if (result != 0) {
    fmt::print(stderr, "The robot program could not be started.\n");
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
      auto capturedStdout = ::testing::internal::GetCapturedStdout();
      fmt::print(stderr,
                 "\n******\nRobot Program Captured Output:\n{}\n******\n",
                 capturedStdout);
      std::exit(1);
    }
  }
}

std::string KillNT(NT_Inst nt, NT_Entry kill) {
  // Before killing sim, store any captured console output.
  auto capturedStdout = ::testing::internal::GetCapturedStdout();

  fmt::print(stderr, "Killing program\n");
  auto time = wpi::Now();

  while (nt::IsConnected(nt)) {
    // Kill program
    nt::SetEntryValue(kill, nt::Value::MakeBoolean(true));
    nt::Flush(nt);
    if (wpi::Now() - time > 3E7) {
      EXPECT_TRUE(false);
      return capturedStdout;
    }
  }

  fmt::print(stderr, "Killed robot program\n");

  // Set kill entry to false for future tests
  nt::SetEntryValue(kill, nt::Value::MakeBoolean(false));
  // Stop NT Client.
  nt::StopClient(nt);

  return capturedStdout;
}
