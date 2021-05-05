// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IntegrationUtils.h"

#include <wpi/SmallString.h>
#include <wpi/SmallVector.h>
#include <wpi/raw_ostream.h>
#include <wpi/timestamp.h>

#include "gtest/gtest.h"

void LaunchSim(std::string projectDirectory) {
  // Start the robot program.
  wpi::SmallString<128> cmd;
  wpi::raw_svector_ostream os(cmd);

  os << "cd " << EXPAND_STRINGIZE(PROJECT_ROOT_DIR) << SYSID_PATH_SEPARATOR
     << projectDirectory << " && " << LAUNCHSIM << " -Pintegration";
  wpi::outs() << "Executing: " << cmd.c_str() << "\n";
  wpi::outs().flush();

  int result = std::system(cmd.c_str());

  // Exit Test if Sim is unable to start
  if (result != 0) {
    wpi::outs() << "The robot program could not be started\n";
    wpi::outs().flush();
    std::exit(1);
  }
}

void Connect(NT_Inst nt, NT_Entry kill) {
  nt::StartClient(nt, "localhost", NT_DEFAULT_PORT);

  nt::SetEntryValue(kill, nt::Value::MakeBoolean(false));
  nt::Flush(nt);

  // Wait for NT to connect or until it times out.
  auto time = wpi::Now();
  while (!nt::IsConnected(nt)) {
    ASSERT_LT(wpi::Now() - time, 1.5E7);
  }
}

void KillNT(NT_Inst nt, NT_Entry kill) {
  wpi::outs() << "Killing program\n";
  wpi::outs().flush();
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

  wpi::outs() << "Killed robot program"
              << "\n";
  wpi::outs().flush();

  // Set kill entry to false for future tests
  nt::SetEntryValue(kill, nt::Value::MakeBoolean(false));
  // Stop NT Client.
  nt::StopClient(nt);
}
