// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdlib>
#include <thread>

#include <ntcore_c.h>
#include <ntcore_cpp.h>
#include <wpi/SmallString.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>
#include <wpi/raw_ostream.h>
#include <wpi/timestamp.h>

#include "gtest/gtest.h"
#include "networktables/NetworkTableValue.h"
#include "sysid/Util.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/telemetry/TelemetryManager.h"

#ifdef _WIN32
#define LAUNCHSIM "gradlew simulateCpp"
#else
#define LAUNCHSIM "./gradlew simulateCpp"
#endif

using namespace std::chrono_literals;

// The constants that are defined in our integration test program.
constexpr double Kv = 1.98;
constexpr double Ka = 0.2;
constexpr double kTrackWidth = 0.762;

// Create our test fixture class so we can reuse the same logic for various test
// mechanisms.
class IntegrationTest : public ::testing::Test {
 public:
  static constexpr const char* kTests[] = {"slow-forward", "slow-backward",
                                           "fast-forward", "fast-backward",
                                           "track-width"};

  void SetUp(sysid::AnalysisType mechanism) {
    m_settings.mechanism = mechanism;
    m_manager = std::make_unique<sysid::TelemetryManager>(m_settings, m_nt);
    // Change the default settings a little bit.
    m_settings.quasistaticRampRate = 0.75;

    // Start the robot program.
    wpi::SmallString<128> cmd;
    wpi::raw_svector_ostream os(cmd);

    os << "cd " << PROJECT_ROOT_DIR << SYSID_PATH_SEPARATOR
       << "integration_test_project"
       << " && " << LAUNCHSIM << " -Pintegration";
    wpi::outs() << "Executing: " << cmd.c_str() << "\n";
    wpi::outs().flush();

    int result = std::system(cmd.c_str());
    ASSERT_EQ(0, result) << "The robot program couldn't be started";

    nt::StartClient(m_nt, "localhost", NT_DEFAULT_PORT);

    // Wait for NT to connect or until it times out.
    auto time = wpi::Now();
    while (!nt::IsConnected(m_nt)) {
      if (wpi::Now() - time > 1.5E7) {
        ASSERT_TRUE(false) << "Was not able to connect to the robot program.";
      }
    }
    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(false));
    nt::Flush(m_nt);
  }

  void TearDown() {
    nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(false));
    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(true));

    while (nt::IsConnected(m_nt)) {
      nt::Flush(m_nt);
    }

    wpi::outs() << "Killed robot program"
                << "\n";

    // Save the JSON and make sure that everything checks out.
    auto path = m_manager->SaveJSON(PROJECT_ROOT_DIR);

    sysid::AnalysisManager analyzer{path, sysid::AnalysisManager::Settings{}};
    auto output = analyzer.Calculate();

    auto ff = std::get<0>(output.ff);
    auto trackWidth = output.trackWidth;

    EXPECT_NEAR(Kv, ff[1], 0.1);
    EXPECT_NEAR(Ka, ff[2], 0.2);

    if (trackWidth) {
      EXPECT_NEAR(kTrackWidth, *trackWidth, 0.1);
    }

    wpi::outs().flush();

    // Delete the JSON.
#ifdef _WIN32
    std::string del = "del " + path;
#else
    std::string del = "rm " + path;
#endif
    std::system(del.c_str());

    // Destroy NT.
    nt::StopClient(m_nt);
    nt::DestroyInstance(m_nt);
  }

  void Run() {
    for (auto&& test : kTests) {
      // Enable the robot.
      nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(true));
      wpi::outs() << "Running: " << test << "\n";
      wpi::outs().flush();

      // Wait 5 ms and flush.
      std::this_thread::sleep_for(5ms);
      nt::Flush(m_nt);

      // Start the test and let it run for 2 or 4 seconds depending on the test.
      m_manager->BeginTest(test);
      auto start = wpi::Now() * 1E-6;
      while (wpi::Now() * 1E-6 - start <
             (wpi::StringRef(test).startswith("fast") ? 2 : 4)) {
        m_manager->Update();
        std::this_thread::sleep_for(0.005s);
        nt::Flush(m_nt);
      }

      // Wait for two seconds while the robot comes to a stop.
      start = wpi::Now() * 1E-6;
      nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(false));
      while (wpi::Now() * 1E-6 - start < 2) {
        m_manager->Update();
        std::this_thread::sleep_for(0.005s);
        nt::Flush(m_nt);
      }

      // Make sure the telemetry manager has ended the test.
      EXPECT_FALSE(m_manager->IsActive());
    }
  }

 private:
  NT_Inst m_nt = nt::CreateInstance();

  NT_Entry m_enable{nt::GetEntry(m_nt, "/SmartDashboard/SysIdRun")};
  NT_Entry m_kill{nt::GetEntry(m_nt, "/SmartDashboard/SysIdKill")};

  std::unique_ptr<sysid::TelemetryManager> m_manager;
  sysid::TelemetryManager::Settings m_settings;
};

TEST_F(IntegrationTest, Drivetrain) {
  SetUp(sysid::analysis::kDrivetrain);
  Run();
}
