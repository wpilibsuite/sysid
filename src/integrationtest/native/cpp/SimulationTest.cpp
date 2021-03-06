// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdlib>
#include <exception>
#include <thread>

#include <ntcore_c.h>
#include <ntcore_cpp.h>
#include <wpi/Logger.h>
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

// Calculated by finding the voltage required to hold the arm horizontal in the
// simulation program.
constexpr double kCos = .250;
constexpr double kG = .002;

// Create our test fixture class so we can reuse the same logic for various test
// mechanisms.
class IntegrationTest : public ::testing::Test {
 public:
  static constexpr const char* kTests[] = {"slow-forward", "slow-backward",
                                           "fast-forward", "fast-backward"};

  static void SetUpTestSuite() {
    m_nt = nt::GetDefaultInstance();
    m_enable = nt::GetEntry(m_nt, "/SmartDashboard/SysIdRun");
    m_kill = nt::GetEntry(m_nt, "/SmartDashboard/SysIdKill");

    // Setup logger.
    m_logger.SetLogger([](unsigned int level, const char* file,
                          unsigned int line,
                          const char* msg) { wpi::outs() << msg << "\n"; });

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
      ASSERT_LT(wpi::Now() - time, 1.5E7);
    }

    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(false));
    nt::Flush(m_nt);
  }

  void SetUp(sysid::AnalysisType mechanism) {
    // Make a new manager
    m_manager =
        std::make_unique<sysid::TelemetryManager>(m_settings, m_logger, m_nt);

    // Change the default settings a little bit.
    m_settings.quasistaticRampRate = 0.75;
    m_settings.mechanism = mechanism;

    if (mechanism == sysid::analysis::kArm) {
      m_settings.units = "Radians";
    } else {
      m_settings.units = "Meters";
    }
  }

  void TearDown() override {
    // Save the JSON and make sure that everything checks out.
    auto path = m_manager->SaveJSON(PROJECT_ROOT_DIR);
    try {
      auto analyzerSettings = sysid::AnalysisManager::Settings{};
      if (m_settings.mechanism == sysid::analysis::kArm) {
        analyzerSettings.motionThreshold = 0.01;  // Reduce threshold for arm
                                                  // test
      }
      sysid::AnalysisManager analyzer{path, analyzerSettings, m_logger};

      auto output = analyzer.Calculate();

      auto ff = std::get<0>(output.ff);
      auto trackWidth = output.trackWidth;

      EXPECT_NEAR(Kv, ff[1], 0.30);
      EXPECT_NEAR(Ka, ff[2], 0.15);

      if (m_settings.mechanism == sysid::analysis::kElevator) {
        EXPECT_NEAR(kG, ff[3], 0.2);
      } else if (m_settings.mechanism == sysid::analysis::kArm) {
        EXPECT_NEAR(kCos, ff[3], 0.15);
      }

      if (trackWidth) {
        EXPECT_NEAR(kTrackWidth, *trackWidth, 0.1);
      }
    } catch (std::exception& e) {
      wpi::outs() << "Teardown Failed: " << e.what() << "\n";
      ADD_FAILURE();
    }

    wpi::outs().flush();

// Delete the JSON.
#ifdef _WIN32
    std::string del = "del " + path;
#else
    std::string del = "rm " + path;
#endif
    std::system(del.c_str());
  }

  static void TearDownTestSuite() {
    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(true));

    while (nt::IsConnected(m_nt)) {
      nt::Flush(m_nt);
    }

    wpi::outs() << "Killed robot program"
                << "\n";

    // Stop NT Client.
    nt::StopClient(m_nt);
  }

  void Run() {
    for (int i = 0; i < 4; i++) {
      auto test = kTests[i];
      m_manager->BeginTest(test);

      // Enable the robot.
      nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(true));
      wpi::outs() << "Running: " << test << "\n";
      wpi::outs().flush();

      // Start the test and let it run for 3 seconds.
      auto start = wpi::Now() * 1E-6;
      while (wpi::Now() * 1E-6 - start < 3) {
        m_manager->Update();
        std::this_thread::sleep_for(0.005s);
        nt::Flush(m_nt);
      }

      // Wait at least one second while the mechanism stops.
      start = wpi::Now() * 1E-6;
      while (m_manager->IsActive() || wpi::Now() * 1E-6 - start < 1) {
        nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(false));
        m_manager->Update();
        std::this_thread::sleep_for(0.005s);
        nt::Flush(m_nt);
      }
    }
  }

 private:
  static NT_Inst m_nt;

  static NT_Entry m_enable;
  static NT_Entry m_kill;

  static std::unique_ptr<sysid::TelemetryManager> m_manager;
  static sysid::TelemetryManager::Settings m_settings;

  static wpi::Logger m_logger;
};

NT_Inst IntegrationTest::m_nt;

NT_Entry IntegrationTest::m_enable;
NT_Entry IntegrationTest::m_kill;

std::unique_ptr<sysid::TelemetryManager> IntegrationTest::m_manager;
sysid::TelemetryManager::Settings IntegrationTest::m_settings;

wpi::Logger IntegrationTest::m_logger;

TEST_F(IntegrationTest, Drivetrain) {
  SetUp(sysid::analysis::kDrivetrain);
  Run();
}

TEST_F(IntegrationTest, Flywheel) {
  SetUp(sysid::analysis::kSimple);
  Run();
}

TEST_F(IntegrationTest, Elevator) {
  SetUp(sysid::analysis::kElevator);
  Run();
}

TEST_F(IntegrationTest, Arm) {
  SetUp(sysid::analysis::kArm);
  Run();
}
