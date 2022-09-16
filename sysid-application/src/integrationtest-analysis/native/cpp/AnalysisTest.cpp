// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdlib>
#include <exception>
#include <string>
#include <thread>

#include <fmt/core.h>
#include <ntcore_c.h>
#include <ntcore_cpp.h>
#include <wpi/Logger.h>
#include <wpi/timestamp.h>

#include "IntegrationUtils.h"
#include "gtest/gtest.h"
#include "networktables/NetworkTableValue.h"
#include "sysid/Util.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/telemetry/TelemetryManager.h"

using namespace std::chrono_literals;

// The constants that are defined in our integration test program.
constexpr double Kv = 1.98;
constexpr double Ka = 0.2;
constexpr double Kv_angular = 1.5;
constexpr double Ka_angular = 0.3;
constexpr double kTrackWidth = 0.762;

// 36000 max samples / 9 samples per entry
constexpr size_t kMaxDataSize = 4000;

// Calculated by finding the voltage required to hold the arm horizontal in the
// simulation program.
constexpr double kCos = .250;
constexpr double kG = .002;

// Create our test fixture class so we can reuse the same logic for various test
// mechanisms.
class AnalysisTest : public ::testing::Test {
 public:
  static constexpr const char* kTests[] = {"slow-forward", "slow-backward",
                                           "fast-forward", "fast-backward"};

  static void SetUpTestSuite() {
    m_nt = nt::GetDefaultInstance();
    m_enable = nt::GetEntry(m_nt, "/SmartDashboard/SysIdRun");
    m_kill = nt::GetEntry(m_nt, "/SmartDashboard/SysIdKill");
    m_overflow = nt::GetEntry(m_nt, "/SmartDashboard/SysIdOverflow");

    // Setup logger.
    m_logger.SetLogger([](unsigned int level, const char* file,
                          unsigned int line,
                          const char* msg) { fmt::print("{}\n", msg); });

    LaunchSim("sysid-projects:analysis-test");

    Connect(m_nt, m_kill);
  }

  void UploadJSON(std::string_view path) {
    std::string jsonFolderPath =
        fmt::format("{}/jsons/", EXPAND_STRINGIZE(PROJECT_ROOT_DIR));

#ifdef _WIN32
    std::string failCommand =
        fmt::format("if not exist \"{}\" mkdir \"{}\" && copy \"{}\" \"{}\"",
                    jsonFolderPath, jsonFolderPath, path, jsonFolderPath);
#else
    std::string failCommand = fmt::format("mkdir -p {} && cp -v {} {}",
                                          jsonFolderPath, path, jsonFolderPath);
#endif
    fmt::print(stderr, "Running: {}\n", failCommand);
    std::system(failCommand.c_str());
  }

  void Initialize(sysid::AnalysisType mechanism) {
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

  void AnalyzeJSON() {
    // Save the JSON and make sure that everything checks out.
    auto path = m_manager->SaveJSON(EXPAND_STRINGIZE(PROJECT_ROOT_DIR));
    try {
      auto analyzerSettings = sysid::AnalysisManager::Settings{};
      sysid::AnalysisManager analyzer{path, analyzerSettings, m_logger};

      analyzer.PrepareData();

      const auto& [ff, trackWidth] = analyzer.CalculateFeedforward();
      const auto& ffGains = std::get<0>(ff);

      fmt::print(stderr, "Ks: {}\nKv: {}\nKa: {}\n", ffGains[0], ffGains[1],
                 ffGains[2]);
      if (m_settings.mechanism != sysid::analysis::kDrivetrainAngular) {
        EXPECT_NEAR(Kv, ffGains[1], 0.003);
        EXPECT_NEAR(Ka, ffGains[2], 0.003);
      } else {
        EXPECT_NEAR(Kv_angular, ffGains[1], 0.003);
        EXPECT_NEAR(Ka_angular, ffGains[2], 0.003);
      }

      if (m_settings.mechanism == sysid::analysis::kElevator) {
        fmt::print(stderr, "Kg: {}\n", ffGains[3]);
        EXPECT_NEAR(kG, ffGains[3], 0.003);
      } else if (m_settings.mechanism == sysid::analysis::kArm) {
        fmt::print(stderr, "KCos: {}\n", ffGains[3]);
        EXPECT_NEAR(kCos, ffGains[3], 0.04);
      }

      if (trackWidth) {
        fmt::print(stderr, "Trackwidth: {}\n", *trackWidth);
        EXPECT_NEAR(kTrackWidth, *trackWidth, 0.1);
      }

      if (HasFailure()) {  // If it failed, write to jsons folder for artifact
                           // upload
        UploadJSON(path);
      }
    } catch (std::exception& e) {
      fmt::print(stderr, "Teardown Failed: {}\n", e.what());
      UploadJSON(path);
      ADD_FAILURE();
    }

// Delete the JSON.
#ifdef _WIN32
    std::string del = "del " + path;
#else
    std::string del = "rm " + path;
#endif
    std::system(del.c_str());
  }

  void VerifyOverflow() {
    // Make sure overflow is true
    auto overflow = nt::GetEntryValue(m_overflow);
    ASSERT_TRUE(overflow->IsBoolean() && overflow->GetBoolean());

    // Make sure the sent data isn't too large
    ASSERT_TRUE(m_manager->GetCurrentDataSize() <= kMaxDataSize);
  }

  static void TearDownTestSuite() {
    KillNT(m_nt, m_kill);
  }

  void RunTest(const char* test, double duration) {
    m_manager->BeginTest(test);

    // Enable the robot.
    nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(true));
    fmt::print(stderr, "Running: {}\n", test);

    // Make sure overflow is false
    auto overflow = nt::GetEntryValue(m_overflow);
    ASSERT_TRUE(overflow->IsBoolean() && !overflow->GetBoolean());

    // Start the test and let it run for specified duration.
    auto start = wpi::Now() * 1E-6;
    while (wpi::Now() * 1E-6 - start < duration) {
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

  void RunFullTests() {
    for (int i = 0; i < 4; i++) {
      auto test = kTests[i];

      // Run each test for 3 seconds
      RunTest(test, 3);
    }
  }

  void RunOverflowTest() {
    RunTest(kTests[0], 25);
  }

 private:
  static NT_Inst m_nt;

  static NT_Entry m_enable;
  static NT_Entry m_kill;
  static NT_Entry m_overflow;

  static std::unique_ptr<sysid::TelemetryManager> m_manager;
  static sysid::TelemetryManager::Settings m_settings;

  static wpi::Logger m_logger;
};

NT_Inst AnalysisTest::m_nt;

NT_Entry AnalysisTest::m_enable;
NT_Entry AnalysisTest::m_kill;
NT_Entry AnalysisTest::m_overflow;

std::unique_ptr<sysid::TelemetryManager> AnalysisTest::m_manager;
sysid::TelemetryManager::Settings AnalysisTest::m_settings;

wpi::Logger AnalysisTest::m_logger;

TEST_F(AnalysisTest, Drivetrain) {
  Initialize(sysid::analysis::kDrivetrain);
  RunFullTests();
  AnalyzeJSON();
}

TEST_F(AnalysisTest, DrivetrainAngular) {
  Initialize(sysid::analysis::kDrivetrainAngular);
  RunFullTests();
  AnalyzeJSON();
}

TEST_F(AnalysisTest, Flywheel) {
  Initialize(sysid::analysis::kSimple);
  RunFullTests();
  AnalyzeJSON();
}

TEST_F(AnalysisTest, Elevator) {
  Initialize(sysid::analysis::kElevator);
  RunFullTests();
  AnalyzeJSON();
}

TEST_F(AnalysisTest, Arm) {
  Initialize(sysid::analysis::kArm);
  RunFullTests();
  AnalyzeJSON();
}

TEST_F(AnalysisTest, Overflow) {
  Initialize(sysid::analysis::kDrivetrain);
  RunOverflowTest();
  VerifyOverflow();
}
