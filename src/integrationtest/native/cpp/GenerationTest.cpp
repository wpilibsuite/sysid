// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <thread>

#include <ntcore_c.h>
#include <ntcore_cpp.h>
#include <wpi/FileSystem.h>
#include <wpi/Logger.h>
#include <wpi/SmallString.h>
#include <wpi/SmallVector.h>
#include <wpi/StringMap.h>
#include <wpi/StringRef.h>
#include <wpi/raw_ostream.h>
#include <wpi/timestamp.h>

#include "gtest/gtest.h"
#include "networktables/NetworkTableValue.h"
#include "sysid/Util.h"
#include "sysid/generation/ConfigManager.h"
#include "sysid/view/Generator.h"

using namespace std::chrono_literals;

const wpi::SmallVector<std::string, 4> kPigeonCtors{"0", "WPI_TalonSRX-1"};
const wpi::SmallVector<std::string, 4> kAnalogCtors{"0"};
const wpi::SmallVector<std::string, 4> kNavXCtors{"SPI.kMXP"};
const wpi::SmallVector<std::string, 4> kADXRS450Ctors{"SPI.kMXP"};
wpi::StringMap<wpi::SmallVector<std::string, 4>> gyroCtorMap = {
    {"AnalogGyro", kAnalogCtors},
    {"Pigeon", kPigeonCtors},
    {"ADXRS450", kADXRS450Ctors},
    {"NavX", kNavXCtors},
    {"None", kAnalogCtors}};

class GenerationTest : public ::testing::Test {
 public:
  GenerationTest() : m_manager(m_settings, m_logger) {}

  void SetUp(wpi::StringRef directory) {
    m_nt = nt::GetDefaultInstance();
    m_kill = nt::GetEntry(m_nt, "/SmartDashboard/SysIdKill");

    wpi::SmallString<128> path;
    wpi::raw_svector_ostream os(path);

    os << PROJECT_ROOT_DIR << SYSID_PATH_SEPARATOR << directory;

    m_directory = path.c_str();

    // Get the path to write the json

    os << SYSID_PATH_SEPARATOR;
    m_jsonPath = path.c_str();

    // Wait between tests
    std::this_thread::sleep_for(2s);
  }

  void Run() {
    // Start the robot program.
    wpi::SmallString<128> cmd;
    wpi::raw_svector_ostream os(cmd);

    os << "cd " << m_directory << " && " << LAUNCHSIM << " -Pintegration";
    wpi::outs() << "Executing: " << cmd.c_str() << "\n";
    wpi::outs().flush();

    int result = std::system(cmd.c_str());
    ASSERT_EQ(0, result) << "The robot program couldn't be started";

    nt::StartClient(m_nt, "localhost", NT_DEFAULT_PORT);

    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(false));
    nt::Flush(m_nt);

    // Wait for NT to connect or until it times out.
    auto time = wpi::Now();
    while (!nt::IsConnected(m_nt)) {
      ASSERT_LT(wpi::Now() - time, 1.5E7);
    }

    wpi::outs() << "waiting for .25 seconds after connecting to see if program "
                   "doesn't crash\n";
    wpi::outs().flush();
    // Makes sure the program didn't crash after .25s.
    std::this_thread::sleep_for(.25s);
    ASSERT_TRUE(nt::IsConnected(m_nt));

    wpi::outs() << "post test sleep (1s)\n";
    wpi::outs().flush();

    std::this_thread::sleep_for(1s);

    wpi::outs() << "Killing program\n";
    wpi::outs().flush();
    time = wpi::Now();

    while (nt::IsConnected(m_nt)) {
      // Kill program
      nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(true));
      nt::Flush(m_nt);
      if (wpi::Now() - time > 3E7) {
        FAIL();
        break;
      }
    }

    wpi::outs() << "Killed robot program"
                << "\n";
    wpi::outs().flush();

    // Set kill entry to false for future tests
    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(false));
    // Stop NT Client.
    nt::StopClient(m_nt);
  }

  void TearDown() override {
    // Set kill entry to false for future tests
    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(false));

    // Make sure client is gone
    nt::StopClient(m_nt);
  }

  std::string m_directory;
  std::string m_jsonPath;

  sysid::ConfigSettings m_settings;
  sysid::ConfigManager m_manager;

 private:
  NT_Inst m_nt;

  NT_Entry m_kill;
  static wpi::Logger m_logger;
};
wpi::Logger GenerationTest::m_logger;

TEST_F(GenerationTest, SingleSided) {
  SetUp("base_projects/GeneralMechanism");
  constexpr size_t size = 2;
  for (auto&& motorController : sysid::kMotorControllers) {
    m_settings.m_motorControllers =
        wpi::SmallVector<std::string, 3>(size, std::string{motorController});
    for (auto&& encoder : sysid::kEncoders) {
      if (wpi::StringRef(encoder) != "roboRIO" &&
          (wpi::StringRef(motorController) == "PWM" ||
           wpi::StringRef(motorController) == "VictorSPX")) {
        continue;
      }
      m_settings.m_encoderType = encoder;
      wpi::outs() << "Testing: " << motorController << " and " << encoder
                  << "\n";
      m_manager.SaveJSON(m_jsonPath, size);
      Run();
      if (HasFatalFailure()) {
        TearDown();
        return;
      }
    }
  }
}

TEST_F(GenerationTest, Drivetrain) {
  SetUp("base_projects/Drivetrain");
  constexpr size_t size = 2;

  // Talons were chosen due to the pigeon having a case where it plugs into one
  m_settings.m_motorControllers =
      wpi::SmallVector<std::string, 3>(size, std::string{"TalonSRX"});

  m_settings.m_encoderType = "Built-In";

  // Encoders + Motor controllers already tested, now just need to test the
  // gyros
  for (auto&& gyro : sysid::kGyros) {
    if (wpi::StringRef(gyro) == "Romi") {
      continue;
    }
    m_settings.m_gyro = gyro;
    for (auto&& gyroCtor : gyroCtorMap[gyro]) {
      m_settings.m_gyroCtor = gyroCtor;
      wpi::outs() << "Testing: " << gyro << " using " << gyroCtor << "\n";
      m_manager.SaveJSON(m_jsonPath, size);
      Run();
      if (HasFatalFailure()) {
        TearDown();
        return;
      }
    }
  }

  m_settings = sysid::kRomiConfig;
  m_manager.SaveJSON(m_jsonPath, 1, true);
  wpi::outs() << "Testing: Romi Config\n";
  Run();
}
