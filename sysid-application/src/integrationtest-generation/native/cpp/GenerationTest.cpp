// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>
#include <thread>

#include <fmt/core.h>
#include <ntcore_c.h>
#include <ntcore_cpp.h>
#include <wpi/Logger.h>
#include <wpi/SmallVector.h>
#include <wpi/StringMap.h>
#include <wpi/fs.h>
#include <wpi/timestamp.h>

#include "IntegrationUtils.h"
#include "gtest/gtest.h"
#include "networktables/NetworkTableValue.h"
#include "sysid/Util.h"
#include "sysid/generation/ConfigManager.h"
#include "sysid/view/Generator.h"

using namespace std::chrono_literals;

const wpi::SmallVector<std::string_view, 2> kTalonEncs{"Built-in",
                                                       "Tachometer"};
const wpi::SmallVector<std::string_view, 2> kSMaxEncs{"Encoder Port",
                                                      "Data Port"};
const wpi::SmallVector<std::string_view, 2> kBuiltInEncs{"Built-in"};
const wpi::SmallVector<std::string_view, 2> kGeneralEncs{"CANCoder",
                                                         "roboRIO quadrature"};

wpi::StringMap<wpi::SmallVector<std::string_view, 2>>
    motorControllerEncoderMap = {{"PWM", kGeneralEncs},
                                 {"VictorSPX", kGeneralEncs},
                                 {"TalonSRX", kTalonEncs},
                                 {"TalonFX", kBuiltInEncs},
                                 {"SPARK MAX (Brushless)", kSMaxEncs},
                                 {"SPARK MAX (Brushed)", kSMaxEncs},
                                 {"Venom", kBuiltInEncs}};

const wpi::SmallVector<std::string_view, 4> kPigeonCtors{"0", "WPI_TalonSRX-1"};
const wpi::SmallVector<std::string_view, 4> kAnalogCtors{"0"};
const wpi::SmallVector<std::string_view, 4> kNavXCtors{
    "SerialPort.kUSB", "I2C", "SerialPort.kMXP", "SPI.kMXP"};
const wpi::SmallVector<std::string_view, 4> kADXRS450Ctors{"SPI.kMXP",
                                                           "kOnboardCS0"};
wpi::StringMap<wpi::SmallVector<std::string_view, 4>> gyroCtorMap = {
    {"AnalogGyro", kAnalogCtors},
    {"Pigeon", kPigeonCtors},
    {"ADXRS450", kADXRS450Ctors},
    // FIXME: Waiting on Linux and macOS builds for navX AHRS
    // {"NavX", kNavXCtors},
    {"None", kAnalogCtors}};

class GenerationTest : public ::testing::Test {
 public:
  void SetUp(std::string_view directory) {
    m_nt = nt::GetDefaultInstance();
    m_kill = nt::GetEntry(m_nt, "/SmartDashboard/SysIdKill");
    m_mechanism = nt::GetEntry(m_nt, "/SmartDashboard/SysIdTest");
    m_mechError = nt::GetEntry(m_nt, "/SmartDashboard/SysIdWrongMech");
    m_enable = nt::GetEntry(m_nt, "/SmartDashboard/SysIdRun");

    // Get the path to write the json
    m_jsonPath = fs::path{EXPAND_STRINGIZE(PROJECT_ROOT_DIR)} /
                 "sysid-projects" / "deploy" / "config.json";
    m_directory = directory;

    // Wait between tests
    std::this_thread::sleep_for(2s);
  }

  void Run() {
    LaunchSim(m_directory);

    Connect(m_nt, m_kill);

    fmt::print(stderr,
               "Waiting for 250 ms after connecting to see if program doesn't "
               "crash\n");
    // Makes sure the program didn't crash after 250ms
    std::this_thread::sleep_for(250ms);
    ASSERT_TRUE(nt::IsConnected(m_nt));

    fmt::print(stderr, "Post test sleep (1s)\n");

    std::this_thread::sleep_for(1s);

    KillNT(m_nt, m_kill);
  }

  void LaunchAndConnect() {
    LaunchSim(m_directory);
    Connect(m_nt, m_kill);
  }

  void SetMechanism(std::string_view mechanismName) {
    nt::SetEntryValue(m_mechanism, nt::Value::MakeString(mechanismName));
    nt::Flush(m_nt);

    // Wait some time for NT to update
    std::this_thread::sleep_for(250ms);
  }

  void VerifyMechStatus(bool shouldFail) {
    // Enable robot to trigger autonomous init.
    nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(true));
    nt::Flush(m_nt);

    // Wait some time for NT to update and then disable
    std::this_thread::sleep_for(250ms);
    nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(false));

    // Test to see if the error flag is proper
    auto mechError = nt::GetEntryValue(m_mechError);
    ASSERT_TRUE(mechError->IsBoolean() &&
                mechError->GetBoolean() == shouldFail);
  }

  void EndSimulation() { KillNT(m_nt, m_kill); }

  void TearDown() override {
    // Set kill, enable, and wrong mechanism entry to false for future tests
    nt::SetEntryValue(m_kill, nt::Value::MakeBoolean(false));
    nt::SetEntryValue(m_mechError, nt::Value::MakeBoolean(false));
    nt::SetEntryValue(m_enable, nt::Value::MakeBoolean(false));

    // Make sure client is gone
    nt::StopClient(m_nt);
  }

  std::string m_directory;
  fs::path m_jsonPath;

  sysid::ConfigSettings m_settings;
  sysid::ConfigManager m_manager{m_settings, m_logger};

 private:
  NT_Inst m_nt;

  NT_Entry m_kill;
  NT_Entry m_mechanism;
  NT_Entry m_mechError;
  NT_Entry m_enable;
  static wpi::Logger m_logger;
};
wpi::Logger GenerationTest::m_logger;

TEST_F(GenerationTest, GeneralMechanism) {
  SetUp("sysid-projects:mechanism");
  constexpr size_t size = 2;
  for (auto&& motorController : sysid::kMotorControllers) {
    m_settings.motorControllers =
        wpi::SmallVector<std::string, 3>(size, std::string{motorController});
    for (auto&& encoder : motorControllerEncoderMap[motorController]) {
      m_settings.encoderType = encoder;
      fmt::print(stderr, "Testing: {} and {}\n", motorController, encoder);

      auto json = m_manager.Generate(size);
      sysid::SaveFile(json.dump(), m_jsonPath);

      Run();
      if (HasFatalFailure()) {
        TearDown();
        return;
      }
    }
  }
}

TEST_F(GenerationTest, Drivetrain) {
  SetUp("sysid-projects:drive");
  constexpr size_t size = 2;

  // Talons were chosen due to the pigeon having a case where it plugs into one
  m_settings.motorControllers =
      wpi::SmallVector<std::string, 3>(size, std::string{"TalonSRX"});

  m_settings.encoderType = "Built-in";

  // Encoders + Motor controllers already tested, now just need to test the
  // gyros
  for (auto&& gyro : sysid::kGyros) {
    if (std::string_view{gyro} == "Romi") {
      continue;
    }
    m_settings.gyro = gyro;
    for (auto&& gyroCtor : gyroCtorMap[gyro]) {
      m_settings.gyroCtor = gyroCtor;
      fmt::print(stderr, "Testing: {} using {}\n", gyro, gyroCtor);

      auto json = m_manager.Generate(size);
      sysid::SaveFile(json.dump(), m_jsonPath);

      Run();
      if (HasFatalFailure()) {
        TearDown();
        return;
      }
    }
  }

  // FIXME: Uncomment once Romi vendordep is available
  // m_settings = sysid::kRomiConfig;
  // m_manager.SaveJSON(m_jsonPath, 1, true);
  // fmt::print(stderr, "Testing: Romi Config\n");
  // Run();
}

TEST_F(GenerationTest, WrongMechDrivetrain) {
  SetUp("sysid-projects:drive");
  // Save default config to path
  m_settings = sysid::ConfigSettings();
  auto json = m_manager.Generate(2);
  sysid::SaveFile(json.dump(), m_jsonPath);

  LaunchAndConnect();

  // Makes sure Drivetrain works
  SetMechanism("Drivetrain");
  VerifyMechStatus(false);

  // Makes sure Arm fails
  SetMechanism("Arm");
  VerifyMechStatus(true);

  EndSimulation();
}

TEST_F(GenerationTest, WrongMechGeneralMechanism) {
  SetUp("sysid-projects:mechanism");
  // Save default config to path
  m_settings = sysid::ConfigSettings();
  auto json = m_manager.Generate(1);
  sysid::SaveFile(json.dump(), m_jsonPath);

  LaunchAndConnect();

  // Makes sure Arm works
  SetMechanism("Arm");
  VerifyMechStatus(false);

  // Makes sure Drivetrain fails
  SetMechanism("Drivetrain");
  VerifyMechStatus(true);

  EndSimulation();
}
