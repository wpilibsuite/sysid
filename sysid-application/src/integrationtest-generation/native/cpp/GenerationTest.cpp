// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>

#include <fmt/core.h>
#include <ntcore_c.h>
#include <ntcore_cpp.h>
#include <wpi/Logger.h>
#include <wpi/SmallVector.h>
#include <wpi/StringExtras.h>
#include <wpi/StringMap.h>
#include <wpi/fs.h>
#include <wpi/timestamp.h>

#include "IntegrationUtils.h"
#include "gtest/gtest.h"
#include "networktables/NetworkTableValue.h"
#include "sysid/Util.h"
#include "sysid/generation/ConfigManager.h"
#include "sysid/generation/HardwareType.h"
#include "sysid/view/Generator.h"

using namespace std::chrono_literals;

const wpi::SmallVector<sysid::HardwareType, 2> kTalonEncs{
    sysid::encoder::kBuiltInSetting, sysid::encoder::kCTRETachometer};
const wpi::SmallVector<sysid::HardwareType, 2> kSMaxEncs{
    sysid::encoder::kSMaxEncoderPort, sysid::encoder::kSMaxDataPort};
const wpi::SmallVector<sysid::HardwareType, 2> kBuiltInEncs{
    sysid::encoder::kBuiltInSetting};
const wpi::SmallVector<sysid::HardwareType, 2> kGeneralEncs{
    // sysid::encoder::kCANCoder,
    sysid::encoder::kRoboRIO};

wpi::StringMap<wpi::SmallVector<sysid::HardwareType, 2>>
    motorControllerEncoderMap = {
        {std::string{sysid::motorcontroller::kPWM.name}, kGeneralEncs},
        //  {std::string{sysid::motorcontroller::kVictorSPX}, kGeneralEncs},
        //  {std::string{sysid::motorcontroller::kTalonSRX}, kTalonEncs},
        //  {std::string{sysid::motorcontroller::kTalonFX}, kBuiltInEncs},
        // {std::string{sysid::motorcontroller::kSPARKMAXBrushless}, kSMaxEncs},
        // {std::string{sysid::motorcontroller::kSPARKMAXBrushed}, kSMaxEncs},
        // {std::string{sysid::motorcontroller::kVenom}, kBuiltInEncs}
};

const wpi::SmallVector<std::string_view, 4> kPigeonCtors{"0", "WPI_TalonSRX-1"};
const wpi::SmallVector<std::string_view, 4> kAnalogCtors{"0"};
const wpi::SmallVector<std::string_view, 4> kNavXCtors{
    "SerialPort.kUSB", "I2C", "SerialPort.kMXP", "SPI.kMXP"};
const wpi::SmallVector<std::string_view, 4> kADXRS450Ctors{"SPI.kMXP",
                                                           "kOnboardCS0"};
wpi::StringMap<wpi::SmallVector<std::string_view, 4>> gyroCtorMap = {
    {std::string{sysid::gyro::kAnalogGyro.name}, kAnalogCtors},
    // {std::string{sysid::gyro::kPigeon.name}, kPigeonCtors},
    // {std::string{sysid::gyro::kADXRS450.name}, kADXRS450Ctors},
    // FIXME: Waiting on Linux and macOS builds for navX AHRS
    // {std::string{sysid::gyro::kNavX.name}, kNavXCtors},
    {std::string{sysid::gyro::kNoGyroOption.name}, kAnalogCtors}};

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

  std::string ReadFile(std::string_view filename) {
    std::ifstream file{filename.data()};
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  void FindInLog(std::string_view str) {
    auto searchString = fmt::format("Setup {}", str);
    fmt::print(stderr, "Searching for: {}\n", searchString);
    bool found = wpi::contains(m_logContent, searchString);
    EXPECT_TRUE(found);

    // Prints out stored output to help with debugging
    if (!found) {
      fmt::print(stderr, "{}\n", m_logContent);
    }
  }

  void TestHardwareConfig(std::string_view motorController,
                          std::string_view encoder) {
    FindInLog(motorController);

    // FIXME: Add in motor controller specific items once CTRE and REV
    // simulations work
    std::string encoderString;
    if (encoder == sysid::encoder::kBuiltInSetting.name) {
      encoderString = fmt::format("{}+{}", motorController, encoder);
    } else if (wpi::starts_with(motorController, "SPARK MAX") &&
               (encoder == sysid::encoder::kSMaxDataPort.name ||
                encoder == sysid::encoder::kSMaxEncoderPort.name)) {
      encoderString = fmt::format("SPARK MAX {}", encoder);
    } else {
      encoderString = encoder;
    }

    FindInLog(encoder);
  }

  void TestHardwareConfig(std::string_view motorController,
                          std::string_view encoder, std::string_view gyro,
                          std::string_view gyroCtor) {
    TestHardwareConfig(motorController, encoder);

    // FIXME: Add in gyro specific conditions once CTRE and REV simulations work
    if (gyro == sysid::gyro::kNoGyroOption.name) {
      FindInLog(gyro);
    } else {
      FindInLog(fmt::format("{}, Port:{}", gyro, gyroCtor));
    }
  }

  void Run() {
    ::testing::internal::CaptureStdout();
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

    // Wait for program output to be generated and then store it
    std::this_thread::sleep_for(1s);
    m_logContent = ::testing::internal::GetCapturedStdout();
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
  std::string m_logContent;
  fs::path m_jsonPath;
  fs::path m_logPath;

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
  for (auto&& motorController : sysid::motorcontroller::kMotorControllers) {
    m_settings.motorControllers =
        wpi::SmallVector<sysid::HardwareType, 3>(size, motorController);
    for (auto&& encoder : motorControllerEncoderMap[motorController.name]) {
      m_settings.encoderType = encoder;
      fmt::print(stderr, "Testing: {} and {}\n", motorController.name,
                 encoder.name);
      auto json = m_manager.Generate(size);
      sysid::SaveFile(json.dump(), m_jsonPath);

      Run();
      if (HasFatalFailure()) {
        TearDown();
        return;
      }

      TestHardwareConfig(motorController.name, encoder.name);
    }
  }
}

TEST_F(GenerationTest, Drivetrain) {
  SetUp("sysid-projects:drive");
  constexpr size_t size = 2;

  // FIXME: Switch back to talon + built-in setup once CTRE is fixed
  // Talons were chosen due to the pigeon having a case where it plugs into one
  // (Temporarily ignored)
  m_settings.motorControllers = wpi::SmallVector<sysid::HardwareType, 3>(
      size, sysid::motorcontroller::kPWM);

  m_settings.encoderType = sysid::encoder::kRoboRIO;

  // Encoders + Motor controllers already tested, now just need to test the
  // gyros
  for (auto&& gyro : sysid::gyro::kGyros) {
    if (gyro == sysid::gyro::kRomiGyro) {
      continue;
    }
    m_settings.gyro = gyro;
    for (auto&& gyroCtor : gyroCtorMap[gyro.name]) {
      m_settings.gyroCtor = gyroCtor;
      fmt::print(stderr, "Testing: {} using {}\n", gyro.name, gyroCtor);
      auto json = m_manager.Generate(size);
      sysid::SaveFile(json.dump(), m_jsonPath);

      Run();
      if (HasFatalFailure()) {
        TearDown();
        return;
      }
      TestHardwareConfig(m_settings.motorControllers[0].name,
                         m_settings.encoderType.name, gyro.name, gyroCtor);
    }
  }

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
