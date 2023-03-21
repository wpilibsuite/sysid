// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <string>
#include <string_view>
#include <thread>

#include <fmt/core.h>
#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <wpi/Logger.h>
#include <wpi/SmallVector.h>
#include <wpi/StringExtras.h>
#include <wpi/StringMap.h>
#include <wpi/timestamp.h>

#include "IntegrationUtils.h"
#include "gtest/gtest.h"
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
        {std::string{sysid::motorcontroller::kVictorSPX.name}, kGeneralEncs},
        {std::string{sysid::motorcontroller::kTalonSRX.name}, kTalonEncs},
        {std::string{sysid::motorcontroller::kTalonFX.name}, kBuiltInEncs},
        {std::string{sysid::motorcontroller::kTalonFXPro.name}, kBuiltInEncs},
        {std::string{sysid::motorcontroller::kSPARKMAXBrushless.name},
         kSMaxEncs},
        {std::string{sysid::motorcontroller::kSPARKMAXBrushed.name}, kSMaxEncs},
        {std::string{sysid::motorcontroller::kVenom.name}, kBuiltInEncs}};

const wpi::SmallVector<std::string_view, 4> kPigeonCtors{
    "0", "WPI_TalonSRX-1", "WPI_TalonSRX-2", "WPI_TalonSRX-10"};
const wpi::SmallVector<std::string_view, 4> kAnalogCtors{"0"};
const wpi::SmallVector<std::string_view, 4> kNavXCtors{
    "SerialPort (USB)", "I2C (MXP)", "SerialPort (MXP)", "SPI (MXP)"};
const wpi::SmallVector<std::string_view, 4> kADXRS450Ctors{"SPI (MXP)",
                                                           "SPI (Onboard CS0)"};
const wpi::SmallVector<std::string_view, 4> kADIS16448Ctors{
    "SPI (MXP)", "SPI (Onboard CS0)", "SPI (Onboard CS1)", "SPI (Onboard CS2)",
    "SPI (Onboard CS3)"};
const wpi::SmallVector<std::string_view, 4> kADIS16470Ctors{
    "SPI (Onboard CS0)", "SPI (Onboard CS1)", "SPI (Onboard CS2)",
    "SPI (Onboard CS3)", "SPI (MXP)"};
wpi::StringMap<wpi::SmallVector<std::string_view, 4>> gyroCtorMap = {
    {std::string{sysid::gyro::kAnalogGyro.name}, kAnalogCtors},
    {std::string{sysid::gyro::kPigeon.name}, kPigeonCtors},
    {std::string{sysid::gyro::kPigeon2.name}, kAnalogCtors},
    {std::string{sysid::gyro::kPigeon2Pro.name}, kAnalogCtors},
    {std::string{sysid::gyro::kADXRS450.name}, kADXRS450Ctors},
    {std::string{sysid::gyro::kNavX.name}, kNavXCtors},
    {std::string{sysid::gyro::kADIS16448.name}, kADIS16448Ctors},
    {std::string{sysid::gyro::kADIS16470.name}, kADIS16470Ctors},
    {std::string{sysid::gyro::kNoGyroOption.name}, kAnalogCtors}};

class GenerationTest : public ::testing::Test {
 public:
  void Initialize(std::string_view directory) {
    m_nt = nt::NetworkTableInstance::GetDefault();
    m_kill =
        m_nt.GetTable("SmartDashboard")->GetBooleanTopic("SysIdKill").Publish();
    m_mechanism =
        m_nt.GetTable("SmartDashboard")->GetStringTopic("SysIdTest").Publish();
    m_mechErrorPub = m_nt.GetTable("SmartDashboard")
                         ->GetBooleanTopic("SysIdWrongMech")
                         .Publish();
    m_mechErrorSub = m_nt.GetTable("SmartDashboard")
                         ->GetBooleanTopic("SysIdWrongMech")
                         .Subscribe(false);
    m_enable =
        m_nt.GetTable("SmartDashboard")->GetBooleanTopic("SysIdRun").Publish();

    // Get the path to write the json
    m_jsonPath = std::filesystem::path{EXPAND_STRINGIZE(PROJECT_ROOT_DIR)} /
                 "sysid-projects" / "deploy" / "config.json";
    m_directory = directory;

    // Wait between tests
    std::this_thread::sleep_for(2s);
  }

  void FindInLog(std::string_view str, std::string_view prefix = "Setup") {
    auto searchString = fmt::format("{} {}", prefix, str);
    fmt::print(stderr, "Searching for: {}\n", searchString);
    bool found = wpi::contains(m_logContent, searchString);
    EXPECT_TRUE(found);

    // Prints out stored output to help with debugging
    if (!found) {
      fmt::print(stderr, "******\nOutput Searched Through:\n{}\n******\n",
                 m_logContent);
    }
  }

  void TestHardwareConfig(std::string_view motorController,
                          std::string_view encoder) {
    FindInLog(motorController);

    std::string encoderString;
    if (encoder == sysid::encoder::kBuiltInSetting.name) {
      encoderString = fmt::format("{}+{}", encoder, motorController);
    } else if (wpi::starts_with(motorController, "SPARK MAX") &&
               (encoder == sysid::encoder::kSMaxDataPort.name ||
                encoder == sysid::encoder::kSMaxEncoderPort.name)) {
      if (encoder == sysid::encoder::kSMaxEncoderPort.name) {
        encoderString = fmt::format("{} {}", motorController, encoder);
      } else {
        encoderString = fmt::format("SPARK MAX {}", encoder);
      }
    } else {
      encoderString = encoder;
    }

    FindInLog(encoderString);

    std::string voltageAccessor;
    if (motorController == sysid::motorcontroller::kSPARKMAXBrushless.name ||
        motorController == sysid::motorcontroller::kSPARKMAXBrushed.name) {
      voltageAccessor = "SPARK MAX";
    } else if (motorController == sysid::motorcontroller::kTalonSRX.name ||
               motorController == sysid::motorcontroller::kVictorSPX.name ||
               motorController == sysid::motorcontroller::kTalonFX.name) {
      voltageAccessor = "CTRE";
    } else if (motorController == sysid::motorcontroller::kTalonFXPro.name) {
      voltageAccessor = "CTRE (Pro)";
    } else if (motorController == sysid::motorcontroller::kVenom.name) {
      voltageAccessor = "Venom";
    } else {
      voltageAccessor = "General";
    }
    FindInLog(fmt::format("{} voltage", voltageAccessor), "Recording");
  }

  void TestHardwareConfig(std::string_view motorController,
                          std::string_view encoder, std::string_view gyro,
                          std::string_view gyroCtor) {
    TestHardwareConfig(motorController, encoder);
    // FIXME: Add in gyro specific conditions once CTRE and REV simulations work
    if (gyro == sysid::gyro::kNoGyroOption.name) {
      FindInLog(gyro);
    } else if (gyro == sysid::gyro::kPigeon.name) {
      std::string pigeonPortStr;
      if (wpi::starts_with(gyroCtor, "WPI_TalonSRX")) {
        auto portStr = wpi::split(gyroCtor, "-").second;
        if (portStr == "10") {
          // If the motor controller pigeon plugged into hasn't been declared
          pigeonPortStr =
              fmt::format("{} (plugged to other motorcontroller)", portStr);

        } else {
          // If the motor controller is plugged into an already declared pigeon
          pigeonPortStr =
              fmt::format("{} (plugged to drive motorcontroller)", portStr);
        }
      } else {
        // Pigeon is plugged into CAN
        pigeonPortStr = fmt::format("{} (CAN)", gyroCtor);
      }
      FindInLog(fmt::format("Pigeon, {}", pigeonPortStr));
    } else if (gyro == sysid::gyro::kPigeon2.name) {
      FindInLog(fmt::format("Pigeon2, {} (CAN)", gyroCtor));
    } else if (gyro == sysid::gyro::kPigeon2Pro.name) {
      FindInLog(fmt::format("Pigeon2, {} (CAN) (Pro)", gyroCtor));
    } else if (gyro == sysid::gyro::kADXRS450.name ||
               gyro == sysid::gyro::kADIS16448.name ||
               gyro == sysid::gyro::kADIS16470.name) {
      FindInLog(gyro);
      FindInLog(fmt::format("{}", gyroCtor));
    } else if (gyro == sysid::gyro::kRomiGyro.name) {
      FindInLog("Romi");
    } else {
      FindInLog(fmt::format("{}, {}", gyro, gyroCtor));
    }
  }

  void Run() {
    LaunchSim(m_directory);

    Connect(m_nt, m_kill);

    fmt::print(stderr,
               "Waiting for 250 ms after connecting to see if program doesn't "
               "crash\n");
    // Makes sure the program didn't crash after 250ms
    std::this_thread::sleep_for(250ms);
    ASSERT_TRUE(m_nt.IsConnected());

    // Wait longer for output to be flushed
    fmt::print(stderr, "Post test sleep (2s)\n");
    std::this_thread::sleep_for(2s);

    m_logContent = KillNT(m_nt, m_kill);
  }

  void LaunchAndConnect() {
    LaunchSim(m_directory);
    Connect(m_nt, m_kill);
  }

  void SetMechanism(std::string_view mechanismName) {
    m_mechanism.Set(mechanismName);
    m_nt.Flush();

    // Wait some time for NT to update
    std::this_thread::sleep_for(250ms);
  }

  void VerifyMechStatus(bool shouldFail) {
    // Enable robot to trigger autonomous init.
    m_enable.Set(true);
    m_nt.Flush();

    // Wait some time for NT to update and then disable
    std::this_thread::sleep_for(250ms);
    m_enable.Set(false);

    // Test to see if the error flag is proper
    ASSERT_EQ(m_mechErrorSub.Get(), shouldFail);
  }

  void EndSimulation() { KillNT(m_nt, m_kill); }

  void TearDown() override {
    // Set kill, enable, and wrong mechanism entry to false for future tests
    m_kill.Set(false);
    m_mechErrorPub.Set(false);
    m_enable.Set(false);

    // Make sure client is gone
    m_nt.StopClient();
  }

  std::string m_directory;
  std::string m_logContent;
  std::filesystem::path m_jsonPath;

  sysid::ConfigSettings m_settings;
  sysid::ConfigManager m_manager{m_settings, m_logger};

 private:
  nt::NetworkTableInstance m_nt;

  nt::BooleanPublisher m_kill;
  nt::StringPublisher m_mechanism;
  nt::BooleanPublisher m_mechErrorPub;
  nt::BooleanSubscriber m_mechErrorSub;
  nt::BooleanPublisher m_enable;
  static wpi::Logger m_logger;
};
wpi::Logger GenerationTest::m_logger;

TEST_F(GenerationTest, GeneralMechanism) {
  Initialize("sysid-projects:mechanism");
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
  Initialize("sysid-projects:drive");
  constexpr size_t size = 2;

  m_settings.motorControllers = wpi::SmallVector<sysid::HardwareType, 3>(
      size, sysid::motorcontroller::kTalonSRX);

  m_settings.encoderType = sysid::encoder::kBuiltInSetting;

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
  // auto json = m_manager.Generate(size);
  // sysid::SaveFile(json.dump(), m_jsonPath);
  // fmt::print(stderr, "Testing: Romi Config\n");
  // Run();
  if (HasFatalFailure()) {
    TearDown();
    return;
  }
  TestHardwareConfig(m_settings.motorControllers[0].name,
                     m_settings.encoderType.name, m_settings.gyro.name,
                     m_settings.gyroCtor);
}

TEST_F(GenerationTest, WrongMechDrivetrain) {
  Initialize("sysid-projects:drive");
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
  Initialize("sysid-projects:mechanism");
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
