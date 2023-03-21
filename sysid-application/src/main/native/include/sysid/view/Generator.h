// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <glass/View.h>
#include <imgui.h>
#include <portable-file-dialogs.h>
#include <wpi/Logger.h>
#include <wpi/mutex.h>
#include <wpinet/EventLoopRunner.h>

#include "sysid/deploy/DeploySession.h"
#include "sysid/generation/ConfigManager.h"
#include "sysid/generation/HardwareType.h"

namespace glass {
class Storage;
}  // namespace glass

namespace sysid {
/**
 * Constexpr way of storing a const char* array of HardwareType names to be used
 * with ImGui's const char* setup. Adapted from:
 * https://stackoverflow.com/a/34465458
 *
 * @tparam S The size of the array storing the HardwareTypes that will be used.
 */
template <size_t S>
struct DisplayNameStorage {
  /**
   * A const char* array to hold all the HardwareType names
   */
  const char* names[S]{};

  /**
   * An int to pass into the size parameter of functions like `ImGui::Combo`
   */
  const int size{S};

  /**
   * Constructs a constexpr DisplayNameStorage object
   *
   * @param devices The array of hardware devices to store display names for.
   */
  explicit constexpr DisplayNameStorage(std::array<HardwareType, S> devices) {
    for (size_t i = 0; i < S; i++) {
      names[i] = devices[i].displayName;
    }
  }
};

// Options
static constexpr auto kMotorControllerNames =
    DisplayNameStorage(sysid::motorcontroller::kMotorControllers);

static constexpr std::array<const char*, 3> kGeneralEncoders{
    sysid::encoder::kCANCoder.displayName,
    sysid::encoder::kCANcoderPro.displayName,
    sysid::encoder::kRoboRIO.displayName};

static constexpr std::array<const char*, 2> kTalonSRXEncoders{
    sysid::encoder::kBuiltInSetting.displayName,
    sysid::encoder::kCTRETachometer.displayName};

static constexpr std::array<const char*, 1> kBuiltInEncoders{
    sysid::encoder::kBuiltInSetting.displayName};

static constexpr std::array<const char*, 2> kSparkMaxEncoders{
    sysid::encoder::kSMaxEncoderPort.displayName,
    sysid::encoder::kSMaxDataPort.displayName};

static constexpr auto kGyroNames = DisplayNameStorage(sysid::gyro::kGyros);
static constexpr const char* kNavXCtors[] = {"SerialPort (USB)", "I2C (MXP)",
                                             "SerialPort (MXP)", "SPI (MXP)"};

static constexpr const char* kADXRS450Ctors[] = {"SPI (Onboard CS0)",
                                                 "SPI (MXP)"};

static constexpr const char* kADIS16448Ctors[] = {
    "SPI (MXP)", "SPI (Onboard CS0)", "SPI (Onboard CS1)", "SPI (Onboard CS2)",
    "SPI (Onboard CS3)"};

static constexpr const char* kADIS16470Ctors[] = {
    "SPI (Onboard CS0)", "SPI (Onboard CS1)", "SPI (Onboard CS2)",
    "SPI (Onboard CS3)", "SPI (MXP)"};

// https://codedocs.revrobotics.com/cpp/classrev_1_1_spark_max_relative_encoder.html#ad2220467e1725840dbe77db4278a8d80
static constexpr const char* kREVBuiltInNumSamples[] = {"1", "2", "4", "8"};

// https://codedocs.revrobotics.com/cpp/classrev_1_1_spark_max_relative_encoder.html#a8922f3c305ef6e57d7f639a17496c45d
static constexpr const char* kREVPeriods[] = {
    "8",  "9",  "10", "11", "12", "13", "14", "15", "16", "17", "18", "19",
    "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31",
    "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43",
    "44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54", "55",
    "56", "57", "58", "59", "60", "61", "62", "63", "64"};

// https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters
static constexpr const char* kCTREBuiltInNumSamples[] = {"1",  "2",  "4", "8",
                                                         "16", "32", "64"};
static constexpr const char* kCTREPeriods[] = {"1",  "5",  "10", "20",
                                               "25", "50", "100"};

/**
 * The generator GUI takes care of providing the user with a user interface to
 * select their project parameters and then deploying the robot program to a
 * roboRIO.
 */
class Generator : public glass::View {
 public:
  /**
   * Creates a Generator Widget
   *
   * @param storage Glass Storage
   * @param logger The program logger
   */
  Generator(glass::Storage& storage, wpi::Logger& logger);

  /**
   * Displays the generator widget.
   */
  void Display() override;

  /**
   * The supported project types.
   */
  static constexpr const char* kAnalysisTypes[] = {"General Mechanism",
                                                   "Drivetrain", "Romi"};

 private:
  /**
   * Helper method to display widgets to select the CTRE Encoder Velocity Period
   */
  void SelectCTREVelocityPeriod();

  /**
   * Updates GUI based on the stored config
   */
  void UpdateFromConfig();

  /**
   * Displays the widgets to invert a motor controller specific encoder.
   *
   * @param drive True if the encoder setup is for a Drivetrain.
   */
  void RegularEncoderSetup(bool drive);

  /**
   * Displays the widgets to setup encoders plugged into the roboRIO
   *
   * @param drive True if the encoder setup is for a Drivetrain.
   */
  void RoboRIOEncoderSetup(bool drive);

  /**
   * Displays the widgets to setup the CANCoder
   *
   * @param drive True if the encoder setup is for a Drivetrain.
   * @param usePro True if CANcoder is using Pro firmware
   */
  void CANCoderSetup(bool drive, bool usePro);

  /**
   * Displays the encoder options for a specific combination of motor controller
   * encoders.
   *
   * @tparam Size The size of the encoder array
   * @param encoders The array of motor controller specific encoders and general
   *                 encoders.
   */
  template <size_t Size>
  void GetEncoder(const std::array<const char*, Size>& encoders) {
    ImGui::Combo("Encoder", &m_encoderIdx, encoders.data(), encoders.size());
    m_settings.encoderType =
        sysid::encoder::FromEncoderName(encoders[m_encoderIdx]);
    if (m_settings.motorControllers[0] ==
            sysid::motorcontroller::kSPARKMAXBrushless &&
        m_settings.encoderType == sysid::encoder::kSMaxEncoderPort) {
      // Spark Max built-in encoder number of samples must be within 8 to 64
      // inclusive
      m_settings.numSamples = 8;
    }
  }

  // Configuration manager along with its settings -- used to generate the JSON
  // configuration.
  std::unique_ptr<ConfigManager> m_manager;
  ConfigSettings m_settings;

  // Persistent storage pointers for project generation.
  double& m_unitsPerRotation;
  std::string& m_analysisType;

  // Indices for combo boxes.
  int m_analysisIdx = 0;
  int m_encoderIdx = 0;
  int m_gyroIdx = 0;
  int m_unitsIdx = 0;
  int m_numSamplesIdx = 0;
  int m_periodIdx = 0;

  HardwareType m_prevMainMotorController = sysid::motorcontroller::kPWM;

  // Keeps track of the number of motor ports the user wants.
  size_t m_occupied = 1;

  // CTRE Pigeon-specific parameters.
  int m_gyroPort = 1;
  int m_gyroParam = 0;
  bool m_isTalon = false;

  // Whether the user is running Spark MAX in Brushed Mode.
  bool m_isSparkMaxBrushed = false;

  // Error Popup
  bool m_errorPopup = false;
  std::string m_errorMessage = "";

  // Selectors for files
  std::unique_ptr<pfd::save_file> m_saveConfigSelector;
  std::unique_ptr<pfd::open_file> m_loadConfigSelector;
  // Logger
  wpi::Logger& m_logger;

  // The team number for the deploy process -- can also be a hostname or IP
  // address of a RoboRIO. This points to the same location in memory as the
  // "team" field in the Logger GUI.
  std::string& m_team;

  // Create a separate logger for the deploy process. We can display all output
  // messages in the modal popup during the deploy.
  wpi::Logger m_deployLogger;

  // Create an event loop runner (runs a libuv event loop in a separate thread)
  // for the deploy process.
  wpi::EventLoopRunner m_deployRunner;

  // Represents the currently running or most recent deploy session.
  std::unique_ptr<DeploySession> m_deploySession;

  // Represents storage for a log message sent from the deploy event loop.
  struct DeployLogMessage {
    std::string message;
    unsigned int level;
  };

  // A vector of log messages from the deploy event loop.
  std::vector<DeployLogMessage> m_deployLog;

  // Mutex for accessing the deploy log. Because the event loop runs on a
  // separate thread, this mutex must be locked when reading/writing to the log.
  wpi::mutex m_deployMutex;
};  // namespace sysid
}  // namespace sysid
