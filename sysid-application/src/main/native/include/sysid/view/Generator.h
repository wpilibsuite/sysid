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
#include <portable-file-dialogs.h>
#include <wpi/EventLoopRunner.h>
#include <wpi/Logger.h>
#include <wpi/mutex.h>

#include "sysid/Util.h"
#include "sysid/deploy/DeploySession.h"
#include "sysid/generation/ConfigManager.h"

namespace sysid {

// Options
static constexpr const char* kMotorControllers[] = {"PWM",
                                                    "TalonSRX",
                                                    "VictorSPX",
                                                    "TalonFX",
                                                    "SPARK MAX (Brushless)",
                                                    "SPARK MAX (Brushed)",
                                                    "Venom"};

static constexpr std::array<const char*, 2> kGeneralEncoders = {
    "CANCoder", "roboRIO quadrature"};

static constexpr std::array<const char*, 2> kTalonSRXEncoders = {"Built-in",
                                                                 "Tachometer"};

static constexpr std::array<const char*, 1> kBuiltInEncoders = {"Built-in"};

static constexpr std::array<const char*, 2> kSparkMaxEncoders = {"Encoder Port",
                                                                 "Data Port"};

static constexpr const char* kGyros[] = {"Analog", "ADXRS450", "NavX", "Pigeon",
                                         "None"};
static constexpr const char* kNavXCtors[] = {"SerialPort.kUSB", "I2C",
                                             "SerialPort.kMXP", "SPI.kMXP"};

static constexpr const char* kADXRS450Ctors[] = {"SPI.kMXP", "kOnboardCS0"};

static constexpr const char* kCTREPeriods[] = {"1",  "2",  "5",  "10",
                                               "25", "50", "100"};

/**
 * The generator GUI takes care of providing the user with a user interface to
 * select their project parameters and then deploying the robot program to a
 * roboRIO.
 */
class Generator : public glass::View {
 public:
  explicit Generator(wpi::Logger& logger);
  void Display() override;

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
   */
  void CANCoderSetup(bool drive);

  /**
   * Displays the encoder options for a specific combination of motor controller
   * specific encoders and general encoders.
   *
   * @tparam X The size of the specific encoder array
   * @tparam Y The size of the general encoder array
   *
   * @param specificEncoders The array storing motor controller specific arrays
   * @param generalEncoder The array storing general encoders.
   */
  template <size_t X, size_t Y>
  void GetEncoder(const std::array<const char*, X>& specificEncoders,
                  const std::array<const char*, Y>& generalEncoders);

  // Configuration manager along with its settings -- used to generate the JSON
  // configuration.
  std::unique_ptr<ConfigManager> m_manager;
  ConfigSettings m_settings;

  // Persistent storage pointers for project generation.
  double* m_pUnitsPerRotation;
  std::string* m_pAnalysisType;

  // Indices for combo boxes.
  int m_analysisIdx = 0;
  int m_encoderIdx = 2;
  int m_gyroIdx = 2;
  int m_unitsIdx = 0;
  int m_periodIdx = 6;

  std::string_view m_prevMainMotorController;

  // Keeps track of the number of motor ports the user wants.
  size_t m_occupied = 1;

  // CTRE Pigeon-specific parameters.
  int m_gyroPort = 1;
  int m_gyroParam = 0;
  bool m_isTalon = false;

  // Whether the user is running Spark MAX in Brushed Mode.
  bool m_isSparkMaxBrushed = false;

  // Selectors for files
  std::unique_ptr<pfd::save_file> m_saveConfigSelector;
  std::unique_ptr<pfd::open_file> m_loadConfigSelector;
  // Logger
  wpi::Logger& m_logger;

  // The team number for the deploy process -- can also be a hostname or IP
  // address of a RoboRIO. This points to the same location in memory as the
  // "team" field in the Logger GUI.
  std::string* m_pTeam;

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
