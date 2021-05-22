// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <memory>
#include <string>

#include <glass/View.h>
#include <wpi/Logger.h>
#include <wpi/SmallString.h>
#include <wpi/SmallVector.h>

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

static constexpr const char* kEncoders[] = {"Built-In", "CANCoder / Alternate",
                                            "roboRIO"};

static constexpr const char* kGyros[] = {"Analog", "ADXRS450", "NavX", "Pigeon",
                                         "None"};
static constexpr const char* kNavXCtors[] = {"SerialPort.kUSB", "I2C",
                                             "SerialPort.kMXP", "SPI.kMXP"};

static constexpr const char* kADXRS450Ctors[] = {"SPI.kMXP", "kOnboardCS0"};

static constexpr const char* kCTREPeriods[] = {"1",  "2",  "5",  "10",
                                               "25", "50", "100"};

class Generator : public glass::View {
 public:
  explicit Generator(wpi::Logger& logger);
  void Display() override;

  static constexpr const char* kAnalysisTypes[] = {"General Mechanism",
                                                   "Drivetrain", "Romi"};

 private:
  void GeneratorUI();
  void SelectCTREVelocityPeriod();

  std::unique_ptr<ConfigManager> m_manager;

  ConfigSettings m_settings;

  // Persistent storage pointers.
  double* m_pUnitsPerRotation;
  std::string* m_pAnalysisType;

  // Indices for combo boxes.
  int m_analysisIdx = 0;
  int m_encoderIdx = 2;
  int m_gyroIdx = 2;
  int m_unitsIdx = 0;
  int m_periodIdx = 6;

  // Ports count tracker
  size_t m_portsCount = 1;

  // Pigeon Specific Parameters
  int m_gyroPort = 1;
  int m_gyroParam = 0;
  bool m_isTalon = false;

  // Brushless Tracker
  bool m_hasBrushed = false;

  // Logger
  wpi::Logger m_logger;
};  // namespace sysid
}  // namespace sysid
