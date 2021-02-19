// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>

#include <units/voltage.h>

class SysIdLogger {
 public:
  /**
   * Sends data after logging is complete.
   * Called in DisabledInit();
   */
  void SendData();

  /**
   * Makes the current execution thread of the logger a realtime thread which
   * ensures that the logging loop will consistently meet its loop time (ideally
   * 5 ms).
   */
  void UpdateThreadPriority();

  void InitLogging();

 protected:
  /**
   * Creates the SysId logger, disables live view telemetry, sets up the
   * following NT Entries: "SysIdAutoSpeed", "SysIdRotate", "SysIdTelemetry"
   */
  SysIdLogger();

  units::volt_t m_primaryMotorVoltage;
  units::volt_t m_secondaryMotorVoltage;
  double m_voltageCommand;
  double m_motorVoltage;
  double m_timestamp;
  double m_startTime;
  bool m_rotate;
  std::string m_testType;
  std::vector<double> m_data;

  /**
   * Updates the autospeed and robotVoltage
   */
  void UpdateData();

 private:
  static constexpr int kThreadPriority = 15;
  static constexpr int kHALThreadPriority = 40;

  // 20 seconds of test data * 200 samples/second * 10 doubles/sample (320kB of
  // reserved data) provides a large initial vector size to avoid reallocating
  // during a test
  static constexpr size_t kDataVectorSize = 40000;
};
