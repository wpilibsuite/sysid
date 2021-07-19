// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

class SysIdLogger {
 public:
  void InitLogging();

  /**
   * Sends data after logging is complete.
   *
   * Called in DisabledInit().
   */
  void SendData();

  /**
   * Makes the current execution thread of the logger a real-time thread which
   * will make it scheduled more consistently.
   */
  static void UpdateThreadPriority();

 protected:
  // 20 seconds of test data * 200 samples/second * 9 doubles/sample (320kB of
  // reserved data) provides a large initial vector size to avoid reallocating
  // during a test
  static constexpr size_t kDataVectorSize = 36000;

  double m_voltageCommand = 0.0;
  double m_motorVoltage = 0.0;
  double m_timestamp = 0.0;
  double m_startTime = 0.0;
  bool m_rotate = false;
  std::string m_testType;
  std::vector<double> m_data;

  /**
   * Creates the SysId logger, disables live view telemetry, sets up the
   * following NT Entries: "SysIdAutoSpeed", "SysIdRotate", "SysIdTelemetry"
   */
  SysIdLogger();

  /**
   * Updates the autospeed and robotVoltage
   */
  void UpdateData();

  /**
   * Reset data before next test.
   */
  virtual void Reset();

 private:
  static constexpr int kThreadPriority = 15;
  static constexpr int kHALThreadPriority = 40;
};
