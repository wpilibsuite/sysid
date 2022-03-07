// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <frc/motorcontrol/MotorController.h>

namespace sysid {

/**
 * Base class that serves to provide methods for robot projects seeking to send
 * and receive data in occurdence to the SysId application's protocols.
 */
class SysIdLogger {
 public:
  /**
   * Code that should be run to initialize the logging routine. Should be called
   * in `AutonomousInit()`.
   */
  void InitLogging();

  /**
   * Sends data after logging is complete.
   *
   * Called in DisabledInit().
   */
  void SendData();

  /**
   * Clears the data entry when sysid logger acknowledges that it received data.
   */
  void ClearWhenReceived();

  /**
   * Makes the current execution thread of the logger a real-time thread which
   * will make it scheduled more consistently.
   */
  static void UpdateThreadPriority();

  /**
   * Utility function for getting motor controller voltage
   *
   * @param controllers A set of motor controllers powering a mechanism.
   * @param controllerNames The names of the motor controllers.
   * @return The average of the measured voltages of the motor controllers.
   */
  static double MeasureVoltage(
      const std::vector<std::unique_ptr<frc::MotorController>>& controllers,
      const std::vector<std::string>& controllerNames);

 protected:
  /**
   * The initial size of the data collection vectors, set to be large enough so
   * that we avoid resizing the vector during data collection. Determined by: 20
   * seconds of test data * 200 samples/second * 9 doubles/sample(320kB of
   * reserved data).
   */
  static constexpr size_t kDataVectorSize = 36000;

  /**
   * The commanded motor voltage. Either as a rate (V/s) for the quasistatic
   * test or as a voltage (V) for the dynamic test.
   */
  double m_voltageCommand = 0.0;

  /**
   * The voltage that the motors should be set to.
   */
  double m_motorVoltage = 0.0;

  /**
   * Keeps track of the current timestamp for data collection purposes.
   */
  double m_timestamp = 0.0;

  /**
   * The timestamp of when the test starts. Mainly used to keep track of the
   * test running for too long.
   */
  double m_startTime = 0.0;

  /**
   * Determines for Drivetrain tests if the robot should be spinning (value sent
   * via NT).
   */
  bool m_rotate = false;

  /**
   * The test that is running (e.g. Quasistatic or Dynamic).
   */
  std::string m_testType;

  /**
   * The mechanism that is being characterized (sent via NT).
   */
  std::string m_mechanism;

  /**
   * Stores all of the collected data.
   */
  std::vector<double> m_data;

  int m_ackNum = 0;

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

  /**
   * Determines if the logger is collecting data for an unsupported mechanism.
   *
   * @returns True if the logger is characterizing an unsupported mechanism
   *          type.
   */
  virtual bool IsWrongMechanism() const = 0;

 private:
  static constexpr int kThreadPriority = 15;
  static constexpr int kHALThreadPriority = 40;
};

}  // namespace sysid
