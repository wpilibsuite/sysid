// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryManager.h"

#include <algorithm>
#include <cctype>
#include <ctime>
#include <stdexcept>
#include <string>
#include <utility>

#include <fmt/chrono.h>
#include <ntcore_cpp.h>
#include <wpi/Logger.h>
#include <wpi/SmallVector.h>
#include <wpi/StringExtras.h>
#include <wpi/numbers>
#include <wpi/raw_ostream.h>
#include <wpi/timestamp.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisType.h"

using namespace sysid;

TelemetryManager::TelemetryManager(const Settings& settings,
                                   wpi::Logger& logger, NT_Inst instance)
    : m_settings(settings),
      m_logger(logger),
      m_inst(instance),
      m_poller(nt::CreateEntryListenerPoller(m_inst)),
      m_voltageCommand(
          nt::GetEntry(m_inst, "/SmartDashboard/SysIdVoltageCommand")),
      m_testType(nt::GetEntry(m_inst, "/SmartDashboard/SysIdTestType")),
      m_rotate(nt::GetEntry(m_inst, "/SmartDashboard/SysIdRotate")),
      m_telemetry(nt::GetEntry(m_inst, "/SmartDashboard/SysIdTelemetry")),
      m_overflow(nt::GetEntry(m_inst, "/SmartDashboard/SysIdOverflow")),
      m_telemetryOld(nt::GetEntry(m_inst, "/robot/telemetry")),
      m_mechanism(nt::GetEntry(m_inst, "/SmartDashboard/SysIdTest")),
      m_mechError(nt::GetEntry(m_inst, "/SmartDashboard/SysIdWrongMech")),
      m_fieldInfo(nt::GetEntry(m_inst, "/FMSInfo/FMSControlData")) {
  // Add listeners for our readable entries.
  nt::AddPolledEntryListener(m_poller, m_telemetry, kNTFlags);
  nt::AddPolledEntryListener(m_poller, m_overflow, kNTFlags);
  nt::AddPolledEntryListener(m_poller, m_mechError, kNTFlags);
  nt::AddPolledEntryListener(m_poller, m_fieldInfo, kNTFlags);
  nt::AddPolledEntryListener(m_poller, m_telemetryOld,
                             NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
}

TelemetryManager::~TelemetryManager() {
  nt::DestroyEntryListenerPoller(m_poller);
}

void TelemetryManager::BeginTest(std::string_view name) {
  // Create a new test params instance for this test.
  m_params = TestParameters{
      wpi::starts_with(name, "fast"), wpi::ends_with(name, "forward"),
      m_settings.mechanism == analysis::kDrivetrainAngular,
      State::WaitingForEnable};

  // Add this test to the list of running tests and set the running flag.
  m_tests.push_back(std::string{name});
  m_isRunningTest = true;

  // Set the Voltage Command Entry
  nt::SetEntryValue(
      m_voltageCommand,
      nt::Value::MakeDouble((m_params.fast ? m_settings.stepVoltage
                                           : m_settings.quasistaticRampRate) *
                            (m_params.forward ? 1 : -1)));

  // Set the test type
  nt::SetEntryValue(m_testType, nt::Value::MakeString(
                                    m_params.fast ? "Dynamic" : "Quasistatic"));

  // Set the rotate entry
  nt::SetEntryValue(m_rotate, nt::Value::MakeBoolean(m_params.rotate));

  // Set the current mechanism in NT.
  nt::SetEntryValue(m_mechanism,
                    nt::Value::MakeString(m_settings.mechanism.name));
  // Clear the telemetry entry
  nt::SetEntryValue(m_telemetry, nt::Value::MakeString(""));
  // Set Overflow to False
  nt::SetEntryValue(m_overflow, nt::Value::MakeBoolean(false));
  // Set Mechanism Error to False
  nt::SetEntryValue(m_mechError, nt::Value::MakeBoolean(false));
  nt::Flush(m_inst);

  // Display the warning message.
  for (auto&& func : m_callbacks) {
    func(
        "Please enable the robot in autonomous mode, and then "
        "disable it "
        "before it runs out of space. \n Note: The robot will "
        "continue "
        "to move until you disable it - It is your "
        "responsibility to "
        "ensure it does not hit anything!");
  }

  WPI_INFO(m_logger, "Started {} test.", m_tests.back());
}

void TelemetryManager::EndTest() {
  // If there is no test running, this is a no-op
  if (!m_isRunningTest) {
    return;
  }

  // Disable the running flag and store the data in the JSON.
  m_isRunningTest = false;
  m_data[m_tests.back()] = m_params.data;

  // Call the cancellation callbacks.
  for (auto&& func : m_callbacks) {
    std::string msg;
    if (m_params.mechError) {
      msg +=
          "\nERROR: The robot indicated that you are using the wrong project "
          "for characterizing your mechanism. \nThis most likely means you "
          "are trying to characterize a mechanism like a Drivetrain with a "
          "deployed config for a General Mechanism (e.g. Arm, Flywheel, and "
          "Elevator) or vice versa. Please double check your settings and "
          "try again.";
    } else if (!m_params.data.empty()) {
      std::string units = m_settings.units;
      std::transform(m_settings.units.begin(), m_settings.units.end(),
                     units.begin(), ::tolower);

      if (wpi::starts_with(m_settings.mechanism.name, "Drivetrain")) {
        double p = (m_params.data.back()[3] - m_params.data.front()[3]) *
                   m_settings.unitsPerRotation;
        double s = (m_params.data.back()[4] - m_params.data.front()[4]) *
                   m_settings.unitsPerRotation;
        double g = m_params.data.back()[7] - m_params.data.front()[7];

        msg = fmt::format(
            "The left and right encoders traveled {} {} and {} {} "
            "respectively.\nThe gyro angle delta was {} degrees.",
            p, units, s, units, g * 180.0 / wpi::numbers::pi);
      } else {
        double p = (m_params.data.back()[2] - m_params.data.front()[2]) *
                   m_settings.unitsPerRotation;
        msg = fmt::format("The encoder reported traveling {} {}.", p, units);
      }

      if (m_params.overflow) {
        msg +=
            "\nNOTE: the robot stopped recording data early because the entry "
            "storage was exceeded.";
      }
    } else {
      msg = "No data was detected.";
    }
    func(msg);
  }

  // Remove previously run test from list of tests if no data was detected.
  if (m_params.data.empty()) {
    m_tests.pop_back();
  }

  // Send a zero command over NT.
  nt::SetEntryValue(m_voltageCommand, nt::Value::MakeDouble(0.0));
  nt::Flush(m_inst);
}

void TelemetryManager::Update() {
  // If there is no test running, these is nothing to update.
  if (!m_isRunningTest) {
    return;
  }

  // Update the NT entries that we're reading.
  bool timedOut = false;
  for (auto&& event : nt::PollEntryListener(m_poller, 0, &timedOut)) {
    // Get the FMS Control Word.
    if (event.entry == m_fieldInfo && event.value && event.value->IsDouble()) {
      uint32_t ctrl = event.value->GetDouble();
      m_params.enabled = ctrl & 0x01;
    }
    // Get the string in the data field.
    if (event.entry == m_telemetry && event.value && event.value->IsString()) {
      std::string value{event.value->GetString()};
      if (!value.empty()) {
        m_params.raw = std::move(value);
        nt::SetEntryValue(m_telemetry, nt::Value::MakeString(""));
      }
    }
    // Get the overflow flag
    if (event.entry == m_overflow && event.value && event.value->IsBoolean()) {
      m_params.overflow = event.value->GetBoolean();
    }
    // Get the mechanism error flag
    if (event.entry == m_mechError && event.value && event.value->IsBoolean()) {
      m_params.mechError = event.value->GetBoolean();
    }

    // Check if we got frc-characterization data.
    if (event.entry == m_telemetryOld) {
      for (auto&& func : m_callbacks) {
        func(
            "Detected data over frc-characterization NT entry.\nPlease ensure "
            "that you are sending data over the sysid NT entries as described "
            "in the documentation.");
        EndTest();
      }
    }
  }

  // Go through our state machine.
  if (m_params.state == State::WaitingForEnable) {
    if (m_params.enabled) {
      m_params.enableStart = wpi::Now() * 1E-6;
      m_params.state = State::RunningTest;
      WPI_INFO(m_logger, "{}", "Transitioned to running test state.");
    }
  }

  if (m_params.state == State::RunningTest) {
    // If for some reason we've disconnected, end the test.
    if (!nt::IsConnected(m_inst)) {
      WPI_WARNING(m_logger, "{}",
                  "NT connection was dropped when executing the test. The test "
                  "has been canceled.");
      EndTest();
    }

    // If the robot has disabled, then we can move on to the next step.
    if (!m_params.enabled) {
      m_params.disableStart = wpi::Now() * 1E-6;
      m_params.state = State::WaitingForData;
    }
  }

  if (m_params.state == State::WaitingForData) {
    double now = wpi::Now() * 1E-6;
    nt::SetEntryValue(m_voltageCommand, nt::Value::MakeDouble(0.0));
    nt::Flush(m_inst);

    // We have the data that we need, so we can parse it and end the test.
    if (!m_params.raw.empty()) {
      // Clean up the string -- remove spaces if there are any.
      m_params.raw.erase(
          std::remove_if(m_params.raw.begin(), m_params.raw.end(), ::isspace),
          m_params.raw.end());

      // Split the string into individual components.
      wpi::SmallVector<std::string_view, 16> res;
      wpi::split(m_params.raw, res, ',');

      // Convert each string to double.
      std::vector<double> values;
      values.reserve(res.size());
      for (auto&& str : res) {
        values.push_back(wpi::parse_float<double>(str).value());
      }

      // Add the values to our result vector.
      for (size_t i = 0; i < values.size() - m_settings.mechanism.rawDataSize;
           i += m_settings.mechanism.rawDataSize) {
        std::vector<double> d(m_settings.mechanism.rawDataSize);

        std::copy_n(std::make_move_iterator(values.begin() + i),
                    m_settings.mechanism.rawDataSize, d.begin());
        m_params.data.push_back(std::move(d));
      }

      WPI_INFO(m_logger,
               "Received data with size: {} for the {} test in {} seconds.",
               m_params.data.size(), m_tests.back(),
               m_params.data.back()[0] - m_params.data.front()[0]);
      EndTest();
    }

    // If we timed out, end the test and let the user know.
    if (now - m_params.disableStart > 5) {
      WPI_WARNING(m_logger, "{}",
                  "TelemetryManager did not receieve data 5 seconds after "
                  "completing the test...");
      EndTest();
    }
  }
}

std::string TelemetryManager::SaveJSON(std::string_view location) {
  m_data["test"] = m_settings.mechanism.name;
  m_data["units"] = m_settings.units;
  m_data["unitsPerRotation"] = m_settings.unitsPerRotation;
  m_data["sysid"] = true;

  std::string loc = fmt::format("{}/sysid_data{:%Y%m%d-%H%M%S}.json", location,
                                fmt::localtime(std::time(nullptr)));

  sysid::SaveFile(m_data.dump(2), fs::path{loc});
  WPI_INFO(m_logger, "Wrote JSON to: {}", loc);

  return loc;
}
