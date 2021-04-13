// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryManager.h"

#include <algorithm>
#include <cctype>
#include <ctime>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <ntcore_cpp.h>
#include <wpi/Logger.h>
#include <wpi/math>
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
      m_telemetryOld(nt::GetEntry(m_inst, "/robot/telemetry")),
      m_mechanism(nt::GetEntry(m_inst, "/SmartDashboard/SysIdTest")),
      m_fieldInfo(nt::GetEntry(m_inst, "/FMSInfo/FMSControlData")) {
  // Add listeners for our readable entries.
  nt::AddPolledEntryListener(m_poller, m_telemetry, kNTFlags);
  nt::AddPolledEntryListener(m_poller, m_fieldInfo, kNTFlags);
  nt::AddPolledEntryListener(m_poller, m_telemetryOld,
                             NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
}

TelemetryManager::~TelemetryManager() {
  nt::DestroyEntryListenerPoller(m_poller);
}

void TelemetryManager::BeginTest(wpi::StringRef name) {
  // Create a new test params instance for this test.
  m_params =
      TestParameters{name.startswith("fast"), name.endswith("forward"),
                     m_settings.mechanism == analysis::kDrivetrainAngular,
                     State::WaitingForEnable};

  // Add this test to the list of running tests and set the running flag.
  m_tests.push_back(name);
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

  WPI_DEBUG(m_logger, "Started " << m_tests.back() << "test");
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
    if (!m_params.data.empty()) {
      std::stringstream stream;
      std::string units = wpi::StringRef(m_settings.units).lower();

      if (wpi::StringRef(m_settings.mechanism.name).startswith("Drivetrain")) {
        double p = (m_params.data.back()[3] - m_params.data.front()[3]) *
                   m_settings.unitsPerRotation;
        double s = (m_params.data.back()[4] - m_params.data.front()[4]) *
                   m_settings.unitsPerRotation;
        double g = m_params.data.back()[7] - m_params.data.front()[7];
        stream << "The left and right encoders traveled " << p << " " << units
               << " and " << s << " " << units
               << " respectively.\nThe gyro angle delta was "
               << g * 180.0 / wpi::math::pi << " degrees.";
      } else {
        double p = (m_params.data.back()[2] - m_params.data.front()[2]) *
                   m_settings.unitsPerRotation;
        stream << "The encoder reported traveling " << p << " " << units << ".";
      }
      func(stream.str());
    }
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
      std::string value = event.value->GetString();
      if (!value.empty()) {
        m_params.raw = std::move(value);
        nt::SetEntryValue(m_telemetry, nt::Value::MakeString(""));
      }
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
      WPI_DEBUG(m_logger, "Transitioned to running test state.");
    }
  }

  if (m_params.state == State::RunningTest) {
    // If for some reason we've disconnected, end the test.
    if (!nt::IsConnected(m_inst)) {
      WPI_WARNING(m_logger,
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
      auto res = sysid::Split(m_params.raw, ',');

      // Convert each string to double.
      std::vector<double> values;
      values.reserve(res.size());
      for (auto&& str : res) {
        values.push_back(std::stod(str));
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
               "Received data with size: "
                   << m_params.data.size() << " for the " << m_tests.back()
                   << " test in "
                   << m_params.data.back()[0] - m_params.data.front()[0]
                   << " seconds.");
      EndTest();
    }

    // If we timed out, end the test and let the user know.
    if (now - m_params.disableStart > 5) {
      WPI_WARNING(m_logger,
                  "TelemetryManager did not receieve data 5 seconds after "
                  "completing the test...");
      EndTest();
    }
  }
}

std::string TelemetryManager::SaveJSON(wpi::StringRef location) {
  m_data["test"] = m_settings.mechanism.name;
  m_data["units"] = m_settings.units;
  m_data["unitsPerRotation"] = m_settings.unitsPerRotation;
  m_data["sysid"] = true;

  // Get the current date and time. This will be included in the file name.
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream ss;
  ss << location;
  ss << SYSID_PATH_SEPARATOR;
  ss << "sysid_data";
  ss << std::put_time(&tm, "%Y%m%d-%H%M%S");
  ss << ".json";

  std::string loc = ss.str();

  std::error_code ec;
  wpi::raw_fd_ostream os{loc, ec};

  if (ec) {
    throw std::runtime_error("Cannot write to file: " + loc);
  }

  os << m_data;
  os.flush();
  WPI_INFO(m_logger, "Wrote JSON to: " << loc);

  return loc;
}
