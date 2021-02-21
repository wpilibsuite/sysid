// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryManager.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <ntcore_cpp.h>
#include <wpi/math>
#include <wpi/raw_ostream.h>
#include <wpi/timestamp.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisType.h"

using namespace sysid;

TelemetryManager::TelemetryManager(const Settings& settings, NT_Inst instance)
    : m_settings(settings),
      m_inst(instance),
      m_poller(nt::CreateEntryListenerPoller(m_inst)),
      m_autospeed(nt::GetEntry(m_inst, "/SmartDashboard/SysIdAutoSpeed")),
      m_rotate(nt::GetEntry(m_inst, "/SmartDashboard/SysIdRotate")),
      m_telemetry(nt::GetEntry(m_inst, "/SmartDashboard/SysIdTelemetry")),
      m_fieldInfo(nt::GetEntry(m_inst, "/FMSInfo/FMSControlData")) {
  // Add listeners for our readable entries.

  nt::AddPolledEntryListener(m_poller, m_telemetry, kNTFlags);
  nt::AddPolledEntryListener(m_poller, m_fieldInfo, kNTFlags);
}

TelemetryManager::~TelemetryManager() {
  nt::DestroyEntryListenerPoller(m_poller);
}

void TelemetryManager::BeginTest(wpi::StringRef name) {
  // Create a new test params instance for this test.
  m_params = TestParameters{name.startswith("fast"), name.endswith("forward"),
                            name.startswith("track"), State::WaitingForEnable};

  // Add this test to the list of running tests and set the running flag.
  m_tests.push_back(name);
  m_isRunningTest = true;

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
      double p = m_params.data.back()[5] - m_params.data.front()[5];
      double s = m_params.data.back()[6] - m_params.data.front()[6];
      double g = m_params.data.back()[9] - m_params.data.front()[9];

      std::stringstream stream;
      std::string units = wpi::StringRef(m_settings.units).lower();

      if (m_settings.mechanism == analysis::kDrivetrain) {
        stream << "The left and right encoders traveled " << p << " " << units
               << " and " << s << " " << units
               << " respectively.\nThe gyro angle delta was "
               << g * 2 * wpi::math::pi << " degrees.";
      } else {
        stream << "The encoder reported traveling " << p << " " << units << ".";
      }
      func(stream.str());
    }
  }

  // Send a zero command over NT.
  nt::SetEntryValue(m_autospeed, nt::Value::MakeDouble(0.0));
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
      }
    }
  }

  // Go through our state machine.
  if (m_params.state == State::WaitingForEnable) {
    if (m_params.enabled) {
      m_params.enableStart = wpi::Now() * 1E-6;
      m_params.state = State::RunningTest;
    }
  }

  if (m_params.state == State::RunningTest) {
    double qrv = m_settings.quasistaticRampRate *
                 (wpi::Now() * 1E-6 - m_params.enableStart);
    double spd = (m_params.fast ? m_settings.stepVoltage : qrv) *
                 (m_params.forward ? 1 : -1);

    nt::SetEntryValue(m_autospeed, nt::Value::MakeDouble(spd / 12.0));
    nt::SetEntryValue(m_rotate, nt::Value::MakeBoolean(m_params.rotate));
    nt::Flush(m_inst);

    // If for some reason we've disconnected, end the test.
    if (!nt::IsConnected(m_inst)) {
      wpi::outs() << "NT connection dropped while executing test...";
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
    nt::SetEntryValue(m_autospeed, nt::Value::MakeDouble(0.0));
    nt::Flush(m_inst);

    // We have the data that we need, so we can parse it and end the test.
    if (!m_params.raw.empty()) {
      // Split the string into individual components.
      auto res = sysid::Split(m_params.raw, ',');

      // Convert each string to double.
      std::vector<double> values;
      values.reserve(res.size());
      for (auto&& str : res) {
        values.push_back(std::stod(str));
      }

      // Add the values to our result vector.
      for (size_t i = 0; i < values.size(); i += 10) {
        std::array<double, 10> d;
        std::copy_n(std::make_move_iterator(values.begin() + i), 10, d.begin());
        m_params.data.push_back(std::move(d));
      }

      wpi::outs() << "Received data with size: " << m_params.data.size()
                  << " for the " << m_tests.back() << " test.\n";
      EndTest();
    }

    // If we timed out, end the test and let the user know.
    if (now - m_params.disableStart > 5) {
      wpi::outs() << "TelemetryManager did not receieve data 5 seconds after "
                     "completing the test...";
      EndTest();
    }
  }
}

std::string TelemetryManager::SaveJSON(wpi::StringRef location) {
  // Use the same data for now while things are sorted out.
  m_data["test"] = m_settings.mechanism.name;
  m_data["units"] = m_settings.units;
  m_data["unitsPerRotation"] = m_settings.unitsPerRotation;

  // Get the current date and time. This will be included in the file name.
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream ss;
  ss << location;
  ss << SYSID_PATH_SEPARATOR;
  ss << "sysid_data";
  ss << std::put_time(&tm, "%Y%m%d-%H%M");
  ss << ".json";

  std::string loc = ss.str();

  std::error_code ec;
  wpi::raw_fd_ostream os{loc, ec};

  if (ec) {
    throw std::runtime_error("Cannot write to file: " + loc);
  }

  os << m_data;
  os.flush();
  wpi::outs() << "Wrote JSON to: " << loc << "\n";

  return loc;
}
