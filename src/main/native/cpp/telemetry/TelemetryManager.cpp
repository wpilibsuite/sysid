// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/telemetry/TelemetryManager.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

#include <wpi/raw_ostream.h>
#include <wpi/timestamp.h>

using namespace sysid;

TelemetryManager::TelemetryManager(Settings settings, NT_Inst instance)
    : m_settings(std::move(settings)),
      m_nt(instance),
      m_autospeed(m_nt.GetEntry("/SmartDashboard/SysIdAutoSpeed")),
      m_telemetry(m_nt.GetEntry("/SmartDashboard/SysIdTelemetry")),
      m_fieldInfo(m_nt.GetEntry("/FMSInfo/FMSControlData")) {
  // Add listeners for our readable entries.
  m_nt.AddListener(m_telemetry);
  m_nt.AddListener(m_fieldInfo);
}

void TelemetryManager::BeginTest(wpi::StringRef name) {
  // Create a new test params instance for this test.
  m_params = TestParameters{name.startswith("fast"), name.endswith("forward"),
                            wpi::Now() * 1E-6};

  // Add this test to the list of running tests and set the running flag.
  m_tests.push_back(name);
  m_isRunningTest = true;
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
      func(m_params.data.back()[5], m_params.data.back()[6]);
    } else {
      func(0.0, 0.0);
    }
  }

  // Send a zero command over NT.
  nt::SetEntryValue(m_autospeed, nt::Value::MakeDouble(0.0));
}

void TelemetryManager::Update() {
  // If there is no test running, these is nothing to update.
  if (!m_isRunningTest) {
    return;
  }

  // Poll NT data.
  bool wasEnabled = m_params.enabled;
  for (auto&& event : m_nt.PollListener()) {
    if (event.entry == m_fieldInfo && event.value && event.value->IsDouble()) {
      // Get the FMS Control Word and look at the first bit for enabled state.
      uint32_t controlWord = event.value->GetDouble();
      m_params.enabled = (controlWord & 0x01) != 0 ? true : false;
    } else if (m_params.enabled && event.entry == m_telemetry && event.value &&
               event.value->IsDoubleArray()) {
      // Get the telemetry array and add it to our data (after doing a size
      // check).
      auto data = event.value->GetDoubleArray();
      if (data.size() == 10) {
        std::array<double, 10> d;
        std::copy_n(std::make_move_iterator(data.begin()), 10, d.begin());
        m_params.data.push_back(std::move(d));
      }
    }
  }

  // If the robot wasn't enabled before, but now is, reset the start time.
  if (!wasEnabled && m_params.enabled) {
    m_params.start = wpi::Now() * 1E-6;
  }

  // Set autospeed value if the robot is enabled, otherwise set it to zero for
  // safety.
  if (m_params.enabled) {
    double now = wpi::Now() * 1E-6;
    double volts =
        (m_params.fast
             ? *m_settings.stepVoltage
             : ((now - m_params.start) * *m_settings.quasistaticRampRate)) *
        (m_params.forward ? 1 : -1);
    nt::SetEntryValue(m_autospeed, nt::Value::MakeDouble(volts / 12.0));
  }

  // If the robot was previously enabled, but isn't now, it means that we should
  // cancel the test.
  if (wasEnabled && !m_params.enabled) {
    EndTest();
  }
}

void TelemetryManager::SaveJSON(wpi::StringRef location) {
  // Use the same data for now while things are sorted out.
  m_data["test"] = "Drivetrain";
  m_data["units"] = "Meters";
  m_data["unitsPerRotation"] = 1.0;

  // Get the current date and time. This will be included in the file name.
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);

  std::stringstream ss;
  ss << location;
  ss << "/sysid_data";
  ss << std::put_time(&tm, "%Y%m%d-%H%M");
  ss << ".json";

  std::error_code ec;
  wpi::raw_fd_ostream os{ss.str(), ec};

  if (ec) {
    throw std::runtime_error("Cannot write to file: " + ss.str());
  }

  os << m_data;
  os.flush();
  wpi::outs() << "Wrote JSON to: " << ss.str() << "\n";
}
