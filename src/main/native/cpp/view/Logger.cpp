// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Logger.h"

#include <exception>

#include <glass/Context.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <ntcore_cpp.h>
#include <units/angle.h>
#include <wpi/math>
#include <wpi/raw_ostream.h>
#include <wpigui.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisType.h"

using namespace sysid;

Logger::Logger() {
  // Add an NT connection listener to update the GUI's state.
  auto instance = nt::GetDefaultInstance();
  auto poller = nt::CreateConnectionListenerPoller(instance);

  nt::AddPolledConnectionListener(poller, true);
  wpi::gui::AddEarlyExecute([poller, &connected = m_ntConnected] {
    bool timedOut;
    for (auto&& event : nt::PollConnectionListener(poller, 0, &timedOut)) {
      connected = event.connected;
    }
  });

  // Initialize team number from storage.
  m_team = glass::GetStorage().GetIntRef("Team");
}

void Logger::Display() {
  // Get the current width of the window. This will be used to scale
  // our UI elements.
  float width = ImGui::GetContentRegionAvail().x;

  // Add team number input and apply button for NT connection.
  ImGui::SetNextItemWidth(width / 5);
  ImGui::InputInt("Team #", m_team, 0);

  if (ImGui::Button("Apply")) {
    m_ntReset = true;
  }

  // Reset and clear the internal manager state.
  ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    m_manager =
        std::make_unique<TelemetryManager>(TelemetryManager::Settings{});
    m_selectedType = 0;
  }

  // Add NT connection indicator.
  static ImVec4 kColorDisconnected{1.0f, 0.4f, 0.4f, 1.0f};
  static ImVec4 kColorConnected{0.2f, 1.0f, 0.2f, 1.0f};
  ImGui::SameLine();
  ImGui::TextColored(m_ntConnected ? kColorConnected : kColorDisconnected,
                     m_ntConnected ? "NT Connected" : "NT Disconnected");

  // Create a Section for project configuration
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Project Parameters");

  // Add a dropdown for mechanism type.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);

  if (ImGui::Combo("Mechanism", &m_selectedType, kTypes,
                   IM_ARRAYSIZE(kTypes))) {
    m_settings.mechanism = analysis::FromName(kTypes[m_selectedType]);
  }

  // Add Dropdown for Units
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
  if (ImGui::Combo("Unit Type", &m_selectedUnit, kUnits,
                   IM_ARRAYSIZE(kUnits))) {
    m_settings.units = kUnits[m_selectedUnit];
  }

  sysid::CreateTooltip(
      "This is the type of units that your robot will be using."
      "For example, if you want your flywheel gains in terms of radians, then "
      "use the "
      "radians unit."
      "On the other hand, if your drivetrain will use gains in meters, choose "
      "meters.");

  // Add Units Per Rotations entry

  // Rotational units have fixed Units per rotations
  m_isRotationalUnits =
      (m_settings.units == "Rotations" || m_settings.units == "Degrees" ||
       m_settings.units == "Radians");
  if (m_settings.units == "Degrees") {
    m_settings.unitsPerRotation = 360.0;
  } else if (m_settings.units == "Radians") {
    m_settings.unitsPerRotation = 2 * wpi::math::pi;
  } else if (m_settings.units == "Rotations") {
    m_settings.unitsPerRotation = 1.0;
  }

  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
  ImGui::InputDouble("Units Per Rotation", &m_settings.unitsPerRotation, 0.0f,
                     0.0f, "%.4f",
                     m_isRotationalUnits ? ImGuiInputTextFlags_ReadOnly
                                         : ImGuiInputTextFlags_None);
  sysid::CreateTooltip(
      "The logger assumes that the code will be sending recorded rotations "
      "over Network Tables."
      "This value will then be multiplied by the units per rotation to get the "
      "measurement in the units you specified.");
  // Create a section for voltage parameters.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Voltage Parameters");

  auto CreateVoltageParameters = [this](const char* text, double* data,
                                        float min, float max) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled,
                        m_manager && m_manager->IsActive());
    float value = static_cast<float>(*data);
    if (ImGui::SliderFloat(text, &value, min, max, "%.2f")) {
      *data = value;
    }
    ImGui::PopItemFlag();
  };

  CreateVoltageParameters("Quasistatic Ramp Rate (V/s)",
                          &m_settings.quasistaticRampRate, 0.10f, 0.60f);
  sysid::CreateTooltip(
      "This is the rate at which the voltage will increase "
      "during the quasistatic and track width tests.");

  CreateVoltageParameters("Dynamic Step Voltage (V)", &m_settings.stepVoltage,
                          2.0f, 10.0f);
  sysid::CreateTooltip(
      "This is the voltage that will be applied for the "
      "dynamic voltage (acceleration) tests.");

  // Create a section for tests.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Tests");

  auto CreateTest = [this, width](const char* text, const char* itext) {
    // Display buttons if we have an NT connection.
    if (m_ntConnected) {
      // Create button to run tests.
      if (ImGui::Button(text)) {
        // Open the warning message.
        ImGui::OpenPopup("Warning");
        m_manager->BeginTest(itext);
        m_opened = text;
      }
      if (m_opened == text && ImGui::BeginPopupModal("Warning")) {
        if (m_manager->IsActive()) {
          ImGui::Text(
              "Please enable the robot in autonomous mode, and then "
              "disable it "
              "before it runs out of space. \n Note: The robot will "
              "continue "
              "to move until you disable it - It is your "
              "responsibility to "
              "ensure it does not hit anything!");
        } else {
          ImGui::Text(
              "The primary encoder has reported: %.3f %s.\n"
              "The secondary encoder has reported: %.3f %s.\n"
              "The gyro has reported: %.3f degrees.\n",
              m_primaryEncoder, m_settings.units.c_str(), m_secondaryEncoder,
              m_settings.units.c_str(), m_gyro);
        }

        const char* button = m_manager->IsActive() ? "End Test" : "Close";
        if (ImGui::Button(button)) {
          m_manager->EndTest();
          ImGui::CloseCurrentPopup();
          m_opened = "";
        }
        ImGui::EndPopup();
      }
    } else {
      // Show disabled text when there is no connection.
      ImGui::TextDisabled("%s", text);
    }

    // Show whether the tests were run or not.
    bool run = m_manager->HasRunTest(itext);
    ImGui::SameLine(width * 0.7);
    ImGui::Text(run ? "Run" : "Not Run");
  };

  CreateTest("Quasistatic Forward", "slow-forward");
  CreateTest("Quasistatic Backward", "slow-backward");
  CreateTest("Dynamic Forward", "fast-forward");
  CreateTest("Dynamic Backward", "fast-backward");
  CreateTest("Track Width", "track-width");

  m_manager->RegisterCancellationCallback(
      [&](double primary, double secondary, double gyro) {
        m_primaryEncoder = primary * m_settings.unitsPerRotation;
        m_secondaryEncoder = secondary * m_settings.unitsPerRotation;
        m_gyro = units::convert<units::radian, units::degree>(gyro);
      });

  // Display the path to where the JSON will be saved and a button to select the
  // location.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Save Location");
  if (ImGui::Button("Choose")) {
    m_selector = std::make_unique<pfd::select_folder>("Select Folder");
  }
  ImGui::SameLine();
  ImGui::InputText("##savelocation", &m_jsonLocation,
                   ImGuiInputTextFlags_ReadOnly);

  // Add button to save.
  ImGui::SameLine(width * 0.9);
  if (ImGui::Button("Save")) {
    try {
      m_manager->SaveJSON(m_jsonLocation);
    } catch (const std::exception& e) {
      ImGui::OpenPopup("Exception Caught!");
      m_exception = e.what();
    }
  }

  // Handle exceptions.
  if (ImGui::BeginPopupModal("Exception Caught!")) {
    ImGui::Text("%s", m_exception.c_str());
    if (ImGui::Button("Close")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  // Run periodic methods.
  SelectDataFolder();
  CheckNTReset();
  m_manager->Update();
}

void Logger::SelectDataFolder() {
  // If the selector exists and is ready with a result, we can store it.
  if (m_selector && m_selector->ready()) {
    m_jsonLocation = m_selector->result();
    m_selector.reset();
  }
}

void Logger::CheckNTReset() {
  if (m_ntReset) {
    // Reset the flag and stop the currently running client.
    m_ntReset = false;
    nt::StopClient(nt::GetDefaultInstance());

    // Start a new client with localhost (if team == 0) or the team number.
    if (*m_team == 0) {
      nt::StartClient(nt::GetDefaultInstance(), "localhost", NT_DEFAULT_PORT);
    } else {
      nt::StartClientTeam(nt::GetDefaultInstance(), *m_team, NT_DEFAULT_PORT);
    }
  }
}
