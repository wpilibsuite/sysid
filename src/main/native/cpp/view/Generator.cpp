// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Generator.h"

#include <iostream>
#include <memory>
#include <string>

#include <glass/Context.h>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <wpi/StringRef.h>
#include <wpi/math>
#include <wpi/raw_os_ostream.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/generation/ConfigManager.h"

using namespace sysid;

Generator::Generator(wpi::Logger& logger) : m_logger(logger) {
  m_manager = std::make_unique<ConfigManager>(m_settings, m_logger);
  // Initialize persistent storage and assign pointers.
  auto& storage = glass::GetStorage();
  m_pUnitsPerRotation = storage.GetDoubleRef("Units Per Rotation", 1.0);
  m_pAnalysisType = storage.GetStringRef("Analysis Type", "Simple");
}

void Generator::GeneratorUI() {
  // Add section for motor and motor controller selection.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Motor / Motor Controller Selection");

  // +
  ImGui::SameLine();

  if (ImGui::Button("(+)")) {
    ++m_portsCount;
  }

  // -
  if (m_portsCount > 1) {
    ImGui::SameLine();
    if (ImGui::Button("(-)")) {
      --m_portsCount;
    }
  }

  ImGui::Spacing();

  // Add motor port selection.
  bool drive = *m_pAnalysisType == "Drivetrain";
  m_hasBrushed = false;
  for (size_t i = 0; i < m_portsCount; ++i) {
    // Ensure that our vector contains i+1 elements.
    ImGui::Spacing();
    ImGui::PushID(i);
    if (m_settings.m_primaryMotorPorts.size() == i) {
      m_settings.m_primaryMotorPorts.emplace_back((drive ? 2 : 1) * i);
      m_settings.m_secondaryMotorPorts.emplace_back(
          m_settings.m_primaryMotorPorts.size() + i);
      m_settings.m_motorControllers.emplace_back(kMotorControllers[0]);

      m_settings.m_primaryMotorsInverted.emplace_back(false);
      m_settings.m_secondaryMotorsInverted.emplace_back(false);
    }

    // Add primary (left for drivetrain) motor ports.
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    std::string port_name =
        (drive ? "L Motor Port " : "Motor Port ") + std::to_string(i);
    ImGui::InputInt(port_name.c_str(), &m_settings.m_primaryMotorPorts[i], 0,
                    0);
    ImGui::SameLine();
    ImGui::Checkbox(drive ? "L inverted" : "Inverted",
                    &m_settings.m_primaryMotorsInverted[i]);

    // Add secondary (right) motor ports.
    if (drive) {
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      std::string right_port_name = "R Motor Port " + std::to_string(i);
      ImGui::InputInt(right_port_name.c_str(),
                      &m_settings.m_secondaryMotorPorts[i], 0, 0);

      ImGui::SameLine();
      ImGui::Checkbox("R Inverted", &m_settings.m_secondaryMotorsInverted[i]);
    }

    // Add buttons to add and remove ports.

    // Add motor controller selector
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);

    std::string name = "Motor Controller ";
    if (drive) {
      name += "Pair ";
    }
    name += std::to_string(i);

    if (ImGui::BeginCombo(name.c_str(),
                          m_settings.m_motorControllers[i].c_str())) {
      for (int n = 0; n < IM_ARRAYSIZE(kMotorControllers); n++) {
        bool is_selected =
            (m_settings.m_motorControllers[i] == kMotorControllers[n]);
        if (ImGui::Selectable(kMotorControllers[n], is_selected)) {
          m_settings.m_motorControllers[i] = kMotorControllers[n];
        }
        if (is_selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::PopID();
    if (m_settings.m_motorControllers[i] == "SPARK MAX (Brushed)") {
      m_hasBrushed = true;
    }
  }

  // Add section for encoders.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Encoder Selection");
  ImGui::Spacing();

  // Add encoder selection.
  if (m_motorControllerIdx > 0) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
    ImGui::Combo("Encoder", &m_encoderIdx, kEncoders, IM_ARRAYSIZE(kEncoders));
  } else {
    m_encoderIdx = 2;
  }

  // Add encoder port selection if roboRIO is selected.
  if (m_encoderIdx > 1) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("A##1", &m_settings.m_primaryEncoderPorts[0], 0, 0);
    ImGui::SameLine(ImGui::GetFontSize() * 4);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("B##1", &m_settings.m_primaryEncoderPorts[1], 0, 0);
    ImGui::SameLine();
    ImGui::Checkbox(drive ? "Left Encoder inverted" : "Encoder Inverted",
                    &m_settings.m_primaryEncoderInverted);

    // Add another row if we are running drive tests.
    if (drive) {
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      ImGui::InputInt("A##2", &m_settings.m_secondaryEncoderPorts[0], 0, 0);
      ImGui::SameLine(ImGui::GetFontSize() * 4);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      ImGui::InputInt("B##2", &m_settings.m_secondaryEncoderPorts[1], 0, 0);
      ImGui::SameLine();
      ImGui::Checkbox("Right Encoder inverted",
                      &m_settings.m_secondaryEncoderInverted);
    }
    ImGui::Checkbox("Reduce Encoding", &m_settings.m_encoding);
    CreateTooltip(
        "This helps reduce encoder noise for high CPR encoders such as the "
        "CTRE Magnetic Encoder and REV Throughbore Encoder");
  }

  // Add CANCoder port selection.
  if (m_encoderIdx == 1 && m_motorControllerIdx > 0 &&
      m_motorControllerIdx < 4) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt(drive ? "L CANCoder Port" : "CANCoder Port",
                    &m_settings.m_primaryEncoderPorts[0], 0, 0);
    ImGui::Checkbox(drive ? "Left Encoder inverted" : "Encoder Inverted",
                    &m_settings.m_primaryEncoderInverted);
    if (drive) {
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      ImGui::InputInt("R CANCoder Port", &m_settings.m_secondaryEncoderPorts[1],
                      0, 0);
      ImGui::Checkbox("Right Encoder inverted",
                      &m_settings.m_secondaryEncoderInverted);
    }
  }

  // Samples Per Average Setting
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
  ImGui::InputInt("Samples Per Average", &m_settings.m_numSamples, 0, 0);
  CreateTooltip(
      "This helps reduce encoder noise by averaging collected samples "
      "together. A value from 5-10 is reccomended for encoders with high "
      "CPRs.");

  m_settings.m_encoderType = std::string{kEncoders[m_encoderIdx]};

  // Add Velocity Measurement Period
  if (m_encoderIdx <= 1) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    ImGui::Combo("Time Measurement Window", &m_periodIdx, kCTREPeriods,
                 IM_ARRAYSIZE(kCTREPeriods));
    m_settings.m_period = std::stoi(std::string{kCTREPeriods[m_periodIdx]});
  }

  // Add gyro selection if selected is drivetrain.
  if (drive) {
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Gyro");
    ImGui::Spacing();

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::Combo("Gyro", &m_gyroIdx, kGyros, IM_ARRAYSIZE(kGyros));

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);

    wpi::StringRef gyroType{kGyros[m_gyroIdx]};

    if (gyroType == "Pigeon") {
      ImGui::InputInt("Gyro Parameter", &m_gyroPort, 0, 0);
      ImGui::SameLine();
      ImGui::Checkbox("Is Talon", &m_isTalon);
      CreateTooltip(
          "Check this checkbox if the Pigeon is hooked up to a TalonSRX");

      m_settings.m_gyroCtor = std::to_string(m_gyroPort);

      // Indicate Gyro is connected to TalonSRX
      if (m_isTalon) {
        m_settings.m_gyroCtor = "WPI_TalonSRX-" + m_settings.m_gyroCtor;
      }
    } else if (gyroType == "ADXRS450") {
      ImGui::Combo("Gyro Parameter", &m_gyroParam, kADXRS450Ctors,
                   IM_ARRAYSIZE(kADXRS450Ctors));
      m_settings.m_gyroCtor = std::string(kADXRS450Ctors[m_gyroParam]);
    } else if (gyroType == "NavX") {
      ImGui::Combo("Gyro Parameter", &m_gyroParam, kNavXCtors,
                   IM_ARRAYSIZE(kNavXCtors));
      m_settings.m_gyroCtor = std::string(kNavXCtors[m_gyroParam]);
    } else {
      ImGui::InputInt("Gyro Parameter", &m_gyroPort, 0, 0);

      // Analog Gyro Port cannot be greater than 1
      if (m_gyroPort > 1) {
        m_gyroPort = 1;
      }
      m_settings.m_gyroCtor = std::to_string(m_gyroPort);
    }

    // Avoid Accessing Bad Gyro Ctor indexes
    if (gyroType != m_settings.m_gyro) {
      m_gyroParam = 0;
    }

    m_settings.m_gyro = gyroType;
  }

  // Add section for other parameters.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Encoder Parameters");
  ImGui::Spacing();

  // Add encoder resolution.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
  ImGui::InputDouble("Counts Per Revolution", &m_settings.m_cpr, 0.0, 0.0,
                     "%.2f");
  sysid::CreateTooltip(
      "This is the number of encoder counts per revolution for your encoder.\n"
      "Common values for this are here:\nCTRE Magnetic Encoder: 4096\nFalcon "
      "500 Integrated: 2048\nREV Throughbore: 8192\nNEO (and NEO 550) "
      "Integrated "
      "Encoders (REV already handles this value): 1");

  // Add gearing
  ImGui::InputDouble("Gearing", &m_settings.m_gearing, 0.0, 0.0, "%.2f");
  sysid::CreateTooltip(
      "This is the gearing between the encoder and the output shaft. For "
      "example, if the encoder is mounted to the magnetic shaft on the kit "
      "chassis, there is no gearing in between the encoder and the output "
      "shaft hence the gearing is 1. However, if the encoder was an integrated "
      "encoder on the motor in the kit chassis gearbox, the gearing would be "
      "10.71.");
}

void Generator::Display() {
  // Add analysis type input.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
  ImGui::Combo("Analysis Type", &m_analysisIdx, kAnalysisTypes,
               IM_ARRAYSIZE(kAnalysisTypes));
  *m_pAnalysisType = kAnalysisTypes[m_analysisIdx];

  if (*m_pAnalysisType != "Romi") {
    GeneratorUI();
  }

  ImGui::Separator();
  wpi::SmallString<128> path;
  wpi::raw_svector_ostream os{path};
  os << "base_projects/";
  if (*m_pAnalysisType == "General Mechanism") {
    os << "GeneralMechanism";
  } else {
    os << "Drivetrain";
  }
  os << "/";
  if (ImGui::Button("Save Config")) {
    if (*m_pAnalysisType == "Romi") {
      m_settings = kRomiConfig;
    }

    if (m_hasBrushed) {
      ImGui::OpenPopup("Brushed");
    } else {
      m_manager->SaveJSON(path.c_str(), m_portsCount,
                          *m_pAnalysisType == "Romi");
    }
  }
  if (ImGui::BeginPopupModal(
          "Brushed", nullptr,
          ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
    ImGui::Text(
        "Make sure you are using the SparkMax to control a BRUSHED Motor "
        "(a.k.a NOT a NEO or NEO 550).\nIncorrectly setting a brushless motor "
        "to brushed can cause it to permanatly break!\n");
    if (ImGui::Button("Yes")) {
      ImGui::CloseCurrentPopup();
      m_manager->SaveJSON(path.c_str(), m_portsCount);
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}
