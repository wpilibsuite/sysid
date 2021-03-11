// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/JSONConverter.h"
#include "sysid/view/JSONConverter.h"

#include <exception>

#include <imgui.h>
#include <portable-file-dialogs.h>
#include <wpi/timestamp.h>

using namespace sysid;

void JSONConverter::Display() {
  if (ImGui::Button("Select frc-characterization JSON")) {
    m_opener =
        std::make_unique<pfd::open_file>("Select frc-characterization JSON");
  }

  if (m_opener && m_opener->ready()) {
    if (!m_opener->result().empty()) {
      m_location = m_opener->result()[0];
      try {
        sysid::ConvertJSON(m_location, m_logger);
        m_timestamp = wpi::Now() * 1E-6;
      } catch (const std::exception& e) {
        ImGui::OpenPopup("Exception Caught!");
        m_exception = e.what();
      }
    }
    m_opener.reset();
  }

  if (wpi::Now() * 1E-6 - m_timestamp < 5) {
    ImGui::SameLine();
    ImGui::Text("Saved!");
  }

  // Handle exceptions.
  if (ImGui::BeginPopupModal("Exception Caught!")) {
    ImGui::Text(
        "An error occurred when parsing the JSON. This most likely means that "
        "the JSON\ndata is incorrectly formatted.");
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s",
                       m_exception.c_str());
    if (ImGui::Button("Close")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}
