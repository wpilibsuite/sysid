// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/JSONConverter.h"
#include "sysid/view/JSONConverter.h"

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
      sysid::ConvertJSON(m_location, m_logger);
      m_timestamp = wpi::Now() * 1E-6;
    }
    m_opener.reset();
  }

  if (wpi::Now() * 1E-6 - m_timestamp < 5) {
    ImGui::SameLine();
    ImGui::Text("Saved!");
  }
}
