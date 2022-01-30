// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/Util.h"

#include <stdexcept>

#include <imgui.h>
#include <wpi/fs.h>
#include <wpi/raw_ostream.h>

void sysid::CreateTooltip(const char* text) {
  ImGui::SameLine();
  ImGui::TextDisabled(" (?)");

  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void sysid::CreateErrorPopup(bool& isError, std::string_view errorMessage) {
  if (isError) {
    ImGui::OpenPopup("Exception Caught!");
  }

  // Handle exceptions.
  ImGui::SetNextWindowSize(ImVec2(480.f, 0.0f));
  if (ImGui::BeginPopupModal("Exception Caught!")) {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s",
                       errorMessage.data());
    ImGui::PopTextWrapPos();
    if (ImGui::Button("Close")) {
      ImGui::CloseCurrentPopup();
      isError = false;
    }
    ImGui::EndPopup();
  }
}

std::string_view sysid::GetAbbreviation(std::string_view unit) {
  if (unit == "Meters") {
    return "m";
  } else if (unit == "Feet") {
    return "ft";
  } else if (unit == "Inches") {
    return "in";
  } else if (unit == "Radians") {
    return "rad";
  } else if (unit == "Degrees") {
    return "deg";
  } else if (unit == "Rotations") {
    return "rot";
  } else {
    throw std::runtime_error("Invalid Unit");
  }
}

void sysid::SaveFile(std::string_view contents, const fs::path& path) {
  // Create the path if it doesn't already exist.
  fs::create_directories(path.root_directory());

  // Open a fd_ostream to write to file.
  std::error_code ec;
  wpi::raw_fd_ostream ostream{path.string(), ec};

  // Check error code.
  if (ec) {
    throw std::runtime_error("Cannot write to file: " + ec.message());
  }

  // Write contents.
  ostream << contents;
}
