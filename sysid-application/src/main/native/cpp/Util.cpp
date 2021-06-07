// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/Util.h"

#include <sstream>
#include <stdexcept>
#include <vector>

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

std::vector<std::string> sysid::Split(const std::string& s, char c) {
  std::vector<std::string> result;
  std::stringstream ss{s};
  std::string item;

  while (std::getline(ss, item, c)) {
    result.push_back(item);
  }

  return result;
}

std::string sysid::GetAbbreviation(std::string_view unit) {
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
