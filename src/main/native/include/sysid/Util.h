// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>

#ifdef _WIN32
#define SYSID_PATH_SEPARATOR "\\"
#else
#define SYSID_PATH_SEPARATOR "/"
#endif

namespace sysid {
void CreateTooltip(const char* text);
std::vector<std::string> Split(const std::string& s, char c);
}  // namespace sysid
