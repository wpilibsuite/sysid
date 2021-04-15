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

// The generated AppleScript by portable-file-dialogs for just *.json does not
// work correctly because it is a uniform type identifier. This means that
// "public." needs to be prepended to it.
#ifdef __APPLE__
#define SYSID_PFD_JSON_EXT "*.public.json"
#else
#define SYSID_PFD_JSON_EXT "*.json"
#endif

namespace sysid {
void CreateTooltip(const char* text);
std::vector<std::string> Split(const std::string& s, char c);
std::string GetAbbreviation(std::string unit);
}  // namespace sysid
