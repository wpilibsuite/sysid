// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <string_view>
#include <vector>

#include <wpi/fs.h>

// The generated AppleScript by portable-file-dialogs for just *.json does not
// work correctly because it is a uniform type identifier. This means that
// "public." needs to be prepended to it.
#ifdef __APPLE__
#define SYSID_PFD_JSON_EXT "*.public.json"
#else
#define SYSID_PFD_JSON_EXT "*.json"
#endif

#ifdef _WIN32
#define LAUNCH "gradlew"
#else
#define LAUNCH "./gradlew"
#endif

// Based on https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html
#define EXPAND_STRINGIZE(s) STRINGIZE(s)
#define STRINGIZE(s) #s

namespace sysid {
static constexpr const char* kUnits[] = {"Meters",  "Feet",      "Inches",
                                         "Radians", "Rotations", "Degrees"};

/**
 * Displays a tooltip beside the widget that this method is called after with
 * the provided text.
 *
 * @param text The text to show in the tooltip.
 */
void CreateTooltip(const char* text);

/**
 * Splits a string into a vector of strings.
 *
 * @param s The string to split.
 * @param c The delimiter.
 *
 * @return The split string.
 */
std::vector<std::string> Split(const std::string& s, char c);

/**
 * Returns the abbreviation for the unit.
 *
 * @param unit The unit to return the abbreviation for.
 *
 * @return The abbreviation for the unit.
 */
std::string GetAbbreviation(std::string_view unit);

/**
 * Saves a file with the provided contents to a specified location.
 *
 * @param contents The file contents.
 * @param location The file location.
 */
void SaveFile(std::string_view contents, const fs::path& path);
}  // namespace sysid
