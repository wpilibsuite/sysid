// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <wpi/Logger.h>
#include <wpi/StringRef.h>

namespace sysid {
/**
 * Converts a JSON from the old frc-characterization format to the new sysid
 * format.
 *
 * @param path   The path to the old JSON.
 * @param logger The logger instance for log messages.
 *
 * @return The full file path of the newly saved JSON.
 */
std::string ConvertJSON(wpi::StringRef path, wpi::Logger& logger);

std::string ToCSV(wpi::StringRef path, wpi::Logger& logger);
}  // namespace sysid
