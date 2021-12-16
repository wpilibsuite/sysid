// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string_view>
#include <utility>
#include <vector>

namespace sysid {
std::vector<std::pair<const char*, std::string_view>> GetLibrariesToDeploy();
}  // namespace sysid
