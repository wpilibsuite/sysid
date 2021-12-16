#pragma once

#include <string_view>
#include <vector>

namespace sysid {
std::vector<std::pair<const char*, std::string_view>> GetLibrariesToDeploy();
}
