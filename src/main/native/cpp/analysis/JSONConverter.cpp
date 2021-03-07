// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/JSONConverter.h"

#include <stdexcept>
#include <string>
#include <system_error>

#include <wpi/Logger.h>
#include <wpi/Twine.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"

// Sizes of the arrays for new sysid data.
static constexpr size_t kDrivetrainSize = 9;
static constexpr size_t kGeneralSize = 4;

// Indices for the old data.
static constexpr size_t kTimestampCol = 0;
static constexpr size_t kLVoltsCol = 3;
static constexpr size_t kRVoltsCol = 4;
static constexpr size_t kLPosCol = 5;
static constexpr size_t kRPosCol = 6;
static constexpr size_t kLVelCol = 7;
static constexpr size_t kRVelCol = 8;

std::string sysid::ConvertJSON(wpi::StringRef path, wpi::Logger& logger) {
  std::error_code ec;
  wpi::raw_fd_istream input{path, ec};

  if (ec) {
    throw std::runtime_error("Unable to read: " + path.str());
  }

  wpi::json ojson;
  input >> ojson;
  WPI_INFO(logger, "Read frc-characterization JSON from " << path);

  auto type = sysid::analysis::FromName(ojson.at("test").get<std::string>());
  auto factor = ojson.at("unitsPerRotation").get<double>();
  auto unit = ojson.at("units").get<std::string>();

  wpi::json json;
  for (auto&& key : AnalysisManager::kJsonDataKeys) {
    if (type == analysis::kDrivetrain) {
      // Get the old data; create a vector for the new data; reserve the
      // appropriate size for the new data.
      auto odata = ojson.at(key).get<std::vector<std::array<double, 10>>>();
      std::vector<std::array<double, kDrivetrainSize>> data;
      data.reserve(odata.size());

      // Transfer the data.
      for (auto&& pt : odata) {
        data.push_back(std::array<double, kDrivetrainSize>{
            pt[kTimestampCol], pt[kLVoltsCol], pt[kRVoltsCol], pt[kLPosCol],
            pt[kRPosCol], pt[kLVelCol], pt[kRVelCol], 0.0, 0.0});
      }
      json[key] = data;
    } else {
      // Get the old data; create a vector for the new data; reserve the
      // appropriate size for the new data.
      auto odata = ojson.at(key).get<std::vector<std::array<double, 10>>>();
      std::vector<std::array<double, kGeneralSize>> data;
      data.reserve(odata.size());

      // Transfer the data.
      for (auto&& pt : odata) {
        data.push_back(std::array<double, kGeneralSize>{
            pt[kTimestampCol], pt[kLVoltsCol], pt[kLPosCol], pt[kLVelCol]});
      }
      json[key] = data;
    }
  }
  json["units"] = unit;
  json["unitsPerRotation"] = factor;
  json["test"] = type.name;
  json["sysid"] = true;

  // Write the new file with "_new" appended to it.
  std::string loc = path.rsplit(".json").first.str() + "_new.json";
  wpi::raw_fd_ostream output{loc, ec};
  output << json;
  output.flush();

  WPI_INFO(logger, "Wrote new JSON to: " << loc);
  return loc;
}
