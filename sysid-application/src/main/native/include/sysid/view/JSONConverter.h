// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include <glass/View.h>
#include <portable-file-dialogs.h>
#include <wpi/Logger.h>

namespace sysid {
class JSONConverter {
 public:
  explicit JSONConverter(wpi::Logger& logger) : m_logger(logger) {}
  /**
   * Function to display the FRC Characterization to SysId JSON converter
   */
  void DisplayFRCCharConvert();

  /**
   * Function to display the SysId JSON to CSV converter.
   */
  void DisplayCSVConvert();

 private:
  /**
   * Helper method to display a specific JSON converter
   *
   * @param tooltip The tooltip describing the JSON converter
   * @param converter The function that takes a filename path and performs the
   *                  previously specifid JSON conversion.
   */
  void DisplayConverter(
      const char* tooltip,
      std::function<std::string(std::string_view, wpi::Logger&)> converter);

  wpi::Logger& m_logger;

  std::string m_location;
  std::unique_ptr<pfd::open_file> m_opener;

  std::string m_exception;

  double m_timestamp = 0;
};
}  // namespace sysid
