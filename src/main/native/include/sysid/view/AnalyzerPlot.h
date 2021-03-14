// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include <implot.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/Logger.h>
#include <wpi/StringMap.h>
#include <wpi/spinlock.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedforwardAnalysis.h"

namespace sysid {
/**
 * Class that helps with plotting data in the analyzer view.
 */
class AnalyzerPlot {
 public:
  // The chart titles of the plots that we wil create.
  static constexpr const char* kChartTitles[] = {
      "Quasistatic Velocity vs. Velocity-Portion Voltage",
      "Dynamic Acceleration vs. Acceleration-Portion Voltage",
      "Quasistatic Velocity vs. Time",
      "Quasistatic Acceleration vs. Time",
      "Dynamic Velocity vs. Time",
      "Dynamic Acceleration vs. Time"};

  /**
   * Constructs an instance of the analyzer plot helper and allocates memory for
   * all data vectors.
   */
  explicit AnalyzerPlot(wpi::Logger& logger);

  /**
   * Sets the raw data to be displayed on the plots.
   */
  void SetData(const Storage& data, const std::vector<double>& ff,
               AnalysisType type);

  /**
   * Displays voltage-domain plots.
   */
  void DisplayVoltageDomainPlots();

  /**
   * Displays time-domain plots.
   */
  void DisplayTimeDomainPlots();

 private:
  // The maximum size of each vector (dataset to plot).
  static constexpr size_t kMaxSize = 2048;

  // Stores ImPlotPoint vectors for all of the data.
  wpi::StringMap<std::vector<ImPlotPoint>> m_data;

  // Stores points for the lines of best fit.
  ImPlotPoint m_KvFit[2];
  ImPlotPoint m_KaFit[2];

  // Stores points for simulated time-domain data.
  std::vector<std::vector<ImPlotPoint>> m_sim;

  // Thread safety
  wpi::spinlock m_mutex;

  // Logger
  wpi::Logger& m_logger;

  // Stores whether this was the first call to Plot() since setting data.
  bool m_fitVoltageDomainPlots = false;
  bool m_fitTimeDomainPlots = false;
};
}  // namespace sysid
