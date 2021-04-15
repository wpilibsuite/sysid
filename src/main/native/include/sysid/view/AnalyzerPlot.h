// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <atomic>
#include <string>
#include <vector>

#include <imgui.h>
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
      "Dynamic Acceleration vs. Time",
      "Timesteps vs. Time"};

  // Size of plots when screenshotting
  static constexpr int kCombinedPlotSize = 300;

  /**
   * Constructs an instance of the analyzer plot helper and allocates memory for
   * all data vectors.
   */
  explicit AnalyzerPlot(wpi::Logger& logger);

  /**
   * Sets the raw data to be displayed on the plots.
   *
   * @param rawData      Raw data storage.
   * @param filteredData Filtered data storage.
   * @param ffGains      List of feedforward gains (Ks, Kv, Ka, and optionally
   *                     either Kg or Kcos).
   * @param startTimes   Array of dataset start times.
   * @param type         Type of analysis.
   * @param abort        Aborts analysis early if set to true from another
   *                     thread.
   */
  void SetData(const Storage& rawData, const Storage& filteredData,
               const std::string& unit, const std::vector<double>& ff,
               const std::array<units::second_t, 4>& startTimes,
               AnalysisType type, std::atomic<bool>& abort);

  /**
   * Displays voltage-domain plots.
   *
   * @return Returns true if plots aren't in the loading state
   */
  bool DisplayVoltageDomainPlots(ImVec2 plotSize = ImVec2(-1, 0));

  /**
   * Displays time-domain plots.
   *
   * @return Returns true if plots aren't in the loading state
   */
  bool DisplayTimeDomainPlots(ImVec2 plotSize = ImVec2(-1, 0));

  void DisplayCombinedPlots();

  bool LoadPlots();

  void FitPlots();

 private:
  // The maximum size of each vector (dataset to plot).
  static constexpr size_t kMaxSize = 2048;

  // Stores ImPlotPoint vectors for all of the data.
  wpi::StringMap<std::vector<ImPlotPoint>> m_filteredData;
  wpi::StringMap<std::vector<ImPlotPoint>> m_rawData;

  std::string m_velocityLabel;
  std::string m_accelerationLabel;

  // Stores points for the lines of best fit.
  ImPlotPoint m_KvFit[2];
  ImPlotPoint m_KaFit[2];

  // Stores points for simulated time-domain data.
  std::vector<std::vector<ImPlotPoint>> m_quasistaticSim;
  std::vector<std::vector<ImPlotPoint>> m_dynamicSim;

  // Stores differences in time deltas
  std::vector<std::vector<ImPlotPoint>> m_dt;
  std::vector<ImPlotPoint> m_dtMeanLine;

  // Thread safety
  wpi::spinlock m_mutex;

  // Logger
  wpi::Logger& m_logger;

  // Stores whether this was the first call to Plot() since setting data.
  std::array<bool, sizeof(kChartTitles) / sizeof(kChartTitles[0])>
      m_fitNextPlot{};
};
}  // namespace sysid
