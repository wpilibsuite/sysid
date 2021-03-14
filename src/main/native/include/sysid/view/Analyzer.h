// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <glass/View.h>
#include <implot.h>
#include <portable-file-dialogs.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/Logger.h>
#include <wpi/StringMap.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"

struct ImPlotPoint;

namespace sysid {
class Analyzer : public glass::View {
 public:
  static constexpr const char* kPresetNames[] = {
      "Default",    "WPILib (2020-)",  "WPILib (Pre-2020)", "CTRE (New)",
      "CTRE (Old)", "REV (Brushless)", "REV (Brushed)"};

  static constexpr const char* kLoopTypes[] = {"Position", "Velocity"};

  static constexpr const char* kUnits[] = {"Meters",  "Feet",      "Inches",
                                           "Radians", "Rotations", "Degrees"};

  explicit Analyzer(wpi::Logger& logger);
  void Display() override;

 private:
  void SelectFile();
  void PrepareData();
  void Calculate();
  void ResetManagerState();

  struct PlotData {
    const char* name;
    ImPlotPoint (*getter)(void*, int);
    std::function<std::vector<PreparedData>*()> data;

    const char* xlabel;
    const char* ylabel;
  };

  /**
   * Plots time domain simulation of feedforward gains
   */
  template <typename Model>
  void PlotTimeDomainSim(const PlotData& data, Model model) {
    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);

    std::vector<double> xs;
    std::vector<double> ys;

    // Get data list and initialize model state
    auto preparedData = *(data.data());
    xs.emplace_back(0.0);
    ys.emplace_back(preparedData[0].velocity);

    model.Reset(preparedData[0].position, preparedData[0].velocity);
    units::second_t t = 0_s;
    for (int j = 1; j < preparedData.size(); ++j) {
      const auto& prev = preparedData[j - 1];
      const auto& now = preparedData[j];

      auto dt =
          units::second_t{now.timestamp} - units::second_t{prev.timestamp};
      t += dt;

      // If there's a large gap or the time went backwards, it's a new
      // section of data, so reset the model state
      if (dt < 0_s || dt > 1_s) {
        ImPlot::PlotLine("##Fit", &xs[0], &ys[0], xs.size());
        xs.clear();
        ys.clear();
        model.Reset(now.position, now.velocity);
        continue;
      }

      model.Update(units::volt_t{prev.voltage}, dt);

      xs.emplace_back(t);
      ys.emplace_back(model.GetVelocity());
    }
    ImPlot::PlotLine("##Fit", &xs[0], &ys[0], xs.size());
  }

  bool first = true;
  std::string m_exception;

  // Everything related to feedback controller calculations.
  AnalysisManager::Settings m_settings;
  wpi::StringMap<FeedbackControllerPreset> m_presets;

  int m_selectedLoopType = 1;
  int m_selectedPreset = 0;

  // Feedforward and feedback gains.
  std::vector<double> m_ff;
  double m_rs;
  double m_Kp;
  double m_Kd;

  // Track width
  std::optional<double> m_trackWidth;

  // Units
  double m_factor;
  std::string m_unit;
  int m_selectedOverrideUnit = 0;

  // Data analysis
  std::unique_ptr<AnalysisManager> m_manager;
  AnalysisType m_type;
  int m_window = 8;
  double m_threshold = 0.2;

  // Plotting
  std::vector<PlotData> m_timeDomainData;
  std::vector<PlotData> m_voltageDomainData;

  // File manipulation
  std::unique_ptr<pfd::open_file> m_selector;
  std::string* m_location;

  // Logger
  wpi::Logger& m_logger;
};
}  // namespace sysid
