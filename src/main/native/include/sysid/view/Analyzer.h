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
#include <portable-file-dialogs.h>
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

  Analyzer();

  void Display() override;

 private:
  void SelectFile();
  void Calculate();

  struct PlotData {
    const char* name;
    ImPlotPoint (*getter)(void*, int);
    std::function<std::vector<PreparedData>*()> data;

    const char* xlabel;
    const char* ylabel;
  };

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
};
}  // namespace sysid
