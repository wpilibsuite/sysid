// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
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
#include "sysid/view/AnalyzerPlot.h"

struct ImPlotPoint;

namespace glass {
class Storage;
}  // namespace glass

namespace sysid {
/**
 * The Analyzer GUI takes care of providing the user with a user interface to
 * load their data, visualize the data, adjust certain variables, and then view
 * the calculated gains.
 */
class Analyzer : public glass::View {
 public:
  /**
   * The different motor controlle rtiming presets that can be used.
   */
  static constexpr const char* kPresetNames[] = {"Default",
                                                 "WPILib (2020-)",
                                                 "WPILib (Pre-2020)",
                                                 "CANCoder",
                                                 "CTRE",
                                                 "REV Brushless Encoder Port",
                                                 "REV Brushed Encoder Port",
                                                 "REV Data Port",
                                                 "Venom"};

  /**
   * The different control loops that can be used.
   */
  static constexpr const char* kLoopTypes[] = {"Position", "Velocity"};

  /**
   * Creates the Analyzer widget
   *
   * @param storage Glass Storage
   * @param logger The program logger
   */
  Analyzer(glass::Storage& storage, wpi::Logger& logger);

  /**
   * Displays the analyzer widget
   */
  void Display() override;

  ~Analyzer() override { AbortDataPrep(); };

 private:
  /**
   * Handles the logic for selecting a json to analyze
   */
  void SelectFile();

  /**
   * Calculates feedback and feedforward gains.
   */
  void Calculate();

  /**
   * Disables the backend to avoid erroneous calculations from happening.
   */
  void ResetManagerState();

  /**
   * Handles the logic of the diagnostic plots
   */
  void PrepareGraphs();

  /**
   * Prepares the data, calculates it, and generates graphs. This should be
   * called when the data needs to be reupdated after user input.
   */
  void RefreshInformation();

  /**
   * Kills the data preparation thread
   */
  void AbortDataPrep();

  /**
   * Handles the logic for displaying feedforward gains
   */
  void DisplayFeedforwardGains();

  /**
   * Handles the logic for displaying feedback gains
   */
  void DisplayFeedbackGains();

  /**
   * Estimates ideal step test duration, qp, and qv for the LQR based off of the
   * data given
   */
  void ConfigParamsOnFileSelect();

  /**
   * Handles logic of displaying a gain on ImGui
   */
  void DisplayGain(const char* text, double* data);

  /**
   * Handles errors when they pop up.
   */
  void HandleGeneralError(const std::exception& e);

  /**
   * Handles json errors when they pop up.
   */
  void HandleJSONError(const wpi::json::exception& e);

  // Whether data has been selected
  bool m_hasData = false;

  // This is true when the analysis is allowed to occur.
  bool m_enabled = true;

  // This is true if the error popup needs to be displayed
  bool m_errorPopup = false;

  std::string m_exception;

  // Everything related to feedback controller calculations.
  AnalysisManager::Settings m_settings;
  wpi::StringMap<FeedbackControllerPreset> m_presets;

  int m_selectedLoopType = 1;
  int m_selectedPreset = 0;

  // Feedforward and feedback gains.
  std::vector<double> m_ff;
  double m_rSquared;
  double m_Kp;
  double m_Kd;
  double m_timescale;

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
  float m_stepTestDuration = 0.0;

  double m_gearingNumerator = 1.0;
  double m_gearingDenominator = 1.0;

  bool combinedGraphFit = false;

  // File manipulation
  std::unique_ptr<pfd::open_file> m_selector;
  std::string m_location;

  // Logger
  wpi::Logger& m_logger;

  // Plot
  AnalyzerPlot m_plot{m_logger};
  bool m_prevPlotsLoaded = false;

  // Stores graph scroll bar position and states for keeping track of scroll
  // positions after loading graphs
  float m_graphScroll;

  std::atomic<bool> m_abortDataPrep{false};
  std::thread m_dataThread;
};
}  // namespace sysid
