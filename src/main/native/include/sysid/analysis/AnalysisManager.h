// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <units/time.h>
#include <wpi/Logger.h>
#include <wpi/StringMap.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>

#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"
#include "sysid/analysis/FeedforwardAnalysis.h"
#include "sysid/analysis/Storage.h"

namespace sysid {

/**
 * Manages analysis of data. Each instance of this class represents a JSON file
 * that is read from storage.
 */
class AnalysisManager {
 public:
  /**
   * Represents settings for an instance of the analysis manager. This contains
   * information about the feedback controller preset, loop type, motion
   * threshold, acceleration window size, LQR parameters, and the selected
   * dataset.
   */
  struct Settings {
    /** The feedback controller preset used to calculate gains. */
    FeedbackControllerPreset preset = presets::kDefault;

    /** The feedback controller loop type (position or velocity). */
    FeedbackControllerLoopType type = FeedbackControllerLoopType::kVelocity;

    /** LQR parameters used for feedback gain calculation. */
    LQRParameters lqr{1, 1.5, 7};

    /** The motion threshold (units/s) for trimming quasistatic test data */
    double motionThreshold = 0.2;

    /** The window size for computing acceleration */
    int windowSize = 9;

    /** The dataset that is being analyzed. */
    int dataset = 0;

    // 0 s indicates it hasn't been set yet
    units::second_t stepTestDuration = 0_s;

    /** The conversion factors. These contain values to convert feedback gains
     * by gearing and cpr. */
    int cpr = 1440;
    double gearing = 1;
    bool convertGainsToEncTicks = false;
  };

  /** Stores feedforward and feedback gains */
  struct Gains {
    std::tuple<std::vector<double>, double> ffGains;
    FeedbackGains fbGains;
    std::optional<double> trackWidth;
  };

  /** The keys (which contain sysid data) that are in the JSON to analyze. */
  static constexpr const char* kJsonDataKeys[] = {
      "slow-forward", "slow-backward", "fast-forward", "fast-backward"};

  /** The names of the various datasets to analyze. */
  static constexpr const char* kDatasets[] = {
      "Combined",      "Forward",        "Backward",
      "Left Forward",  "Left Backward",  "Left Combined",
      "Right Forward", "Right Backward", "Right Combined"};

  /**
   * Constructs an instance of the analysis manager with the given path (to the
   * JSON) and analysis manager settings.
   *
   * @param path     The path to the JSON containing the sysid data.
   * @param settings The settings for this instance of the analysis manager.
   * @param logger   The logger instance to use for log data.
   */
  AnalysisManager(wpi::StringRef path, Settings& settings, wpi::Logger& logger);

  /**
   * Prepares data from the JSON and stores the output in the StringMap.
   */
  void PrepareData();

  /**
   * Calculates the gains with the latest data (from the pointers in the
   * settings struct that this instance was constructed with).
   *
   * @return The latest feedforward and feedback gains.
   */
  Gains Calculate();

  /**
   * Overrides the units in the JSON with the user-provided ones.
   *
   * @param unit             The unit to output gains in.
   * @param unitsPerRotation The conversion factor between rotations and the
   *                         selected unit.
   */
  void OverrideUnits(wpi::StringRef unit, double unitsPerRotation);

  /**
   * Resets the units back to those defined in the JSON.
   */
  void ResetUnitsFromJSON();

  /**
   * Returns the analysis type of the current instance (read from the JSON).
   *
   * @return The analysis type.
   */
  const AnalysisType& GetAnalysisType() const { return m_type; }

  /**
   * Returns the units of analysis.
   *
   * @return The units of analysis.
   */
  const std::string& GetUnit() const { return m_unit; }

  /**
   * Returns the factor (a.k.a. units per rotation) for analysis.
   *
   * @return The factor (a.k.a. units per rotation) for analysis.
   */
  double GetFactor() const { return m_factor; }

  /**
   * Returns a reference to the iterator of the currently selected raw datset.
   * Unfortunately, due to ImPlot internals, the reference cannot be const so
   * the user should be careful not to change any data.
   *
   * @return A reference to the raw internal data.
   */
  Storage& GetRawData() { return m_rawDatasets[kDatasets[m_settings.dataset]]; }

  /**
   * Returns a reference to the iterator of the currently selected filtered
   * datset. Unfortunately, due to ImPlot internals, the reference cannot be
   * const so the user should be careful not to change any data.
   *
   * @return A reference to the filtered internal data.
   */
  Storage& GetFilteredData() {
    return m_filteredDatasets[kDatasets[m_settings.dataset]];
  }

  /**
   * Returns the minimum duration of the Step Voltage Test of the currently
   * stored data.
   */
  double GetMinDuration() const { return m_minDuration.to<double>(); }

  /**
   * Returns the maximum duration of the Step Voltage Test of the currently
   * stored data.
   */
  double GetMaxDuration() const { return m_maxDuration.to<double>(); }

  /**
   * Returns the different start times of the recorded tests.
   */
  const std::array<units::second_t, 4> GetStartTimes() { return m_startTimes; }

 private:
  wpi::Logger& m_logger;

  // This is used to store the various datasets (i.e. Combined, Forward,
  // Backward, etc.)
  wpi::json m_json;
  wpi::StringMap<Storage> m_rawDatasets;
  wpi::StringMap<Storage> m_filteredDatasets;

  // Stores the various start times of the different tests.
  std::array<units::second_t, 4> m_startTimes;

  // The settings for this instance. This contains pointers to the feedback
  // controller preset, LQR parameters, acceleration window size, etc.
  Settings& m_settings;

  // Miscellaneous data from the JSON -- the analysis type, the units, and the
  // units per rotation.
  AnalysisType m_type;
  std::string m_unit;
  double m_factor;

  units::second_t m_minDuration;
  units::second_t m_maxDuration;

  // Stores an optional track width if we are doing the drivetrain angular test.
  std::optional<double> m_trackWidth;
};
}  // namespace sysid
