// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <wpi/StringMap.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>

#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedbackAnalysis.h"
#include "sysid/analysis/FeedbackControllerPreset.h"
#include "sysid/analysis/FeedforwardAnalysis.h"

namespace sysid {
/**
 * Represents each data point after it is cleaned and various parameters are
 * calculated.
 */
struct PreparedData {
  double timestamp;
  double voltage;
  double position;
  double velocity;
  double acceleration;
  double cos;
};

/** The raw data that is contained within the JSON. */
using RawData = std::array<double, 10>;

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
    int windowSize = 8;

    /** The dataset that is being analyzed. */
    int dataset = 0;
  };

  /** Stores feedforward and feedback gains */
  struct Gains {
    std::tuple<std::vector<double>, double> ff;
    std::tuple<double, double> fb;
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

  /** Represents one "set" of data. 0 is slow tests, 1 is fast tests. */
  using Storage =
      std::tuple<std::vector<PreparedData>, std::vector<PreparedData>>;

  /**
   * Constructs an instance of the analysis manager with the given path (to the
   * JSON) and analysis manager settings.
   *
   * @param path     The path to the JSON containing the sysid data.
   * @param settings The settings for this instance of the analysis manager.
   */
  AnalysisManager(wpi::StringRef path, const Settings& settings);

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
   * Returns a reference to the iterator of the currently selected datset.
   * Unfortunately, due to ImPlot internals, the reference cannot be const so
   * the user should be careful not to change any data.
   *
   * @return A reference to the raw internal data.
   */
  Storage& GetRawData() { return m_datasets[kDatasets[m_settings.dataset]]; }

  /**
   * Trims the existing raw data vector to remove any values where the voltage
   * is less than or equal to zero or where the velocity is less than the
   * provided threshold.
   *
   * @param data       The raw data.
   * @param threshold  The motion threshold -- all velocities below this value
   *                   will be discarded.
   * @param drivetrain Whether the data also contains data for the right side of
   *                   the drivetrain.
   */
  static void TrimQuasistaticData(std::vector<RawData>* data, double threshold,
                                  bool drivetrain = false);

  /**
   * Computes acceleration from the given raw data and returns a new vector of
   * prepared data that can be used for feedforward and feedback analysis.
   *
   * @param data   The raw data.
   * @param window The window size to compute acceleration.
   * @param right  Whether acceleration should be computed on the right side of
   *               the drivetrain.
   *
   * @return The prepared data.
   */
  static std::vector<PreparedData> ComputeAcceleration(
      const std::vector<RawData>& data, int window, bool right = false);

  /**
   * Trims the step voltage data such that we discard all data before the point
   * of maximum acceleration.
   *
   * @param data The step voltage (dynamic test) data to trim.
   */
  static void TrimStepVoltageData(std::vector<PreparedData>* data);

 private:
  /**
   * Special function that prepares drivetrain data. This is called
   * automatically from PrepareData() if m_type.mechanism ==
   * Mechanism::kDrivetrain.
   *
   * @param data The data to prepare. This should be moved in from
   *             PrepareData().
   */
  void PrepareDataDrivetrain(wpi::StringMap<std::vector<RawData>>&& data);

  /** The columns in the raw data. */
  class Cols {
   public:
    static constexpr uint8_t kTimestamp = 0;
    static constexpr uint8_t kBattery = 1;
    static constexpr uint8_t kAutospeed = 2;
    static constexpr uint8_t kLVolts = 3;
    static constexpr uint8_t kRVolts = 4;
    static constexpr uint8_t kLPos = 5;
    static constexpr uint8_t kRPos = 6;
    static constexpr uint8_t kLVel = 7;
    static constexpr uint8_t kRVel = 8;
    static constexpr uint8_t kGyro = 9;
  };

  // This is used to store the various datasets (i.e. Combined, Forward,
  // Backward, etc.)
  wpi::json m_json;
  wpi::StringMap<Storage> m_datasets;

  // The settings for this instance. This contains pointers to the feedback
  // controller preset, LQR parameters, acceleration window size, etc.
  const Settings& m_settings;

  // Miscellaneous data from the JSON -- the analysis type, units per rotation
  // (factor), the units, and whether we have track width.
  AnalysisType m_type;
  double m_factor;
  std::string m_unit;
  bool m_hasTrackWidth;
};
}  // namespace sysid
