// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Analyzer.h"

#include <algorithm>
#include <exception>
#include <thread>

#include <fmt/core.h>
#include <glass/Context.h>
#include <glass/Storage.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <wpi/fs.h>
#include <wpi/json.h>
#include <wpi/numbers>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/FeedbackControllerPreset.h"
#include "sysid/analysis/FilteringUtils.h"
#include "sysid/view/UILayout.h"

using namespace sysid;

Analyzer::Analyzer(glass::Storage& storage, wpi::Logger& logger)
    : m_location(""), m_logger(logger) {
  // Fill the StringMap with preset values.
  m_presets["Default"] = presets::kDefault;
  m_presets["WPILib (2020-)"] = presets::kWPILibNew;
  m_presets["WPILib (Pre-2020)"] = presets::kWPILibOld;
  m_presets["CANCoder"] = presets::kCTRECANCoder;
  m_presets["CTRE"] = presets::kCTREDefault;
  m_presets["REV Brushless Encoder Port"] = presets::kREVNEOBuiltIn;
  m_presets["REV Brushed Encoder Port"] = presets::kREVNonNEO;
  m_presets["REV Data Port"] = presets::kREVNonNEO;
  m_presets["Venom"] = presets::kVenom;

  ResetData();
  UpdateFeedbackGains();
}

void Analyzer::UpdateFeedforwardGains() {
  WPI_INFO(m_logger, "{}", "Gain calc");
  try {
    const auto& [ff, trackWidth] = m_manager->CalculateFeedforward();
    m_ff = std::get<0>(ff);
    m_accelRSquared = std::get<1>(ff);
    m_accelRMSE = std::get<2>(ff);
    m_trackWidth = trackWidth;
    m_settings.preset.measurementDelay =
        m_settings.type == FeedbackControllerLoopType::kPosition
            ? m_manager->GetPositionDelay()
            : m_manager->GetVelocityDelay();
    m_conversionFactor = m_manager->GetFactor();
    PrepareGraphs();
  } catch (const sysid::InvalidDataError& e) {
    m_state = AnalyzerState::kGeneralDataError;
    HandleError(e.what());
  } catch (const sysid::NoQuasistaticDataError& e) {
    m_state = AnalyzerState::kMotionThresholdError;
    HandleError(e.what());
  } catch (const sysid::NoDynamicDataError& e) {
    m_state = AnalyzerState::kTestDurationError;
    HandleError(e.what());
  } catch (const AnalysisManager::FileReadingError& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  } catch (const wpi::json::exception& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  } catch (const std::exception& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  }
}

void Analyzer::UpdateFeedbackGains() {
  if (m_ff[1] > 0 && m_ff[2] > 0) {
    const auto& fb = m_manager->CalculateFeedback(m_ff);
    m_timescale = units::second_t{m_ff[2] / m_ff[1]};
    m_Kp = fb.Kp;
    m_Kd = fb.Kd;
  }
}

bool Analyzer::DisplayGain(const char* text, double* data,
                           bool readOnly = true) {
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
  if (readOnly) {
    return ImGui::InputDouble(text, data, 0.0, 0.0, "%.5G",
                              ImGuiInputTextFlags_ReadOnly);
  } else {
    return ImGui::InputDouble(text, data, 0.0, 0.0, "%.5G");
  }
}

static void SetPosition(double beginX, double beginY, double xShift,
                        double yShift) {
  ImGui::SetCursorPos(ImVec2(beginX + xShift * 10 * ImGui::GetFontSize(),
                             beginY + yShift * 1.75 * ImGui::GetFontSize()));
}

bool Analyzer::IsErrorState() {
  return m_state == AnalyzerState::kMotionThresholdError ||
         m_state == AnalyzerState::kTestDurationError ||
         m_state == AnalyzerState::kGeneralDataError ||
         m_state == AnalyzerState::kFileError;
}

bool Analyzer::IsDataErrorState() {
  return m_state == AnalyzerState::kMotionThresholdError ||
         m_state == AnalyzerState::kTestDurationError ||
         m_state == AnalyzerState::kGeneralDataError;
}

void Analyzer::DisplayFileSelector() {
  // Get the current width of the window. This will be used to scale
  // our UI elements.
  float width = ImGui::GetContentRegionAvail().x;

  // Show the file location along with an option to choose.
  if (ImGui::Button("Select")) {
    m_selector = std::make_unique<pfd::open_file>(
        "Select Data", "",
        std::vector<std::string>{"JSON File", SYSID_PFD_JSON_EXT});
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(width - ImGui::CalcTextSize("Select").x -
                          ImGui::GetFontSize() * 5);
  ImGui::InputText("##location", &m_location, ImGuiInputTextFlags_ReadOnly);
}

void Analyzer::ResetData() {
  m_plot.ResetData();
  m_manager = std::make_unique<AnalysisManager>(m_settings, m_logger);
  m_location = "";
  m_ff = std::vector<double>{1, 1, 1};
  UpdateFeedbackGains();
}

bool Analyzer::DisplayResetAndUnitOverride() {
  auto type = m_manager->GetAnalysisType();
  auto unit = m_manager->GetUnit();

  float width = ImGui::GetContentRegionAvail().x;
  ImGui::SameLine(width - ImGui::CalcTextSize("Reset").x);
  if (ImGui::Button("Reset")) {
    ResetData();
    m_state = AnalyzerState::kWaitingForJSON;
    return true;
  }

  if (type == analysis::kDrivetrain) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * kTextBoxWidthMultiple);
    if (ImGui::Combo("Dataset", &m_dataset, kDatasets, 3)) {
      PrepareData();
      m_settings.dataset =
          static_cast<AnalysisManager::Settings::DrivetrainDataset>(m_dataset);
    }
    ImGui::SameLine();
  } else {
    m_settings.dataset =
        AnalysisManager::Settings::DrivetrainDataset::kCombined;
  }

  ImGui::Spacing();
  ImGui::Text(
      "Units:              %s\n"
      "Units Per Rotation: %.4f\n"
      "Type:               %s",
      std::string(unit).c_str(), m_conversionFactor, type.name);

  if (type == analysis::kDrivetrainAngular) {
    ImGui::SameLine();
    sysid::CreateTooltip(
        "Here, the units and units per rotation represent what the wheel "
        "positions and velocities were captured in. The track width value "
        "will reflect the unit selected here. However, the Kv and Ka will "
        "always be in Vs/rad and Vs^2 / rad respectively.");
  }

  if (ImGui::Button("Override Units")) {
    ImGui::OpenPopup("Override Units");
  }

  auto size = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowSize(ImVec2(size.x / 4, size.y * 0.2));
  if (ImGui::BeginPopupModal("Override Units")) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7);
    ImGui::Combo("Units", &m_selectedOverrideUnit, kUnits,
                 IM_ARRAYSIZE(kUnits));
    unit = kUnits[m_selectedOverrideUnit];

    if (unit == "Degrees") {
      m_conversionFactor = 360.0;
    } else if (unit == "Radians") {
      m_conversionFactor = 2 * wpi::numbers::pi;
    } else if (unit == "Rotations") {
      m_conversionFactor = 1.0;
    }

    bool isRotational = m_selectedOverrideUnit > 2;

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7);
    ImGui::InputDouble(
        "Units Per Rotation", &m_conversionFactor, 0.0, 0.0, "%.4f",
        isRotational ? ImGuiInputTextFlags_ReadOnly : ImGuiInputTextFlags_None);

    bool ex = false;

    if (ImGui::Button("Close")) {
      ImGui::CloseCurrentPopup();
      m_manager->OverrideUnits(unit, m_conversionFactor);
      PrepareData();
    }

    ImGui::EndPopup();
  }

  ImGui::SameLine();
  if (ImGui::Button("Reset Units from JSON")) {
    m_manager->ResetUnitsFromJSON();
    PrepareData();
  }

  return false;
}

void Analyzer::ConfigParamsOnFileSelect() {
  WPI_INFO(m_logger, "{}", "Configuring Params");
  m_stepTestDuration = m_settings.stepTestDuration.to<float>();

  // Estimate qp as 1/8 * units-per-rot
  m_settings.lqr.qp = 0.125 * m_manager->GetFactor();
  // Estimate qv as 1/4 * max velocity = 1/4 * (12V - kS) / kV
  m_settings.lqr.qv = 0.25 * (12.0 - m_ff[0]) / m_ff[1];
}

void Analyzer::Display() {
  DisplayFileSelector();
  DisplayGraphs();

  switch (m_state) {
    case AnalyzerState::kWaitingForJSON: {
      ImGui::Text(
          "SysId is currently in theoretical analysis mode.\n"
          "To analyze recorded test data, select a "
          "data JSON.");
      sysid::CreateTooltip(
          "Theoretical feedback gains can be calculated from a "
          "physical model of the mechanism being controlled. "
          "Theoretical gains for several common mechanisms can "
          "be obtained from ReCalc (https://reca.lc).");
      ImGui::Spacing();
      ImGui::Spacing();

      ImGui::SetNextItemOpen(true, ImGuiCond_Once);
      if (ImGui::CollapsingHeader("Feedforward Gains (Theoretical)")) {
        float beginX = ImGui::GetCursorPosX();
        float beginY = ImGui::GetCursorPosY();
        CollectFeedforwardGains(beginX, beginY);
      }
      ImGui::SetNextItemOpen(true, ImGuiCond_Once);
      if (ImGui::CollapsingHeader("Feedback Analysis")) {
        DisplayFeedbackGains();
      }
      break;
    }
    case AnalyzerState::kNominalDisplay: {  // Allow the user to select which
                                            // data set they want analyzed and
                                            // add a
      // reset button. Also show the units and the units per rotation.
      if (DisplayResetAndUnitOverride()) {
        return;
      }
      ImGui::Spacing();
      ImGui::Spacing();

      ImGui::SetNextItemOpen(true, ImGuiCond_Once);
      if (ImGui::CollapsingHeader("Feedforward Analysis")) {
        float beginX = ImGui::GetCursorPosX();
        float beginY = ImGui::GetCursorPosY();
        DisplayFeedforwardGains(beginX, beginY);
      }
      ImGui::SetNextItemOpen(true, ImGuiCond_Once);
      if (ImGui::CollapsingHeader("Feedback Analysis")) {
        DisplayFeedbackGains();
      }
      break;
    }
    case AnalyzerState::kFileError: {
      CreateErrorPopup(m_errorPopup, m_exception);
      if (!m_errorPopup) {
        m_state = AnalyzerState::kWaitingForJSON;
        return;
      }
      break;
    }
    case AnalyzerState::kGeneralDataError:
    case AnalyzerState::kTestDurationError:
    case AnalyzerState::kMotionThresholdError: {
      CreateErrorPopup(m_errorPopup, m_exception);
      if (DisplayResetAndUnitOverride()) {
        return;
      }
      float beginX = ImGui::GetCursorPosX();
      float beginY = ImGui::GetCursorPosY();
      DisplayFeedforwardParameters(beginX, beginY);
      break;
    }
  }

  // Periodic functions
  try {
    SelectFile();
  } catch (const AnalysisManager::FileReadingError& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  } catch (const wpi::json::exception& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  }
}

void Analyzer::PrepareData() {
  try {
    m_manager->PrepareData();
    UpdateFeedforwardGains();
    UpdateFeedbackGains();
  } catch (const sysid::InvalidDataError& e) {
    m_state = AnalyzerState::kGeneralDataError;
    HandleError(e.what());
  } catch (const sysid::NoQuasistaticDataError& e) {
    m_state = AnalyzerState::kMotionThresholdError;
    HandleError(e.what());
  } catch (const sysid::NoDynamicDataError& e) {
    m_state = AnalyzerState::kTestDurationError;
    HandleError(e.what());
  } catch (const AnalysisManager::FileReadingError& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  } catch (const wpi::json::exception& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  } catch (const std::exception& e) {
    m_state = AnalyzerState::kFileError;
    HandleError(e.what());
  }
}

void Analyzer::PrepareRawGraphs() {
  if (m_manager->HasData()) {
    AbortDataPrep();
    m_dataThread = std::thread([&] {
      m_plot.SetRawData(m_manager->GetOriginalData(), m_manager->GetUnit(),
                        m_abortDataPrep);
    });
  }
}

void Analyzer::PrepareGraphs() {
  if (m_manager->HasData()) {
    WPI_INFO(m_logger, "{}", "Graph state");
    AbortDataPrep();
    m_dataThread = std::thread([&] {
      m_plot.SetData(m_manager->GetRawData(), m_manager->GetFilteredData(),
                     m_manager->GetUnit(), m_ff, m_manager->GetStartTimes(),
                     m_manager->GetAnalysisType(), m_abortDataPrep);
    });
    UpdateFeedbackGains();
    m_state = AnalyzerState::kNominalDisplay;
  }
}

void Analyzer::HandleError(std::string_view msg) {
  m_exception = msg;
  m_errorPopup = true;
  if (m_state == AnalyzerState::kFileError) {
    m_location = "";
  }
  PrepareRawGraphs();
}

void Analyzer::DisplayGraphs() {
  ImGui::SetNextWindowPos(ImVec2{kDiagnosticPlotWindowPos},
                          ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2{kDiagnosticPlotWindowSize},
                           ImGuiCond_FirstUseEver);
  ImGui::Begin("Diagnostic Plots");

  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
  if (ImGui::SliderFloat("Point Size", &m_plot.m_pointSize, 1, 2, "%.2f")) {
    if (!IsErrorState()) {
      PrepareGraphs();
    } else {
      PrepareRawGraphs();
    }
  }

  ImGui::SameLine();
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
  const char* items[] = {"Forward", "Backward"};
  if (ImGui::Combo("Direction", &m_plot.m_direction, items, 2)) {
    if (!IsErrorState()) {
      PrepareGraphs();
    } else {
      PrepareRawGraphs();
    }
  }

  // If the plots were already loaded, store the scroll position. Else go to
  // the last recorded scroll position if they have just been initialized
  bool plotsLoaded = m_plot.DisplayPlots();
  if (plotsLoaded) {
    if (m_prevPlotsLoaded) {
      m_graphScroll = ImGui::GetScrollY();
    } else {
      ImGui::SetScrollY(m_graphScroll);
    }

    // If a JSON is selected
    if (m_state == AnalyzerState::kNominalDisplay) {
      DisplayGain("Acceleration R²", &m_accelRSquared);
      CreateTooltip(
          "The coefficient of determination of the OLS fit of acceleration "
          "versus velocity and voltage.  Acceleration is extremely noisy, "
          "so this is generally quite small.");

      ImGui::SameLine();
      DisplayGain("Acceleration RMSE", &m_accelRMSE);
      CreateTooltip(
          "The standard deviation of the residuals from the predicted "
          "acceleration."
          "This can be interpreted loosely as the mean measured disturbance "
          "from the \"ideal\" system equation.");

      DisplayGain("Sim velocity R²", m_plot.GetSimRSquared());
      CreateTooltip(
          "The coefficient of determination the simulated velocity. "
          "Velocity is much less-noisy than acceleration, so this "
          "is pretty close to 1 for a decent fit.");

      ImGui::SameLine();
      DisplayGain("Sim velocity RMSE", m_plot.GetSimRMSE());
      CreateTooltip(
          "The standard deviation of the residuals from the simulated velocity "
          "predictions - essentially the size of the mean error of the "
          "simulated model "
          "in the recorded velocity units.");
    }
  }
  m_prevPlotsLoaded = plotsLoaded;

  ImGui::End();
}

void Analyzer::SelectFile() {
  // If the selector exists and is ready with a result, we can store it.
  if (m_selector && m_selector->ready() && !m_selector->result().empty()) {
    // Store the location of the file and reset the selector.
    WPI_INFO(m_logger, "Opening File: {}", m_selector->result()[0]);
    m_location = m_selector->result()[0];
    m_selector.reset();
    WPI_INFO(m_logger, "{}", "Opened File");
    m_manager =
        std::make_unique<AnalysisManager>(m_location, m_settings, m_logger);
    PrepareData();
    m_dataset = 0;
    m_settings.dataset =
        AnalysisManager::Settings::DrivetrainDataset::kCombined;
    ConfigParamsOnFileSelect();
    UpdateFeedbackGains();
  }
}

void Analyzer::AbortDataPrep() {
  if (m_dataThread.joinable()) {
    m_abortDataPrep = true;
    m_dataThread.join();
    m_abortDataPrep = false;
  }
}

void Analyzer::DisplayFeedforwardParameters(float beginX, float beginY) {
  // Increase spacing to not run into trackwidth in the normal analyzer view
  constexpr double kHorizontalOffset = 0.9;
  SetPosition(beginX, beginY, kHorizontalOffset, 0);

  bool displayAll =
      !IsErrorState() || m_state == AnalyzerState::kGeneralDataError;

  if (displayAll) {
    // Wait for enter before refresh so double digit entries like "15" don't
    // prematurely refresh with "1". That can cause display stuttering.
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    int window = m_settings.medianWindow;
    if (ImGui::InputInt("Window Size", &window, 0, 0,
                        ImGuiInputTextFlags_EnterReturnsTrue)) {
      m_settings.medianWindow = std::clamp(window, 1, 15);
      PrepareData();
    }

    CreateTooltip(
        "The number of samples in the velocity median "
        "filter's sliding window.");
  }

  if (displayAll || m_state == AnalyzerState::kMotionThresholdError) {
    // Wait for enter before refresh so decimal inputs like "0.2" don't
    // prematurely refresh with a velocity threshold of "0".
    SetPosition(beginX, beginY, kHorizontalOffset, 1);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    double threshold = m_settings.motionThreshold;
    if (ImGui::InputDouble("Velocity Threshold", &threshold, 0.0, 0.0, "%.3f",
                           ImGuiInputTextFlags_EnterReturnsTrue)) {
      m_settings.motionThreshold = std::max(0.0, threshold);
      PrepareData();
    }
    CreateTooltip("Velocity data below this threshold will be ignored.");
  }

  if (displayAll || m_state == AnalyzerState::kTestDurationError) {
    SetPosition(beginX, beginY, kHorizontalOffset, 2);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    if (ImGui::SliderFloat("Test Duration", &m_stepTestDuration,
                           m_manager->GetMinStepTime().value(),
                           m_manager->GetMaxStepTime().value(), "%.2f")) {
      m_settings.stepTestDuration = units::second_t{m_stepTestDuration};
      PrepareData();
    }
  }
}

void Analyzer::CollectFeedforwardGains(float beginX, float beginY) {
  SetPosition(beginX, beginY, 0, 0);
  if (DisplayGain("Kv", &m_ff[1], false)) {
    UpdateFeedbackGains();
  }

  SetPosition(beginX, beginY, 0, 1);
  if (DisplayGain("Ka", &m_ff[2], false)) {
    UpdateFeedbackGains();
  }

  SetPosition(beginX, beginY, 0, 2);
  // Show Timescale
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  DisplayGain("Response Timescale (ms)",
              reinterpret_cast<double*>(&m_timescale));
  CreateTooltip(
      "The characteristic timescale of the system response in milliseconds. "
      "Both the control loop period and total signal delay should be "
      "at least 3-5 times shorter than this to optimally control the "
      "system.");
}

void Analyzer::DisplayFeedforwardGains(float beginX, float beginY) {
  SetPosition(beginX, beginY, 0, 0);
  DisplayGain("Ks", &m_ff[0]);

  SetPosition(beginX, beginY, 0, 1);
  DisplayGain("Kv", &m_ff[1]);

  SetPosition(beginX, beginY, 0, 2);
  DisplayGain("Ka", &m_ff[2]);

  SetPosition(beginX, beginY, 0, 3);
  // Show Timescale
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  DisplayGain("Response Timescale (ms)",
              reinterpret_cast<double*>(&m_timescale));
  CreateTooltip(
      "The characteristic timescale of the system response in milliseconds. "
      "Both the control loop period and total signal delay should be "
      "at least 3-5 times shorter than this to optimally control the "
      "system.");

  SetPosition(beginX, beginY, 0, 4);
  auto positionDelay = m_manager->GetPositionDelay();
  DisplayGain("Position Measurement Delay (ms)",
              reinterpret_cast<double*>(&positionDelay));
  CreateTooltip(
      "The average elapsed time between the first application of "
      "voltage and the first detected change in mechanism position "
      "in the step-voltage tests.  This includes CAN delays, and "
      "may overestimate the true delay for on-motor-controller "
      "feedback loops by up to 20ms.");

  SetPosition(beginX, beginY, 0, 5);
  auto velocityDelay = m_manager->GetVelocityDelay();
  DisplayGain("Velocity Measurement Delay (ms)",
              reinterpret_cast<double*>(&velocityDelay));
  CreateTooltip(
      "The average elapsed time between the first application of "
      "voltage and the maximum calculated mechanism acceleration "
      "in the step-voltage tests.  This includes CAN delays, and "
      "may overestimate the true delay for on-motor-controller "
      "feedback loops by up to 20ms.");

  SetPosition(beginX, beginY, 0, 6);

  if (m_manager->GetAnalysisType() == analysis::kElevator) {
    DisplayGain("Kg", &m_ff[3]);
  } else if (m_manager->GetAnalysisType() == analysis::kArm) {
    DisplayGain("Kcos", &m_ff[3]);

    double offset;
    auto unit = m_manager->GetUnit();
    if (unit == "Radians") {
      offset = m_ff[4];
    } else if (unit == "Degrees") {
      offset = m_ff[4] / wpi::numbers::pi * 180.0;
    } else if (unit == "Rotations") {
      offset = m_ff[4] / (2 * wpi::numbers::pi);
    }
    DisplayGain(
        fmt::format("Angle offset to horizontal ({})", GetAbbreviation(unit))
            .c_str(),
        &offset);
    CreateTooltip(
        "This is the angle offset which, when added to the angle measurement, "
        "zeroes it out when the arm is horizontal. This is needed for the arm "
        "feedforward to work.");
  } else if (m_trackWidth) {
    DisplayGain("Track Width", &*m_trackWidth);
  }
  double endY = ImGui::GetCursorPosY();

  DisplayFeedforwardParameters(beginX, beginY);
  ImGui::SetCursorPosY(endY);
}

void Analyzer::DisplayFeedbackGains() {
  // Allow the user to select a feedback controller preset.
  ImGui::Spacing();
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * kTextBoxWidthMultiple);
  if (ImGui::Combo("Gain Preset", &m_selectedPreset, kPresetNames,
                   IM_ARRAYSIZE(kPresetNames))) {
    m_settings.preset = m_presets[kPresetNames[m_selectedPreset]];
    m_settings.type = FeedbackControllerLoopType::kVelocity;
    m_selectedLoopType =
        static_cast<int>(FeedbackControllerLoopType::kVelocity);
    m_settings.convertGainsToEncTicks = m_selectedPreset > 2;
    UpdateFeedbackGains();
  }
  ImGui::SameLine();
  sysid::CreateTooltip(
      "Gain presets represent how feedback gains are calculated for your "
      "specific feedback controller:\n\n"
      "Default, WPILib (2020-): For use with the new WPILib PIDController "
      "class.\n"
      "WPILib (Pre-2020): For use with the old WPILib PIDController class.\n"
      "CTRE: For use with CTRE units. These are the default units that ship "
      "with CTRE motor controllers.\n"
      "REV (Brushless): For use with NEO and NEO 550 motors on a SPARK MAX.\n"
      "REV (Brushed): For use with brushed motors connected to a SPARK MAX.");

  if (m_settings.preset != m_presets[kPresetNames[m_selectedPreset]]) {
    ImGui::SameLine();
    ImGui::TextDisabled("(modified)");
  }

  // Show our feedback controller preset values.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  double value = m_settings.preset.outputConversionFactor * 12;
  if (ImGui::InputDouble("Max Controller Output", &value, 0.0, 0.0, "%.1f") &&
      value > 0) {
    m_settings.preset.outputConversionFactor = value / 12.0;
    UpdateFeedbackGains();
  }

  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  value = m_settings.preset.outputVelocityTimeFactor;
  if (ImGui::InputDouble("Velocity Denominator Units (s)", &value, 0.0, 0.0,
                         "%.1f") &&
      value > 0) {
    m_settings.preset.outputVelocityTimeFactor = value;
    UpdateFeedbackGains();
  }

  sysid::CreateTooltip(
      "This represents the denominator of the velocity unit used by the "
      "feedback controller. For example, CTRE uses 100 ms = 0.1 s.");

  auto ShowPresetValue = [this](const char* text, double* data,
                                float cursorX = 0.0f) {
    if (cursorX > 0) {
      ImGui::SetCursorPosX(cursorX);
    }

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    return ImGui::InputDouble(text, data, 0.0, 0.0, "%.5G");
  };

  // Show controller period.
  if (ShowPresetValue("Controller Period (ms)",
                      reinterpret_cast<double*>(&m_settings.preset.period))) {
    if (m_settings.preset.period > 0_s &&
        m_settings.preset.measurementDelay >= 0_s) {
      UpdateFeedbackGains();
    }
  }

  // Show whether the controller gains are time-normalized.
  if (ImGui::Checkbox("Time-Normalized?", &m_settings.preset.normalized)) {
    UpdateFeedbackGains();
  }

  // Show position/velocity measurement delay.
  if (ShowPresetValue(
          "Measurement Delay (ms)",
          reinterpret_cast<double*>(&m_settings.preset.measurementDelay))) {
    if (m_settings.preset.period > 0_s &&
        m_settings.preset.measurementDelay >= 0_s) {
      UpdateFeedbackGains();
    }
  }

  sysid::CreateTooltip(
      "The average measurement delay of the process variable in milliseconds. "
      "This may depend on your encoder settings and choice of motor "
      "controller. Default velocity filtering windows are quite long "
      "on many motor controllers, so be careful that this value is "
      "accurate if the characteristic timescale of the mechanism "
      "is small.");

  // Add CPR and Gearing for converting Feedback Gains
  ImGui::Separator();
  ImGui::Spacing();

  if (ImGui::Checkbox("Convert Gains to Encoder Counts",
                      &m_settings.convertGainsToEncTicks)) {
    UpdateFeedbackGains();
  }
  sysid::CreateTooltip(
      "Whether the feedback gains should be in terms of encoder counts or "
      "output units. Because smart motor controllers usually don't have "
      "direct access to the output units (i.e. m/s for a drivetrain), they "
      "perform feedback on the encoder counts directly. If you are using a "
      "PID Controller on the RoboRIO, you are probably performing feedback "
      "on the output units directly.\n\nNote that if you have properly set "
      "up position and velocity conversion factors with the SPARK MAX, you "
      "can leave this box unchecked. The motor controller will perform "
      "feedback on the output directly.");

  if (m_settings.convertGainsToEncTicks) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    if (ImGui::InputDouble("##Numerator", &m_gearingNumerator, 0.0, 0.0, "%.4f",
                           ImGuiInputTextFlags_EnterReturnsTrue) &&
        m_gearingNumerator > 0) {
      m_settings.gearing = m_gearingNumerator / m_gearingDenominator;
      UpdateFeedbackGains();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    if (ImGui::InputDouble("##Denominator", &m_gearingDenominator, 0.0, 0.0,
                           "%.4f", ImGuiInputTextFlags_EnterReturnsTrue) &&
        m_gearingDenominator > 0) {
      m_settings.gearing = m_gearingNumerator / m_gearingDenominator;
      UpdateFeedbackGains();
    }
    sysid::CreateTooltip(
        "The gearing between the encoder and the output shaft (# of "
        "encoder turns / # of output shaft turns).");

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    if (ImGui::InputInt("CPR", &m_settings.cpr, 0, 0,
                        ImGuiInputTextFlags_EnterReturnsTrue) &&
        m_settings.cpr > 0) {
      UpdateFeedbackGains();
    }
    sysid::CreateTooltip(
        "The counts per rotation of your encoder. This is the number of counts "
        "reported in user code when the encoder is rotated exactly once. Some "
        "common values for various motors/encoders are:\n\n"
        "Falcon 500: 2048\nNEO: 1\nCTRE Mag Encoder / CANCoder: 4096\nREV "
        "Through Bore Encoder: 8192\n");
  }

  ImGui::Separator();
  ImGui::Spacing();

  // Allow the user to select a loop type.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * kTextBoxWidthMultiple);
  if (ImGui::Combo("Loop Type", &m_selectedLoopType, kLoopTypes,
                   IM_ARRAYSIZE(kLoopTypes))) {
    m_settings.type =
        static_cast<FeedbackControllerLoopType>(m_selectedLoopType);
    if (m_state == AnalyzerState::kWaitingForJSON) {
      m_settings.preset.measurementDelay = 0_ms;
    } else {
      if (m_settings.type == FeedbackControllerLoopType::kPosition) {
        m_settings.preset.measurementDelay = m_manager->GetPositionDelay();
      } else {
        m_settings.preset.measurementDelay = m_manager->GetVelocityDelay();
      }
    }
    UpdateFeedbackGains();
  }

  ImGui::Spacing();

  // Show Kp and Kd.
  float beginY = ImGui::GetCursorPosY();
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  DisplayGain("Kp", &m_Kp);

  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  DisplayGain("Kd", &m_Kd);

  // Come back to the starting y pos.
  ImGui::SetCursorPosY(beginY);

  std::string_view abbreviation = GetAbbreviation(m_manager->GetUnit());

  if (m_selectedLoopType == 0) {
    ImGui::SetCursorPosX(ImGui::GetFontSize() * 9);
    if (DisplayGain(
            fmt::format("Max Position Error ({})", abbreviation).c_str(),
            &m_settings.lqr.qp, false)) {
      if (m_settings.lqr.qp > 0) {
        UpdateFeedbackGains();
      }
    }
  }

  ImGui::SetCursorPosX(ImGui::GetFontSize() * 9);
  if (DisplayGain(
          fmt::format("Max Velocity Error ({}/s)", abbreviation).c_str(),
          &m_settings.lqr.qv, false)) {
    if (m_settings.lqr.qv > 0) {
      UpdateFeedbackGains();
    }
  }
  ImGui::SetCursorPosX(ImGui::GetFontSize() * 9);
  if (DisplayGain("Max Control Effort (V)", &m_settings.lqr.r, false)) {
    if (m_settings.lqr.r > 0) {
      UpdateFeedbackGains();
    }
  }
}
