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
#include "sysid/analysis/ArmSim.h"
#include "sysid/analysis/ElevatorSim.h"
#include "sysid/analysis/FeedbackControllerPreset.h"
#include "sysid/analysis/SimpleMotorSim.h"
#include "sysid/view/UILayout.h"

using namespace sysid;

static double kBadDataGain = -9999.0;

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
}

void Analyzer::DisplayGain(const char* text, double* data) {
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
  if (!m_enabled) {
    data = &kBadDataGain;
  }
  ImGui::InputDouble(text, data, 0.0, 0.0, "%.5G",
                     ImGuiInputTextFlags_ReadOnly);
}

static void SetPosition(double beginX, double beginY, double xShift,
                        double yShift) {
  ImGui::SetCursorPos(ImVec2(beginX + xShift * 10 * ImGui::GetFontSize(),
                             beginY + yShift * 1.75 * ImGui::GetFontSize()));
}

void Analyzer::ConfigParamsOnFileSelect() {
  if (m_enabled) {
    m_stepTestDuration = m_settings.stepTestDuration.to<float>();

    // Estimate qp as 1/8 * units-per-rot
    m_settings.lqr.qp = 0.125 * m_manager->GetFactor();
    // Estimate qv as 1/4 * max velocity = 1/4 * (12V - kS) / kV
    m_settings.lqr.qv = 0.25 * (12.0 - m_ff[0]) / m_ff[1];
  }
}

void Analyzer::Display() {
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
  ImGui::SetNextItemWidth(width - ImGui::CalcTextSize("Select").x);
  ImGui::InputText("##location", &m_location, ImGuiInputTextFlags_ReadOnly);

  // Allow the user to select which data set they want analyzed and add a reset
  // button. Also show the units and the units per rotation.
  if (m_manager) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * kTextBoxWidthMultiple);
    if (ImGui::Combo("Dataset", &m_settings.dataset, AnalysisManager::kDatasets,
                     m_type == analysis::kDrivetrain ? 9 : 3)) {
      m_enabled = true;
      Calculate();
      PrepareGraphs();
    }
    ImGui::SameLine(width - ImGui::CalcTextSize("Reset").x);
    if (ImGui::Button("Reset")) {
      m_plot.ResetData();
      m_manager.reset();
      m_location = "";
      m_enabled = true;
    }
    ImGui::Spacing();
    ImGui::Text(
        "Units:              %s\n"
        "Units Per Rotation: %.4f\n"
        "Type:               %s",
        m_unit.c_str(), m_factor, m_type.name);

    if (m_type == analysis::kDrivetrainAngular) {
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
      m_unit = kUnits[m_selectedOverrideUnit];

      if (m_unit == "Degrees") {
        m_factor = 360.0;
      } else if (m_unit == "Radians") {
        m_factor = 2 * wpi::numbers::pi;
      } else if (m_unit == "Rotations") {
        m_factor = 1.0;
      }

      bool isRotational = m_selectedOverrideUnit > 2;

      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7);
      ImGui::InputDouble("Units Per Rotation", &m_factor, 0.0, 0.0, "%.4f",
                         isRotational ? ImGuiInputTextFlags_ReadOnly
                                      : ImGuiInputTextFlags_None);

      bool ex = false;

      if (ImGui::Button("Close")) {
        ImGui::CloseCurrentPopup();
        try {
          m_manager->OverrideUnits(m_unit, m_factor);
          m_enabled = true;
          Calculate();
        } catch (const std::exception& e) {
          ex = true;
          m_exception = e.what();
        }
      }

      ImGui::EndPopup();
      if (ex) {
        m_errorPopup = true;
      }
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset Units from JSON")) {
      m_manager->ResetUnitsFromJSON();
      m_enabled = true;
      Calculate();
    }
  }

  // Function that displays a read-only value.
  ImGui::Spacing();
  ImGui::Spacing();

  // Collapsing Headers are used for Feedforward and Feedback Analysis.
  ImGui::SetNextItemOpen(true, ImGuiCond_Once);
  if (ImGui::CollapsingHeader("Feedforward Analysis")) {
    // Depending on whether a file has been selected or not, display a generic
    // warning message or show the feedforward gains.
    if (!m_manager) {
      ImGui::Text("Please Select a JSON File");
    } else {
      DisplayFeedforwardGains();
    }
  }

  ImGui::SetNextWindowPos(ImVec2{kDiagnosticPlotWindowPos},
                          ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2{kDiagnosticPlotWindowSize},
                           ImGuiCond_FirstUseEver);
  ImGui::Begin("Diagnostic Plots");
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
    if (m_manager) {
      DisplayGain("Acceleration R²", &m_rSquared);
      CreateTooltip(
          "The coefficient of determination of the OLS fit of acceleration "
          "versus velocity and voltage.  Acceleration is extremely noisy, "
          "so this is generally quite small.");

      DisplayGain("Sim velocity R²", m_plot.GetSimRSquared());
      CreateTooltip(
          "The coefficient of determination the simulated velocity. "
          "Velocity is much less-noisy than acceleration, so this "
          "is pretty close to 1 for a decent fit.");

      DisplayGain("Sim RMSE", m_plot.GetRMSE());
      CreateTooltip(
          "The Root Mean Squared Error (RMSE) of the simulation "
          "predictions compared to the recorded data. It is essentially the "
          "mean error of the simulated model in the recorded velocity units.");
    }
  }
  m_prevPlotsLoaded = plotsLoaded;

  ImGui::End();

  ImGui::SetNextItemOpen(true, ImGuiCond_Once);
  if (ImGui::CollapsingHeader("Feedback Analysis")) {
    // Depending on whether a file has been selected or not, display a generic
    // warning message or show the feedback gains.
    if (!m_manager) {
      ImGui::Text("Please Select a JSON File");
    } else {
      DisplayFeedbackGains();
    }
  }

  // Periodic functions
  try {
    SelectFile();
  } catch (const std::exception& e) {
    HandleGeneralError(e);
  }
  CreateErrorPopup(m_errorPopup, m_exception);
}

void Analyzer::SelectFile() {
  // If the selector exists and is ready with a result, we can store it.
  if (m_selector && m_selector->ready() && !m_selector->result().empty()) {
    // Store the location of the file and reset the selector.
    WPI_INFO(m_logger, "Opening File: {}", m_selector->result()[0]);
    m_location = m_selector->result()[0];
    m_selector.reset();
    m_enabled = true;
    WPI_INFO(m_logger, "{}", "Opened File");

    // Create the analysis manager.
    try {
      m_manager =
          std::make_unique<AnalysisManager>(m_location, m_settings, m_logger);
      m_type = m_manager->GetAnalysisType();
      m_factor = m_manager->GetFactor();
      m_unit = m_manager->GetUnit();
      RefreshInformation();
      ConfigParamsOnFileSelect();
    } catch (const wpi::json::exception& e) {
      HandleJSONError(e);
    }
  }
}

void Analyzer::PrepareData() {
  WPI_INFO(m_logger, "{}", "Preparing Data.");
  if (!m_enabled) {
    WPI_INFO(m_logger, "{}", "Returning early for data preparation.");
    return;
  }
  try {
    m_manager->PrepareData();
  } catch (const wpi::json::exception& e) {
    HandleJSONError(e);
  } catch (const std::exception& e) {
    HandleGeneralError(e);
  }
}

void Analyzer::Calculate() {
  WPI_INFO(m_logger, "{}", "Calculating Gains.");
  if (!m_enabled) {
    WPI_INFO(m_logger, "{}", "Returning early for gain calculation.");
    return;
  }
  try {
    const auto& [ff, fb, trackWidth] = m_manager->Calculate();
    m_ff = std::get<0>(ff);
    m_rSquared = std::get<1>(ff);
    m_timescale = m_ff[2] / m_ff[1];
    m_Kp = fb.Kp;
    m_Kd = fb.Kd;
    m_trackWidth = trackWidth;
    m_factor = m_manager->GetFactor();
  } catch (const std::exception& e) {
    HandleGeneralError(e);
  }
}

void Analyzer::AbortDataPrep() {
  if (m_dataThread.joinable()) {
    m_abortDataPrep = true;
    m_dataThread.join();
    m_abortDataPrep = false;
  }
}

void Analyzer::PrepareGraphs() {
  WPI_INFO(m_logger, "{}", "Graphing Data.");
  if (!m_enabled) {
    // Try to display raw data if manager didn't get reset due to a JSON error
    if (m_manager) {
      WPI_INFO(m_logger, "{}", "Attempting to graph only raw time series.");
      try {
        AbortDataPrep();
        m_dataThread = std::thread([&] {
          m_plot.SetRawData(m_manager->GetOriginalData(), m_manager->GetUnit(),
                            m_abortDataPrep);
        });
      } catch (const std::exception& e) {
        HandleGeneralError(e);
      }
      return;
    } else {
      return;
    }
  }
  try {
    AbortDataPrep();
    m_dataThread = std::thread([&] {
      m_plot.SetData(m_manager->GetRawData(), m_manager->GetFilteredData(),
                     m_manager->GetUnit(), m_ff, m_manager->GetStartTimes(),
                     m_type, m_abortDataPrep);
    });
  } catch (const std::exception& e) {
    HandleGeneralError(e);
  }
}

void Analyzer::RefreshInformation() {
  PrepareData();
  Calculate();
  PrepareGraphs();
}

void Analyzer::ResetManagerState() {
  m_enabled = false;
}

void Analyzer::HandleGeneralError(const std::exception& e) {
  m_exception = e.what();
  ResetManagerState();
  m_errorPopup = true;
}

void Analyzer::HandleJSONError(const wpi::json::exception& e) {
  m_exception = fmt::format(
      "The provided JSON was invalid! You may need to rerun the logger.\nError "
      "message:{}",
      std::string(e.what()));
  ResetManagerState();
  // Invalid JSONS should not be stored or used as data
  m_manager.reset();
  m_location = "";
  m_errorPopup = true;
}

void Analyzer::DisplayFeedforwardGains() {
  float beginX = ImGui::GetCursorPosX();
  float beginY = ImGui::GetCursorPosY();
  const char* gainNames[] = {"Ks", "Kv", "Ka"};

  for (size_t i = 0; i < 3; i++) {
    SetPosition(beginX, beginY, 0, i);
    DisplayGain(gainNames[i], &m_ff[i]);
  }

  size_t row = 3;

  SetPosition(beginX, beginY, 0, row);

  if (m_type == analysis::kElevator) {
    DisplayGain("Kg", &m_ff[3]);
    ++row;
  } else if (m_type == analysis::kArm) {
    DisplayGain("Kcos", &m_ff[3]);
    ++row;
  } else if (m_trackWidth) {
    DisplayGain("Track Width", &*m_trackWidth);
    ++row;
  }

  SetPosition(beginX, beginY, 0, row);
  DisplayGain("Response Timescale (s)", &m_timescale);
  CreateTooltip(
      "The characteristic timescale of the system response in seconds. "
      "Both the control loop period and total signal delay should be "
      "at least 3-5 times shorter than this to optimally control the "
      "system.");

  double endY = ImGui::GetCursorPosY();

  // Increase spacing to not run into trackwidth in the normal analyzer view
  constexpr double kHorizontalOffset = 0.9;
  SetPosition(beginX, beginY, kHorizontalOffset, 0);

  // Wait for enter before refresh so double digit entries like "15" don't
  // prematurely refresh with "1". That can cause display stuttering.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  int window = m_settings.medianWindow;
  if (ImGui::InputInt("Window Size", &window, 0, 0,
                      ImGuiInputTextFlags_EnterReturnsTrue)) {
    m_settings.medianWindow = std::clamp(window, 1, 15);
    m_enabled = true;
    RefreshInformation();
  }

  CreateTooltip(
      "The number of samples in the velocity median "
      "filter's sliding window.");

  // Wait for enter before refresh so decimal inputs like "0.2" don't
  // prematurely refresh with a velocity threshold of "0".
  SetPosition(beginX, beginY, kHorizontalOffset, 0.9);
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  double threshold = m_settings.motionThreshold;
  if (ImGui::InputDouble("Velocity Threshold", &threshold, 0.0, 0.0, "%.3f",
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
    m_settings.motionThreshold = std::max(0.0, threshold);
    m_enabled = true;
    RefreshInformation();
  }

  CreateTooltip("Velocity data below this threshold will be ignored.");

  SetPosition(beginX, beginY, kHorizontalOffset, 2);
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  if (ImGui::SliderFloat("Test Duration", &m_stepTestDuration,
                         m_manager->GetMinDuration(),
                         m_manager->GetMaxDuration(), "%.2f")) {
    m_settings.stepTestDuration = units::second_t{m_stepTestDuration};
    m_enabled = true;
    RefreshInformation();
  }

  ImGui::SetCursorPosY(endY);
}

void Analyzer::DisplayFeedbackGains() {
  // Allow the user to select a feedback controller preset.
  ImGui::Spacing();
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * kTextBoxWidthMultiple);
  if (ImGui::Combo("Gain Preset", &m_selectedPreset, kPresetNames,
                   IM_ARRAYSIZE(kPresetNames))) {
    m_settings.preset = m_presets[kPresetNames[m_selectedPreset]];
    m_settings.convertGainsToEncTicks = m_selectedPreset > 2;
    m_enabled = true;
    Calculate();
  }
  ImGui::SameLine();
  sysid::CreateTooltip(
      "Gain presets represent how feedback gains are calculated for your "
      "specific feedback controller:\n\n"
      "Default, WPILib (2020-): For use with the new WPILib PIDController "
      "class.\n"
      "WPILib (Pre-2020): For use with the old WPILib PIDController class.\n"
      "CTRE (New): For use with new CTRE units. Note that CTRE has not "
      "released an update with these units.\n"
      "CTRE (Old): For use with old CTRE units. These are the default units "
      "that ship with CTRE motor controllers.\n"
      "REV (Brushless): For use with NEO and NEO 550 motors on a SPARK MAX.\n"
      "REV (Brushed): For use with brushless motors connected to a SPARK MAX.");

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
    m_enabled = true;
    Calculate();
  }

  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
  value = m_settings.preset.outputVelocityTimeFactor;
  if (ImGui::InputDouble("Velocity Denominator Units (s)", &value, 0.0, 0.0,
                         "%.1f") &&
      value > 0) {
    m_settings.preset.outputVelocityTimeFactor = value;
    m_enabled = true;
    Calculate();
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
    if (ImGui::InputDouble(text, data, 0.0, 0.0, "%.4f") && *data > 0) {
      m_enabled = true;
      Calculate();
    }
  };

  // Show controller period.
  ShowPresetValue("Controller Period (s)",
                  reinterpret_cast<double*>(&m_settings.preset.period));

  // Show whether the controller gains are time-normalized.
  if (ImGui::Checkbox("Time-Normalized?", &m_settings.preset.normalized)) {
    m_enabled = true;
    Calculate();
  }

  // Show position/velocity measurement delay.
  ShowPresetValue(
      "Measurement Delay (s)",
      reinterpret_cast<double*>(&m_settings.preset.measurementDelay));

  // Add CPR and Gearing for converting Feedback Gains
  ImGui::Separator();
  ImGui::Spacing();

  if (ImGui::Checkbox("Convert Gains to Encoder Counts",
                      &m_settings.convertGainsToEncTicks)) {
    m_enabled = true;
    Calculate();
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
    if (ImGui::InputDouble("##Numerator", &m_gearingNumerator, 0.0, 0.0,
                           "%.4f") &&
        m_gearingNumerator > 0) {
      m_settings.gearing = m_gearingNumerator / m_gearingDenominator;
      m_enabled = true;
      Calculate();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    if (ImGui::InputDouble("##Denominator", &m_gearingDenominator, 0.0, 0.0,
                           "%.4f") &&
        m_gearingDenominator > 0) {
      m_settings.gearing = m_gearingNumerator / m_gearingDenominator;
      m_enabled = true;
      Calculate();
    }
    sysid::CreateTooltip(
        "The gearing between the encoder and the output shaft (# of "
        "encoder turns / # of output shaft turns).");

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    if (ImGui::InputInt("CPR", &m_settings.cpr, 0) && m_settings.cpr > 0) {
      m_enabled = true;
      Calculate();
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
    m_enabled = true;
    Calculate();
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

  auto ShowLQRParam = [this](const char* text, double* data, float min,
                             float max, bool power = true) {
    float val = *data;
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    ImGui::SetCursorPosX(ImGui::GetFontSize() * 9);

    if (ImGui::SliderFloat(text, &val, min, max, "%.1f",
                           ImGuiSliderFlags_None |
                               (power ? ImGuiSliderFlags_Logarithmic : 0))) {
      *data = static_cast<double>(val);
      m_enabled = true;
      Calculate();
    }
  };

  std::string_view abbreviation = GetAbbreviation(m_unit);

  if (m_selectedLoopType == 0) {
    ShowLQRParam(fmt::format("Max Position Error ({})", abbreviation).c_str(),
                 &m_settings.lqr.qp, 0.05f, 40.0f);
  }

  ShowLQRParam(fmt::format("Max Velocity Error ({}/s)", abbreviation).c_str(),
               &m_settings.lqr.qv, 0.05f, 40.0f);
  ShowLQRParam("Max Control Effort (V)", &m_settings.lqr.r, 0.1f, 12.0f, false);
}
