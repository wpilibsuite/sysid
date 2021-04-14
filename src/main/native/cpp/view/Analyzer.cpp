// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Analyzer.h"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <thread>

#include <glass/Context.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <wpi/FileSystem.h>
#include <wpi/json.h>
#include <wpi/math>
#include <wpi/raw_ostream.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/ArmSim.h"
#include "sysid/analysis/ElevatorSim.h"
#include "sysid/analysis/SimpleMotorSim.h"

using namespace sysid;

Analyzer::Analyzer(wpi::Logger& logger) : m_logger(logger) {
  // Fill the StringMap with preset values.
  m_presets["Default"] = presets::kDefault;
  m_presets["WPILib (2020-)"] = presets::kWPILibNew;
  m_presets["WPILib (Pre-2020)"] = presets::kWPILibOld;
  m_presets["CTRE (New)"] = presets::kCTRENew;
  m_presets["CTRE (Old)"] = presets::kCTREOld;
  m_presets["REV (Brushless)"] = presets::kREVBrushless;
  m_presets["REV (Brushed)"] = presets::kREVBrushed;

  // Load the last file location from storage if it exists.
  m_location = glass::GetStorage().GetStringRef("AnalyzerJSONLocation");
}

void Analyzer::Display() {
  // Get the current width of the window. This will be used to scale
  // our UI elements.
  float width = ImGui::GetContentRegionAvail().x;

  // If this is the first call to Display() and there's a valid m_location, load
  // it.
  if (first) {
    if (!m_location->empty() && wpi::sys::fs::exists(*m_location)) {
      try {
        m_manager = std::make_unique<AnalysisManager>(*m_location, m_settings,
                                                      m_logger);
        m_type = m_manager->GetAnalysisType();
        Calculate();
        PrepareData();
        PrepareGraphs();
        m_stepTestDuration = m_settings.stepTestDuration.to<float>();
      } catch (const std::exception& e) {
        // If we run into an error here, let's just ignore it and make the user
        // explicitly select their file.
        ResetManagerState();
        WPI_ERROR(
            m_logger,
            "An error occurred when attempting to load the previous JSON.");
        static_cast<void>(e);
      }
    } else {
      *m_location = "";
    }
    first = false;
  }

  // Show the file location along with an option to choose.
  if (ImGui::Button("Select")) {
    m_selector = std::make_unique<pfd::open_file>(
        "Select Data", "",
        std::vector<std::string>{"JSON File", SYSID_PFD_JSON_EXT});
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(width - ImGui::CalcTextSize("Select").x);
  ImGui::InputText("##location", m_location, ImGuiInputTextFlags_ReadOnly);

  // Allow the user to select which data set they want analyzed and add a reset
  // button. Also show the units and the units per rotation.
  if (m_manager) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 15);
    if (ImGui::Combo("Dataset", &m_settings.dataset, AnalysisManager::kDatasets,
                     m_type == analysis::kDrivetrain ? 9 : 3)) {
      Calculate();
      PrepareGraphs();
    }
    ImGui::SameLine(width - ImGui::CalcTextSize("Reset").x);
    if (ImGui::Button("Reset")) {
      m_manager.reset();
      *m_location = "";
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
        m_factor = 2 * wpi::math::pi;
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
          Calculate();
        } catch (const std::exception& e) {
          ex = true;
          m_exception = e.what();
        }
      }

      ImGui::EndPopup();
      if (ex) {
        ImGui::OpenPopup("Exception Caught!");
      }
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset Units from JSON")) {
      m_manager->ResetUnitsFromJSON();
      Calculate();
    }
  }

  // Function that displays a read-only value.
  auto ShowGain = [](const char* text, double* data) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
    ImGui::InputDouble(text, data, 0.0, 0.0, "%.5G",
                       ImGuiInputTextFlags_ReadOnly);
  };

  ImGui::Spacing();
  ImGui::Spacing();

  // Collapsing Headers are used for Feedforward and Feedback Analysis.
  if (ImGui::CollapsingHeader("Feedforward Analysis")) {
    // Depending on whether a file has been selected or not, display a generic
    // warning message or show the feedforward gains.
    if (!m_manager) {
      ImGui::Text("Please Select a JSON File");
    } else {
      float beginY = ImGui::GetCursorPosY();
      ShowGain("Ks", &m_ff[0]);
      ShowGain("Kv", &m_ff[1]);
      ShowGain("Ka", &m_ff[2]);

      if (m_type == analysis::kElevator) {
        ShowGain("Kg", &m_ff[3]);
      } else if (m_type == analysis::kArm) {
        ShowGain("Kcos", &m_ff[3]);
      }

      if (m_trackWidth) {
        ShowGain("Track Width", &*m_trackWidth);
      }

      ShowGain("r-squared", &m_rSquared);

      double endY = ImGui::GetCursorPosY();

      // Come back to the starting y pos.
      ImGui::SetCursorPosY(beginY);

      ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
      if (ImGui::Button("Combined Diagnostics")) {
        ImGui::OpenPopup("Combined Diagnostics");
      }

      ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      int window = m_settings.windowSize;
      if (ImGui::InputInt("Window Size", &window, 0, 0)) {
        m_settings.windowSize = std::clamp(window, 2, 15);
        RefreshInformation();
      }
      ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      double threshold = m_settings.motionThreshold;
      if (ImGui::InputDouble("Velocity Threshold", &threshold, 0.0, 0.0,
                             "%.3f")) {
        m_settings.motionThreshold = std::max(0.0, threshold);
        RefreshInformation();
      }

      ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      if (ImGui::SliderFloat("Test Duration", &m_stepTestDuration,
                             m_manager->GetMinDuration(),
                             m_manager->GetMaxDuration(), "%.2f")) {
        m_settings.stepTestDuration = units::second_t{m_stepTestDuration};
        RefreshInformation();
      }

      ImGui::SetNextWindowSize(ImVec2(m_plot.kCombinedPlotSize * 4 + 50,
                                      m_plot.kCombinedPlotSize * 2 + 25),
                               ImGuiCond_Once);
      // Show plots for screenshots
      if (ImGui::BeginPopupModal("Combined Diagnostics")) {
        // Only fit the plots when the popup first shows as they get messed up
        // by being set to a different size
        if (!combinedGraphFit) {
          m_plot.FitPlots();
          combinedGraphFit = true;
        }
        m_plot.DisplayCombinedPlots();

        // Button to close popup.
        if (ImGui::Button("Close")) {
          ImGui::CloseCurrentPopup();
          combinedGraphFit = false;

          // Fit plots afterwards so that the regular plot view isn't messed up
          m_plot.FitPlots();
        }
        ImGui::EndPopup();
      }

      ImGui::SetCursorPosY(endY);
      ImGui::Separator();
    }
  }

  ImGui::SetNextWindowPos(ImVec2(895, 25), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(375, 660), ImGuiCond_FirstUseEver);
  ImGui::Begin("Diagnostic Plots");
  // If the plots were already loaded, store the scroll position. Else go to
  // the last recorded scroll position if they have just been initialized
  bool plotsLoaded = m_plot.LoadPlots();
  if (plotsLoaded) {
    if (m_prevPlotsLoaded) {
      m_graphScroll = ImGui::GetScrollY();
    } else {
      ImGui::SetScrollY(m_graphScroll);
    }
  }
  m_prevPlotsLoaded = plotsLoaded;
  ImGui::End();

  if (ImGui::CollapsingHeader("Feedback Analysis")) {
    // Depending on whether a file has been selected or not, display a generic
    // warning message or show the feedback gains.
    if (!m_manager) {
      ImGui::Text("Please Select a JSON File");
    } else {
      // Allow the user to select a feedback controller preset.
      ImGui::Spacing();
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
      if (ImGui::Combo("Gain Preset", &m_selectedPreset, kPresetNames,
                       IM_ARRAYSIZE(kPresetNames))) {
        m_settings.preset = m_presets[kPresetNames[m_selectedPreset]];
        m_settings.convertGainsToEncTicks = m_selectedPreset > 2;
        Calculate();
      }
      ImGui::SameLine();
      sysid::CreateTooltip(
          "Gain presets represent how feedback gains are calculated for your "
          "specific feedback controller:\n\n"
          "Default, WPILib (2020-): For use with the new WPILib PIDController "
          "class.\n"
          "WPILib (Pre-2020): For use with the old WPILib PIDController "
          "class.\n"
          "CTRE (New): For use with new CTRE units. Note that CTRE has not "
          "released an update with these units.\n"
          "CTRE (Old): For use with old CTRE units. These are the default "
          "units that ship with CTRE motor controllers.\n"
          "REV (Brushless): For use with NEO and NEO 550 motors on a SPARK "
          "MAX.\n"
          "REV (Brushed): For use with brushless motors connected to a SPARK "
          "MAX.");

      if (m_settings.preset != m_presets[kPresetNames[m_selectedPreset]]) {
        ImGui::SameLine();
        ImGui::TextDisabled("(modified)");
      }

      float beginY = ImGui::GetCursorPosY();
      // Show our feedback controller preset values.
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      double value = m_settings.preset.outputConversionFactor * 12;
      if (ImGui::InputDouble("Max Controller Output", &value, 0.0, 0.0,
                             "%.1f") &&
          value > 0) {
        m_settings.preset.outputConversionFactor = value / 12.0;
        Calculate();
      }

      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      value = m_settings.preset.outputVelocityTimeFactor;
      if (ImGui::InputDouble("Velocity Denominator Units (s)", &value, 0.0, 0.0,
                             "%.1f") &&
          value > 0) {
        m_settings.preset.outputVelocityTimeFactor = value;
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
          Calculate();
        }
      };

      // Show controller period.
      ShowPresetValue("Controller Period (s)",
                      reinterpret_cast<double*>(&m_settings.preset.period));

      // Show whether the controller gains are time-normalized.
      if (ImGui::Checkbox("Time-Normalized?", &m_settings.preset.normalized)) {
        Calculate();
      }
      float endY = ImGui::GetCursorPosY();

      // Show position/velocity measurement delay.
      ImGui::SetCursorPosY(beginY);
      ShowPresetValue("Position Measurement Delay (s)",
                      reinterpret_cast<double*>(
                          &m_settings.preset.positionMeasurementDelay),
                      ImGui::GetFontSize() * 23);
      ShowPresetValue("Velocity Measurement Delay (s)",
                      reinterpret_cast<double*>(
                          &m_settings.preset.velocityMeasurementDelay),
                      ImGui::GetFontSize() * 23);

      ImGui::SetCursorPosY(endY);

      // Add CPR and Gearing for converting Feedback Gains
      ImGui::Separator();
      ImGui::Spacing();

      if (ImGui::Checkbox("Convert Gains to Encoder Counts",
                          &m_settings.convertGainsToEncTicks)) {
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
        if (ImGui::InputDouble("Gearing", &m_settings.gearing, 0.0, 0.0,
                               "%.4f") &&
            m_settings.gearing > 0) {
          Calculate();
        }
        sysid::CreateTooltip(
            "The gearing between the encoder and the output shaft (# of "
            "encoder turns / # of output shaft turns).");

        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
        if (ImGui::InputInt("CPR", &m_settings.cpr, 0) && m_settings.cpr > 0) {
          Calculate();
        }
        sysid::CreateTooltip(
            "The counts per rotation of your encoder. This is the number of "
            "counts reported in user code when the encoder is rotated exactly "
            "once. Some common values for various motors/encoders "
            "are:\n\nFalcon "
            "500: 2048\nNEO: 1\nCTRE Mag Encoder / CANCoder: 4096\nREV Through "
            "Bore "
            "Encoder: 8192\n");
      }

      ImGui::Separator();
      ImGui::Spacing();

      // Allow the user to select a loop type.
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
      if (ImGui::Combo("Loop Type", &m_selectedLoopType, kLoopTypes,
                       IM_ARRAYSIZE(kLoopTypes))) {
        m_settings.type =
            static_cast<FeedbackControllerLoopType>(m_selectedLoopType);
        Calculate();
      }

      ImGui::Spacing();

      // Show Kp and Kd.
      beginY = ImGui::GetCursorPosY();
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      ShowGain("Kp", &m_Kp);

      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      ShowGain("Kd", &m_Kd);

      // Come back to the starting y pos.
      ImGui::SetCursorPosY(beginY);

      auto ShowLQRParam = [this](const char* text, double* data, float min,
                                 float max, bool power = true) {
        float val = *data;
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
        ImGui::SetCursorPosX(ImGui::GetFontSize() * 17);

        if (ImGui::SliderFloat(
                text, &val, min, max, "%.1f",
                ImGuiSliderFlags_None |
                    (power ? ImGuiSliderFlags_Logarithmic : 0))) {
          *data = static_cast<double>(val);
          Calculate();
        }
      };

      if (m_selectedLoopType == 0) {
        ShowLQRParam("Max Position Error (units)", &m_settings.lqr.qp, 0.05f,
                     40.0f);
      }

      ShowLQRParam("Max Velocity Error (units/s)", &m_settings.lqr.qv, 0.05f,
                   40.0f);
      ShowLQRParam("Max Control Effort (V)", &m_settings.lqr.r, 0.1f, 12.0f,
                   false);
    }
  }

  // Periodic functions
  try {
    SelectFile();
  } catch (const std::exception& e) {
    m_exception = e.what();
    ResetManagerState();
    ImGui::OpenPopup("Exception Caught!");
  }

  // Handle exceptions.
  if (ImGui::BeginPopupModal("Exception Caught!")) {
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s",
                       m_exception.c_str());
    if (ImGui::Button("Close")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void Analyzer::SelectFile() {
  // If the selector exists and is ready with a result, we can store it.
  if (m_selector && m_selector->ready() && !m_selector->result().empty()) {
    // Store the location of the file and reset the selector.
    *m_location = m_selector->result()[0];
    m_selector.reset();

    // Create the analysis manager.
    try {
      m_manager =
          std::make_unique<AnalysisManager>(*m_location, m_settings, m_logger);
      m_type = m_manager->GetAnalysisType();
      RefreshInformation();
      m_stepTestDuration = m_settings.stepTestDuration.to<float>();
    } catch (const wpi::json::exception& e) {
      m_exception =
          "The provided JSON was invalid! You may need to rerun the logger.\n" +
          std::string(e.what());
      ResetManagerState();
      ImGui::OpenPopup("Exception Caught!");
    }
  }
}

void Analyzer::PrepareData() {
  try {
    m_manager->PrepareData();
  } catch (const wpi::json::exception& e) {
    m_exception =
        "The provided JSON was invalid! You may need to rerun the logger.\n" +
        std::string(e.what());
    ResetManagerState();
    ImGui::OpenPopup("Exception Caught!");
  } catch (const std::exception& e) {
    m_exception = e.what();
    ResetManagerState();
    ImGui::OpenPopup("Exception Caught!");
  }
}

void Analyzer::Calculate() {
  try {
    const auto& [ff, fb, trackWidth] = m_manager->Calculate();
    m_ff = std::get<0>(ff);
    m_rSquared = std::get<1>(ff);
    m_Kp = fb.Kp;
    m_Kd = fb.Kd;
    m_trackWidth = trackWidth;
    m_unit = m_manager->GetUnit();
    m_factor = m_manager->GetFactor();
  } catch (const std::exception& e) {
    m_exception = e.what();
    ResetManagerState();
    ImGui::OpenPopup("Exception Caught!");
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
  try {
    AbortDataPrep();
    m_dataThread = std::thread([&] {
      m_plot.SetData(m_manager->GetRawData(), m_manager->GetFilteredData(),
                     m_ff, m_manager->GetStartTimes(), m_type, m_abortDataPrep);
    });
  } catch (const std::exception& e) {
    m_exception = e.what();
    ResetManagerState();
    ImGui::OpenPopup("Exception Caught!");
  }
}

void Analyzer::RefreshInformation() {
  PrepareData();
  Calculate();
  PrepareGraphs();
}

void Analyzer::ResetManagerState() {
  m_manager.reset();
  *m_location = "";
}
