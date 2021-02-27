// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Analyzer.h"

#include <implot.h>

#include <algorithm>
#include <exception>

#include <glass/Context.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>
#include <wpi/FileSystem.h>
#include <wpi/math>
#include <wpi/raw_ostream.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisType.h"

using namespace sysid;

struct VoltageDomainPlotData {
  const std::vector<PreparedData>* vec;
  const std::vector<double>& ff;
  AnalysisType type;
};

// This is the factor by which we should divide the data when plotting. For
// example, if the size of the data is larger than 1 << 12, we will only visit
// every other point.
static int CalculatePlotVectorFactor(size_t size) {
  return static_cast<int>(std::ceil(size * 1.0 / (1 << 12)));
}

// Methods that return various ImPlotPoint values for plotting, given a
// const_iterator to the data.
ImPlotPoint GetVelocityVsVoltage(void* data, int idx) {
  auto& d = *static_cast<VoltageDomainPlotData*>(data);

  // If the data is too big, then we should "stride" the data so that we only
  // visit every 2, 3, ... points.
  int factor = CalculatePlotVectorFactor(d.vec->size());

  auto& p = d.vec->at(idx * factor);
  if (d.type == analysis::kElevator) {
    return ImPlotPoint(p.voltage - d.ff[3] -
                           std::copysign(d.ff[0], p.velocity) -
                           d.ff[2] * p.acceleration,
                       p.velocity);
  }
  if (d.type == analysis::kArm) {
    return ImPlotPoint(p.voltage - std::copysign(d.ff[0], p.velocity) -
                           d.ff[2] * p.acceleration - d.ff[3] * p.cos,
                       p.velocity);
  }

  return ImPlotPoint(
      p.voltage - std::copysign(d.ff[0], p.velocity) - d.ff[2] * p.acceleration,
      p.velocity);
}

ImPlotPoint GetAccelerationVsVoltage(void* data, int idx) {
  auto& d = *static_cast<VoltageDomainPlotData*>(data);
  // If the data is too big, then we should "stride" the data so that we only
  // visit every 2, 3, ... points.
  int factor = CalculatePlotVectorFactor(d.vec->size());

  auto& p = d.vec->at(idx * factor);
  if (d.type == analysis::kElevator) {
    return ImPlotPoint(p.voltage - d.ff[3] -
                           std::copysign(d.ff[0], p.velocity) -
                           d.ff[1] * p.velocity,
                       p.acceleration);
  }
  if (d.type == analysis::kArm) {
    return ImPlotPoint(p.voltage - std::copysign(d.ff[0], p.velocity) -
                           d.ff[1] * p.velocity - d.ff[3] * p.cos,
                       p.acceleration);
  }

  return ImPlotPoint(
      p.voltage - std::copysign(d.ff[0], p.velocity) - d.ff[1] * p.velocity,
      p.acceleration);
}

ImPlotPoint GetVelocityVsTime(void* data, int idx) {
  auto& d = *static_cast<std::vector<PreparedData>*>(data);
  idx *= CalculatePlotVectorFactor(d.size());
  return ImPlotPoint(d[idx].timestamp - d[0].timestamp, d[idx].velocity);
}

ImPlotPoint GetAccelerationVsTime(void* data, int idx) {
  auto& d = *static_cast<std::vector<PreparedData>*>(data);
  idx *= CalculatePlotVectorFactor(d.size());
  return ImPlotPoint(d[idx].timestamp - d[0].timestamp, d[idx].acceleration);
}

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

  // Configure plot data.
  m_timeDomainData.push_back(
      PlotData{"Quasistatic Velocity vs. Time", GetVelocityVsTime,
               [this] { return &std::get<0>(m_manager->GetRawData()); },
               "Time (s)", "Velocity (units/s)"});
  m_timeDomainData.push_back(
      PlotData{"Quasistatic Acceleration vs. Time", GetAccelerationVsTime,
               [this] { return &std::get<0>(m_manager->GetRawData()); },
               "Time (s)", "Acceleration (units/s/s)"});
  m_timeDomainData.push_back(
      PlotData{"Dynamic Velocity vs. Time", GetVelocityVsTime,
               [this] { return &std::get<1>(m_manager->GetRawData()); },
               "Time (s)", "Velocity (units/s)"});
  m_timeDomainData.push_back(
      PlotData{"Dynamic Acceleration vs. Time", GetAccelerationVsTime,
               [this] { return &std::get<1>(m_manager->GetRawData()); },
               "Time (s)", "Acceleration (units/s/s)"});

  m_voltageDomainData.push_back(PlotData{
      "Quasistatic Velocity vs. Velocity-Portion Voltage", GetVelocityVsVoltage,
      [this] { return &std::get<0>(m_manager->GetRawData()); },
      "Velocity-Portion Voltage (V)", "Velocity (units/s)"});

  m_voltageDomainData.push_back(
      PlotData{"Dynamic Acceleration vs. Acceleration-Portion Voltage",
               GetAccelerationVsVoltage,
               [this] { return &std::get<1>(m_manager->GetRawData()); },
               "Velocity-Portion Voltage (V)", "Acceleration (units/s/s)"});
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
    m_selector = std::make_unique<pfd::open_file>("Select Data");
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

      if (ImGui::Button("Close")) {
        ImGui::CloseCurrentPopup();
        m_manager->OverrideUnits(m_unit, m_factor);
        Calculate();
      }

      ImGui::EndPopup();
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset Units from JSON")) {
      m_manager->ResetUnitsFromJSON();
      Calculate();
    }
  }

  // Function that displays a read-only value.
  auto ShowGain = [](const char* text, double* data) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    ImGui::InputDouble(text, data, 0.0, 0.0, "%.3G",
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

      ShowGain("r-squared", &m_rs);
      float endY = ImGui::GetCursorPosY();

      // Come back to the starting y pos.
      ImGui::SetCursorPosY(beginY);

      // Create buttons to show diagnostics.
      auto ShowDiagnostics = [](const char* text) {
        ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
        if (ImGui::Button(text)) {
          ImGui::OpenPopup(text);
        }
      };

      ShowDiagnostics("Voltage-Domain Diagnostics");
      ShowDiagnostics("Time-Domain Diagnostics");

      ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      int window = m_settings.windowSize;
      if (ImGui::InputInt("Window Size", &window, 0, 0)) {
        m_settings.windowSize = std::clamp(window, 2, 10);
        PrepareData();
        Calculate();
      }

      ImGui::SetCursorPosX(ImGui::GetFontSize() * 15);
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      double threshold = m_settings.motionThreshold;
      if (ImGui::InputDouble("Velocity Threshold", &threshold, 0.0, 0.0,
                             "%.3f")) {
        m_settings.motionThreshold = std::max(0.0, threshold);
        PrepareData();
        Calculate();
      }

      auto size = ImGui::GetIO().DisplaySize;
      ImGui::SetNextWindowSize(ImVec2(size.x / 2.5, size.y * 0.9));

      // Show voltage domain diagnostic plots.
      if (ImGui::BeginPopupModal("Voltage-Domain Diagnostics")) {
        // Loop through the plot data sources that we have.
        for (size_t i = 0; i < m_voltageDomainData.size(); ++i) {
          auto& data = m_voltageDomainData[i];
          // Make sure that the axes are properly scaled to show all data and
          // set the marker size.
          ImPlot::FitNextPlotAxes();

          // Create the plot.
          if (ImPlot::BeginPlot(data.name, data.xlabel, data.ylabel,
                                ImVec2(-1, 0), ImPlotFlags_None,
                                ImPlotAxisFlags_NoGridLines,
                                ImPlotAxisFlags_NoGridLines)) {
            // For voltage domain data, we need to create a struct with the raw
            // data, feedforward values, and the type of analysis.
            VoltageDomainPlotData d{data.data(), m_ff, m_type};

            // Calculate the factor by which we should scale the data.
            int factor = CalculatePlotVectorFactor(data.data()->size());

            // Plot the scatter data.
            ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
            ImPlot::PlotScatterG("", data.getter, &d,
                                 data.data()->size() / factor);

            // Plot the line of best fit.
            auto minE =
                std::min_element(data.data()->cbegin(), data.data()->cend(),
                                 [i](const auto& a, const auto& b) {
                                   if (i == 0) {
                                     return a.velocity < b.velocity;
                                   }
                                   return a.acceleration < b.acceleration;
                                 });
            auto maxE =
                std::max_element(data.data()->cbegin(), data.data()->cend(),
                                 [i](const auto& a, const auto& b) {
                                   if (i == 0) {
                                     return a.velocity < b.velocity;
                                   }
                                   return a.acceleration < b.acceleration;
                                 });

            double min;
            double max;
            if (i == 0) {
              min = minE->velocity;
              max = maxE->velocity;
            } else {
              min = minE->acceleration;
              max = maxE->acceleration;
            }

            double x[2] = {m_ff[i + 1] * min, m_ff[i + 1] * max};
            double y[2] = {min, max};

            ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
            ImPlot::PlotLine("##Fit", &x[0], &y[0], 2);

            ImPlot::EndPlot();
          }
        }
        // Button to close popup.
        if (ImGui::Button("Close")) {
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }

      ImGui::SetNextWindowSize(ImVec2(size.x / 2.5, size.y * 0.9));

      // Show time domain diagnostic plots.
      if (ImGui::BeginPopupModal("Time-Domain Diagnostics")) {
        // Loop through the plot data sources that we have.
        for (auto&& data : m_timeDomainData) {
          // Make sure that the axes are properly scaled to show all data and
          // set the marker size.
          ImPlot::FitNextPlotAxes();
          ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);

          // Create the plot.
          if (ImPlot::BeginPlot(data.name, data.xlabel, data.ylabel)) {
            // Calculate the factor by which we should scale the data.
            int factor = CalculatePlotVectorFactor(data.data()->size());

            // For time domain data, only the raw data is required, so we can
            // plot it directly.
            ImPlot::PlotScatterG("", data.getter, data.data(),
                                 data.data()->size() / factor);
            ImPlot::EndPlot();
          }
        }
        // Button to close popup.
        if (ImGui::Button("Close")) {
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }

      ImGui::SetCursorPosY(endY);
    }
  }
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
                      ImGui::GetFontSize() * 17);
      ShowPresetValue("Velocity Measurement Delay (s)",
                      reinterpret_cast<double*>(
                          &m_settings.preset.velocityMeasurementDelay),
                      ImGui::GetFontSize() * 17);

      ImGui::SetCursorPosY(endY);

      // Add EPR and Gearing for converting Feedback Gains
      ImGui::Separator();
      ImGui::Spacing();

      if (ImGui::Checkbox("Convert Gains to Encoder Ticks",
                          &m_settings.convertGainsToEncTicks)) {
        Calculate();
      }
      sysid::CreateTooltip(
          "Whether the feedback gains should be in terms of encoder ticks or "
          "output units. Because smart motor controllers usually don't have "
          "direct access to the output units (i.e. m/s for a drivetrain), they "
          "perform feedback on the encoder units directly. If you are using a "
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
        if (ImGui::InputInt("EPR", &m_settings.epr, 0) && m_settings.epr > 0) {
          Calculate();
        }
        sysid::CreateTooltip(
            "The edges per rotation of your encoder. This is the number of "
            "ticks reported in user code when the encoder is rotated exactly "
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
    m_manager =
        std::make_unique<AnalysisManager>(*m_location, m_settings, m_logger);
    m_type = m_manager->GetAnalysisType();
    Calculate();
  }
}

void Analyzer::PrepareData() {
  try {
    m_manager->PrepareData();
  } catch (const std::exception& e) {
    m_exception = e.what();
    ResetManagerState();
    ImGui::OpenPopup("Exception Caught!");
  }
}

void Analyzer::Calculate() {
  try {
    auto gains = m_manager->Calculate();
    m_ff = std::get<0>(gains.ff);
    m_rs = std::get<1>(gains.ff);
    m_Kp = std::get<0>(gains.fb);
    m_Kd = std::get<1>(gains.fb);
    m_trackWidth = gains.trackWidth;
    m_unit = m_manager->GetUnit();
    m_factor = m_manager->GetFactor();
  } catch (const std::exception& e) {
    m_exception = e.what();
    ResetManagerState();
    ImGui::OpenPopup("Exception Caught!");
  }
}

void Analyzer::ResetManagerState() {
  m_manager.reset();
  *m_location = "";
}
