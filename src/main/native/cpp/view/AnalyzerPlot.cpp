// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/AnalyzerPlot.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

#include <imgui.h>
#include <implot.h>
#include <units/time.h>
#include <wpi/raw_ostream.h>

#include "sysid/analysis/AnalysisManager.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/analysis/ArmSim.h"
#include "sysid/analysis/ElevatorSim.h"
#include "sysid/analysis/SimpleMotorSim.h"

using namespace sysid;

static ImPlotPoint Getter(void* data, int idx) {
  return static_cast<ImPlotPoint*>(data)[idx];
}

template <typename Model>
static std::vector<std::vector<ImPlotPoint>> PopulateTimeDomainSim(
    const std::vector<PreparedData>& data,
    const std::array<double, 4>& startTimes, size_t step, Model model) {
  // Create the vector of ImPlotPoints that will contain our simulated data.
  std::vector<std::vector<ImPlotPoint>> pts;
  std::vector<ImPlotPoint> tmp;
  tmp.emplace_back(0.0, data[0].velocity);

  model.Reset(data[0].position, data[0].velocity);
  units::second_t t = 0_s;

  for (size_t i = step; i < data.size(); i += step) {
    const auto& now = data[i];
    const auto& pre = data[i - step];

    auto dt = units::second_t{now.timestamp} - units::second_t{pre.timestamp};
    t += dt;

    // If the current time stamp and previous time stamp are across a test's
    // start timestamp, it is the start of a new test and the model needs to be
    // reset.
    for (const auto& startTime : startTimes) {
      if (now.timestamp >= startTime && pre.timestamp <= startTime) {
        pts.emplace_back(std::move(tmp));
        model.Reset(now.position, now.velocity);
        continue;
      }
    }

    model.Update(units::volt_t{pre.voltage}, dt);
    tmp.emplace_back(t.to<double>(), model.GetVelocity());
  }

  pts.emplace_back(std::move(tmp));
  return pts;
}

AnalyzerPlot::AnalyzerPlot(wpi::Logger& logger) : m_logger(logger) {
  // Pre-allocate our vectors with the max data size.
  for (auto&& title : kChartTitles) {
    m_data[title].reserve(kMaxSize);
  }
}

void AnalyzerPlot::SetData(const Storage& data, const std::vector<double>& ff,
                           const std::array<double, 4>& startTimes,
                           AnalysisType type, std::atomic<bool>& abort) {
  std::scoped_lock lock(m_mutex);
  auto& [slow, fast] = data;

  // Clear all data vectors.
  for (auto it = m_data.begin(); it != m_data.end(); ++it) {
    if (abort) {
      return;
    }
    it->second.clear();
  }

  // Calculate step sizes to ensure that we only use the memory that we
  // allocated.
  auto sStep = std::ceil(slow.size() * 1.0 / kMaxSize);
  auto fStep = std::ceil(fast.size() * 1.0 / kMaxSize);

  // Calculate min and max velocities and accelerations of the slow and fast
  // datasets respectively.
  auto sMinE =
      std::min_element(slow.cbegin(), slow.end(), [](auto& a, auto& b) {
        return a.velocity < b.velocity;
      })->velocity;

  auto sMaxE =
      std::max_element(slow.cbegin(), slow.end(), [](auto& a, auto& b) {
        return a.velocity < b.velocity;
      })->velocity;

  auto fMinE =
      std::min_element(fast.cbegin(), fast.end(), [](auto& a, auto& b) {
        return a.acceleration < b.acceleration;
      })->acceleration;

  auto fMaxE =
      std::max_element(fast.cbegin(), fast.end(), [](auto& a, auto& b) {
        return a.acceleration < b.acceleration;
      })->acceleration;

  int dtSamples = 0;
  double dtSum = 0;
  // Populate quasistatic time-domain graphs and quasistatic velocity vs.
  // velocity-portion voltage graph.
  double t = slow[0].timestamp;
  for (size_t i = 0; i < slow.size(); i += sStep) {
    if (abort) {
      return;
    }
    // Calculate portion of voltage that corresponds to change in velocity.
    double Vportion = slow[i].voltage - std::copysign(ff[0], slow[i].velocity) -
                      ff[2] * slow[i].acceleration;

    if (type == analysis::kElevator) {
      Vportion -= ff[3];
    } else if (type == analysis::kArm) {
      Vportion -= ff[3] * slow[i].cos;
    }

    // Calculate points to show the line of best fit.
    m_KvFit[0] = ImPlotPoint(ff[1] * sMinE, sMinE);
    m_KvFit[1] = ImPlotPoint(ff[1] * sMaxE, sMaxE);

    m_data[kChartTitles[0]].emplace_back(Vportion, slow[i].velocity);
    m_data[kChartTitles[2]].emplace_back(slow[i].timestamp - t,
                                         slow[i].velocity);
    m_data[kChartTitles[3]].emplace_back(slow[i].timestamp - t,
                                         slow[i].acceleration);

    if (i > 0) {
      // If the current timestamp is not in the startTimes array, it is the
      // during a test and should be included. If it is in the startTimes array,
      // it is the beginning of a new test and the dt will be inflated.
      // Therefore we skip those to exclude that dt and effectively reset dt
      // calculations.
      if (std::find(startTimes.begin(), startTimes.end(), slow[i].timestamp) ==
          startTimes.end()) {
        double dt = (slow[i].timestamp - slow[i - 1].timestamp) * 1000;
        m_data[kChartTitles[6]].emplace_back(slow[i].timestamp - t, dt);
        dtSum += dt;
        ++dtSamples;
      }
    }
  }

  // Populate dynamic time-domain graphs and dynamic acceleration vs.
  // acceleration-portion voltage graph.
  t = fast[0].timestamp;
  for (size_t i = 0; i < fast.size(); i += fStep) {
    if (abort) {
      return;
    }
    // Calculate portion of voltage that corresponds to change in acceleration.
    double Vportion = fast[i].voltage - std::copysign(ff[0], fast[i].velocity) -
                      ff[1] * fast[i].velocity;

    if (type == analysis::kElevator) {
      Vportion -= ff[3];
    } else if (type == analysis::kArm) {
      Vportion -= ff[3] * fast[i].cos;
    }

    // Calculate points to show the line of best fit.
    m_KaFit[0] = ImPlotPoint(ff[2] * fMinE, fMinE);
    m_KaFit[1] = ImPlotPoint(ff[2] * fMaxE, fMaxE);

    m_data[kChartTitles[1]].emplace_back(Vportion, fast[i].acceleration);
    m_data[kChartTitles[4]].emplace_back(fast[i].timestamp - t,
                                         fast[i].velocity);
    m_data[kChartTitles[5]].emplace_back(fast[i].timestamp - t,
                                         fast[i].acceleration);
    if (i > 0) {
      // If the current timestamp is not in the startTimes array, it is the
      // during a test and should be included. If it is in the startTimes array,
      // it is the beginning of a new test and the dt will be inflated.
      // Therefore we skip those to exclude that dt and effectively reset dt
      // calculations.
      if (std::find(startTimes.begin(), startTimes.end(), fast[i].timestamp) ==
          startTimes.end()) {
        double dt = (fast[i].timestamp - fast[i - 1].timestamp) * 1000;
        m_data[kChartTitles[6]].emplace_back(fast[i].timestamp - t, dt);
        dtSum += dt;
        ++dtSamples;
      }
    }
  }

  // Load dt mean for plot data
  double dtMean = dtSum / dtSamples;
  double maxTime = std::max(slow.back().timestamp - slow.front().timestamp,
                            fast.back().timestamp - fast.front().timestamp);

  // Set first recorded timestamp to mean
  m_dtMeanLine.emplace_back(0.0, dtMean);

  // Set last recorded timestamp to mean
  m_dtMeanLine.emplace_back(maxTime, dtMean);

  // Populate simulated time-domain data.
  if (type == analysis::kElevator) {
    m_quasistaticSim =
        PopulateTimeDomainSim(slow, startTimes, fStep,
                              sysid::ElevatorSim{ff[0], ff[1], ff[2], ff[3]});
    m_dynamicSim =
        PopulateTimeDomainSim(fast, startTimes, fStep,
                              sysid::ElevatorSim{ff[0], ff[1], ff[2], ff[3]});
  } else if (type == analysis::kArm) {
    m_quasistaticSim = PopulateTimeDomainSim(
        slow, startTimes, fStep, sysid::ArmSim{ff[0], ff[1], ff[2], ff[3]});
    m_dynamicSim = PopulateTimeDomainSim(
        fast, startTimes, fStep, sysid::ArmSim{ff[0], ff[1], ff[2], ff[3]});
  } else {
    m_quasistaticSim = PopulateTimeDomainSim(
        slow, startTimes, fStep, sysid::SimpleMotorSim{ff[0], ff[1], ff[2]});
    m_dynamicSim = PopulateTimeDomainSim(
        fast, startTimes, fStep, sysid::SimpleMotorSim{ff[0], ff[1], ff[2]});
  }

  FitPlots();
}

void AnalyzerPlot::FitPlots() {
  // Set the "fit" flag to true.
  std::for_each(m_fitNextPlot.begin(), m_fitNextPlot.end(),
                [](auto& f) { f = true; });
}

bool AnalyzerPlot::DisplayVoltageDomainPlots(ImVec2 plotSize) {
  std::unique_lock lock(m_mutex, std::defer_lock);

  if (!lock.try_lock()) {
    ImGui::Text("Loading %c",
                "|/-\\"[static_cast<int>(ImGui::GetTime() / 0.05f) & 3]);
    return false;
  }

  bool forPicture = plotSize.x != -1;

  // Quasistatic Velocity vs. Velocity Portion Voltage.
  if (m_fitNextPlot[0]) {
    ImPlot::FitNextPlotAxes();
  }

  if (ImPlot::BeginPlot(kChartTitles[0], "Velocity-Portion Voltage",
                        "Quasistatic Velocity", plotSize, ImPlotFlags_None,
                        ImPlotAxisFlags_NoGridLines,
                        ImPlotAxisFlags_NoGridLines)) {
    // Get a reference to the data that we are plotting.
    auto& data = m_data[kChartTitles[0]];

    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
    ImPlot::PlotScatterG("Collected Data", Getter, data.data(), data.size());

    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
    ImPlot::PlotLineG("Fit", Getter, m_KvFit, 2);

    ImPlot::EndPlot();

    if (m_fitNextPlot[0]) {
      m_fitNextPlot[0] = false;
    }
  }

  if (m_fitNextPlot[1]) {
    ImPlot::FitNextPlotAxes();
  }

  if (forPicture) {
    ImGui::SameLine();
  }

  if (ImPlot::BeginPlot(kChartTitles[1], "Acceleration-Portion Voltage",
                        "Dynamic Acceleration", plotSize, ImPlotFlags_None,
                        ImPlotAxisFlags_NoGridLines)) {
    // Get a reference to the data we are plotting.
    auto& data = m_data[kChartTitles[1]];

    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
    ImPlot::PlotScatterG("Collected Data", Getter, data.data(), data.size());

    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
    ImPlot::PlotLineG("Fit", Getter, m_KaFit, 2);

    ImPlot::EndPlot();

    if (m_fitNextPlot[1]) {
      m_fitNextPlot[1] = false;
    }
  }
  return true;
}

static void PlotSimData(std::vector<std::vector<ImPlotPoint>>& data) {
  for (auto&& pts : data) {
    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
    ImPlot::PlotLineG("Simulation", Getter, pts.data(), pts.size());
  }
}

bool AnalyzerPlot::DisplayTimeDomainPlots(ImVec2 plotSize) {
  std::unique_lock lock(m_mutex, std::defer_lock);

  if (!lock.try_lock()) {
    ImGui::Text("Loading %c",
                "|/-\\"[static_cast<int>(ImGui::GetTime() / 0.05f) & 3]);
    return false;
  }

  bool forPicture = plotSize.x != -1;

  // Iterate through the chart titles for these plots and graph them.
  for (size_t i = 2; i < 6; ++i) {
    const char* x = "Time (s)";
    const char* y =
        i % 2 == 0 ? "Velocity (units / s)" : "Acceleration (units / s / s)";
    bool isVelocity = (i == 2 || i == 4);

    if (m_fitNextPlot[i]) {
      ImPlot::FitNextPlotAxes();
    }

    if (forPicture && i % 4 != 0) {
      ImGui::SameLine();
    }
    if (ImPlot::BeginPlot(kChartTitles[i], x, y, plotSize, ImPlotFlags_None,
                          ImPlotAxisFlags_NoGridLines)) {
      // Get a reference to the data we are plotting.
      auto& data = m_data[kChartTitles[i]];

      // Set Legend Location:
      ImPlot::SetLegendLocation(ImPlotLocation_NorthEast,
                                ImPlotOrientation_Vertical, false);

      ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
      ImPlot::PlotScatterG("Collected Data", Getter, data.data(), data.size());

      // Plot Simulation Data for Velocity Data
      if (isVelocity) {
        PlotSimData((i == 2) ? m_quasistaticSim : m_dynamicSim);
      }
      ImPlot::EndPlot();

      if (m_fitNextPlot[i]) {
        m_fitNextPlot[i] = false;
      }
    }
  }

  if (forPicture) {
    ImGui::SameLine();
  }

  if (m_fitNextPlot[6]) {
    ImPlot::SetNextPlotLimitsY(0, 50);
    ImPlot::FitNextPlotAxes(true, false, false, false);
  }
  if (ImPlot::BeginPlot(kChartTitles[6], "Time (s)", "Change in Time (ms)",
                        plotSize, ImPlotFlags_None,
                        ImPlotAxisFlags_NoGridLines)) {
    // Get a reference to the data we are plotting.
    auto& data = m_data[kChartTitles[6]];

    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
    ImPlot::PlotScatterG("Timesteps", Getter, data.data(), data.size());

    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);

    ImPlot::PlotLineG("Mean dt", Getter, m_dtMeanLine.data(),
                      m_dtMeanLine.size());

    ImPlot::EndPlot();

    if (m_fitNextPlot[6]) {
      m_fitNextPlot[6] = false;
    }
  }
  return true;
}

bool AnalyzerPlot::LoadPlots() {
  // See if the plots are loaded
  return DisplayVoltageDomainPlots() && DisplayTimeDomainPlots();
}

void AnalyzerPlot::DisplayCombinedPlots() {
  const ImVec2 plotSize{kCombinedPlotSize, kCombinedPlotSize};
  DisplayVoltageDomainPlots(plotSize);
  DisplayTimeDomainPlots(plotSize);
}
