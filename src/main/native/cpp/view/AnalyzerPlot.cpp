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
    const std::vector<PreparedData>& data, size_t step, Model model) {
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

    // If there's a large gap or the time went backwards, it's a new
    // section of data, so reset the model state
    if (dt < 0_s || dt > 1_s) {
      pts.emplace_back(std::move(tmp));
      model.Reset(now.position, now.velocity);
      continue;
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
                           AnalysisType type) {
  std::scoped_lock lock(m_mutex);
  auto& [slow, fast] = data;

  // Clear all data vectors.
  for (auto it = m_data.begin(); it != m_data.end(); ++it) {
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

  // Populate quasistatic time-domain graphs and quasistatic velocity vs.
  // velocity-portion voltage graph.
  double t = slow[0].timestamp;
  for (size_t i = 0; i < slow.size(); i += sStep) {
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
  }

  // Populate dynamic time-domain graphs and dynamic acceleration vs.
  // acceleration-portion voltage graph.
  t = fast[0].timestamp;
  for (size_t i = 0; i < fast.size(); i += fStep) {
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
  }

  // Populate simulated time-domain data.
  if (type == analysis::kElevator) {
    m_sim = PopulateTimeDomainSim(
        fast, fStep, sysid::ElevatorSim{ff[0], ff[1], ff[2], ff[3]});
  } else if (type == analysis::kArm) {
    m_sim = PopulateTimeDomainSim(fast, fStep,
                                  sysid::ArmSim{ff[0], ff[1], ff[2], ff[3]});
  } else {
    m_sim = PopulateTimeDomainSim(fast, fStep,
                                  sysid::SimpleMotorSim{ff[0], ff[1], ff[2]});
  }

  // Set the "fit" flag to true.
  m_fitVoltageDomainPlots = true;
  m_fitTimeDomainPlots = true;
}

void AnalyzerPlot::DisplayVoltageDomainPlots() {
  std::unique_lock lock(m_mutex, std::defer_lock);

  if (!lock.try_lock()) {
    ImGui::Text("Loading %c",
                "|/-\\"[static_cast<int>(ImGui::GetTime() / 0.05f) & 3]);
    return;
  }

  // Quasistatic Velocity vs. Velocity Portion Voltage.
  if (m_fitVoltageDomainPlots) {
    ImPlot::FitNextPlotAxes();
  }
  if (ImPlot::BeginPlot(kChartTitles[0], "Velocity-Portion Voltage",
                        "Quasistatic Velocity", ImVec2(-1, 0), ImPlotFlags_None,
                        ImPlotAxisFlags_NoGridLines,
                        ImPlotAxisFlags_NoGridLines)) {
    // Get a reference to the data that we are plotting.
    auto& data = m_data[kChartTitles[0]];

    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
    ImPlot::PlotScatterG("", Getter, data.data(), data.size());

    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
    ImPlot::PlotLineG("##Fit", Getter, m_KvFit, 2);

    ImPlot::EndPlot();
  }

  if (m_fitVoltageDomainPlots) {
    ImPlot::FitNextPlotAxes();
    m_fitVoltageDomainPlots = false;
  }
  if (ImPlot::BeginPlot(kChartTitles[1], "Acceleration-Portion Voltage",
                        "Dynamic Acceleration", ImVec2(-1, 0), ImPlotFlags_None,
                        ImPlotAxisFlags_NoGridLines)) {
    // Get a reference to the data we are plotting.
    auto& data = m_data[kChartTitles[1]];

    ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
    ImPlot::PlotScatterG("##Fit", Getter, data.data(), data.size());

    ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
    ImPlot::PlotLineG("", Getter, m_KaFit, 2);

    ImPlot::EndPlot();
  }
}

void AnalyzerPlot::DisplayTimeDomainPlots() {
  std::unique_lock lock(m_mutex, std::defer_lock);

  if (!lock.try_lock()) {
    ImGui::Text("Loading %c",
                "|/-\\"[static_cast<int>(ImGui::GetTime() / 0.05f) & 3]);
    return;
  }

  // Iterate through the chart titles for these plots and graph them.
  for (size_t i = 2; i < 6; ++i) {
    const char* x = "Time (s)";
    const char* y =
        i % 2 == 0 ? "Velocity (units / s)" : "Acceleration (units / s / s)";

    if (m_fitTimeDomainPlots) {
      ImPlot::FitNextPlotAxes();
    }
    if (ImPlot::BeginPlot(kChartTitles[i], x, y, ImVec2(-1, 0),
                          ImPlotFlags_None, ImPlotAxisFlags_NoGridLines)) {
      // Get a reference to the data we are plotting.
      auto& data = m_data[kChartTitles[i]];

      ImPlot::SetNextMarkerStyle(IMPLOT_AUTO, 1, IMPLOT_AUTO_COL, 0);
      ImPlot::PlotScatterG("", Getter, data.data(), data.size());

      // Plot simulated time-domain data.
      if (i == 4) {
        for (auto&& pts : m_sim) {
          ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 1.5);
          ImPlot::PlotLineG("##Simulated", Getter, pts.data(), pts.size());
        }
      }
      ImPlot::EndPlot();
    }
  }
  if (m_fitTimeDomainPlots) {
    m_fitTimeDomainPlots = false;
  }
}
