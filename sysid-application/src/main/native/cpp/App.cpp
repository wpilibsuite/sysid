// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef RUNNING_SYSID_TESTS

#include <memory>
#include <string_view>

#include <fmt/format.h>
#include <glass/Context.h>
#include <glass/Window.h>
#include <glass/WindowManager.h>
#include <glass/other/Log.h>
#include <imgui.h>
#include <libssh/libssh.h>
#include <uv.h>
#include <wpi/Logger.h>
#include <wpi/fs.h>
#include <wpigui.h>

#include "sysid/view/Analyzer.h"
#include "sysid/view/Generator.h"
#include "sysid/view/JSONConverter.h"
#include "sysid/view/Logger.h"

namespace gui = wpi::gui;

static std::unique_ptr<glass::WindowManager> gWindowManager;

glass::Window* gLoggerWindow;
glass::Window* gAnalyzerWindow;
glass::Window* gGeneratorWindow;
glass::Window* gProgramLogWindow;

std::unique_ptr<sysid::JSONConverter> gJSONConverter;

glass::LogData gLog;
wpi::Logger gLogger;

const char* GetWPILibVersion();

namespace sysid {
std::string_view GetResource_sysid_16_png();
std::string_view GetResource_sysid_32_png();
std::string_view GetResource_sysid_48_png();
std::string_view GetResource_sysid_64_png();
std::string_view GetResource_sysid_128_png();
std::string_view GetResource_sysid_256_png();
std::string_view GetResource_sysid_512_png();
}  // namespace sysid

void Application() {
  // Create the wpigui (along with Dear ImGui) and Glass contexts.
  gui::CreateContext();
  glass::CreateContext();

  // Add icons
  gui::AddIcon(sysid::GetResource_sysid_16_png());
  gui::AddIcon(sysid::GetResource_sysid_32_png());
  gui::AddIcon(sysid::GetResource_sysid_48_png());
  gui::AddIcon(sysid::GetResource_sysid_64_png());
  gui::AddIcon(sysid::GetResource_sysid_128_png());
  gui::AddIcon(sysid::GetResource_sysid_256_png());
  gui::AddIcon(sysid::GetResource_sysid_512_png());

  // Add messages from the global sysid logger into the Log window.
  gLogger.SetLogger([](unsigned int level, const char* file, unsigned int line,
                       const char* msg) {
    const char* lvl = "";
    if (level >= wpi::WPI_LOG_CRITICAL) {
      lvl = "CRITICAL: ";
    } else if (level >= wpi::WPI_LOG_ERROR) {
      lvl = "ERROR: ";
    } else if (level >= wpi::WPI_LOG_WARNING) {
      lvl = "WARNING: ";
    } else if (level >= wpi::WPI_LOG_INFO) {
      lvl = "INFO: ";
    }
    std::string filename = fs::path{file}.filename().string();
    gLog.Append(fmt::format("{}{} ({}:{})\n", lvl, msg, filename, line));
  });

  // Set the number of workers for the libuv threadpool.
  uv_os_setenv("UV_THREADPOOL_SIZE", "6");

  // Initialize libssh.
  ssh_init();

  // Initialize window manager and add views.
  gWindowManager = std::make_unique<glass::WindowManager>("SysId");
  gWindowManager->GlobalInit();

  gLoggerWindow = gWindowManager->AddWindow(
      "Logger", std::make_unique<sysid::Logger>(gLogger));

  gAnalyzerWindow = gWindowManager->AddWindow(
      "Analyzer", std::make_unique<sysid::Analyzer>(gLogger));

  gProgramLogWindow = gWindowManager->AddWindow(
      "Program Log", std::make_unique<glass::LogView>(&gLog));

  gGeneratorWindow = gWindowManager->AddWindow(
      "Generator", std::make_unique<sysid::Generator>(gLogger));

  // Set default positions and sizes for windows.
  gGeneratorWindow->SetDefaultPos(5, 25);
  gGeneratorWindow->SetDefaultSize(350, 255);

  gLoggerWindow->SetDefaultPos(5, 285);
  gLoggerWindow->SetDefaultSize(350, 400);

  gAnalyzerWindow->SetDefaultPos(360, 25);
  gAnalyzerWindow->SetDefaultSize(530, 530);

  gProgramLogWindow->SetDefaultPos(360, 560);
  gProgramLogWindow->SetDefaultSize(530, 125);
  gProgramLogWindow->DisableRenamePopup();

  gJSONConverter = std::make_unique<sysid::JSONConverter>(gLogger);

  // Configure save file.
  gui::ConfigurePlatformSaveFile("sysid.ini");

  // Add menu bar.
  gui::AddLateExecute([] {
    ImGui::BeginMainMenuBar();
    gui::EmitViewMenu();

    if (ImGui::BeginMenu("Widgets")) {
      gWindowManager->DisplayMenu();
      ImGui::EndMenu();
    }

    bool about = false;
    if (ImGui::BeginMenu("Info")) {
      if (ImGui::MenuItem("About")) {
        about = true;
      }
      ImGui::EndMenu();
    }

    bool frcCharConvert = false;
    bool toCSV = false;
    if (ImGui::BeginMenu("JSON Converters")) {
      if (ImGui::MenuItem("FRC-Char Converter")) {
        frcCharConvert = true;
      }
      if (ImGui::MenuItem("JSON to CSV Converter")) {
        toCSV = true;
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();

    if (frcCharConvert) {
      ImGui::OpenPopup("FRC-Char Converter");
      frcCharConvert = false;
    }

    if (toCSV) {
      ImGui::OpenPopup("SysId JSON to CSV Converter");
      toCSV = false;
    }

    if (ImGui::BeginPopupModal("FRC-Char Converter")) {
      gJSONConverter->DisplayFRCCharConvert();
      if (ImGui::Button("Close")) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    if (ImGui::BeginPopupModal("SysId JSON to CSV Converter")) {
      gJSONConverter->DisplayCSVConvert();
      if (ImGui::Button("Close")) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    if (about) {
      ImGui::OpenPopup("About");
      about = false;
    }
    if (ImGui::BeginPopupModal("About")) {
      ImGui::Text("SysId: System Identification for Robot Mechanisms");
      ImGui::Separator();
      ImGui::Text("v%s", GetWPILibVersion());
      if (ImGui::Button("Close")) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
  });

  gui::Initialize("System Identification", 1280, 720);
  gui::Main();

  ssh_finalize();

  glass::DestroyContext();
  gui::DestroyContext();
}

#endif
