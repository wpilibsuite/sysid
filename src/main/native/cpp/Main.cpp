// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef RUNNING_SYSID_TESTS

#include <memory>

#include <glass/Context.h>
#include <glass/Window.h>
#include <glass/WindowManager.h>
#include <glass/other/Log.h>
#include <wpi/Logger.h>
#include <wpi/Path.h>
#include <wpigui.h>

#include "sysid/view/Analyzer.h"
#include "sysid/view/Logger.h"

namespace gui = wpi::gui;

static std::unique_ptr<glass::WindowManager> gWindowManager;

glass::Window* gLoggerWindow;
glass::Window* gAnalyzerWindow;
glass::Window* gGeneratorWindow;
glass::Window* gProgramLogWindow;

glass::LogData gLog;
wpi::Logger gLogger;

const char* GetWPILibVersion();

#ifdef _WIN32
int __stdcall WinMain(void* hInstance, void* hPrevInstance, char* pCmdLine,
                      int nCmdShow) {
#else
int main() {
#endif
  // Create the wpigui (along with Dear ImGui) and Glass contexts.
  gui::CreateContext();
  glass::CreateContext();

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
    const char* filename = wpi::sys::path::filename(file).data();

    gLog.Append(wpi::Twine{lvl} + msg + wpi::Twine{" ("} + filename +
                wpi::Twine{':'} + wpi::Twine{line} + wpi::Twine{")\n"});
  });

  // Initialize window manager and add views.
  gWindowManager = std::make_unique<glass::WindowManager>("SysId");
  gWindowManager->GlobalInit();

  gLoggerWindow = gWindowManager->AddWindow(
      "Logger", std::make_unique<sysid::Logger>(gLogger));

  gAnalyzerWindow = gWindowManager->AddWindow(
      "Analyzer", std::make_unique<sysid::Analyzer>(gLogger));

  gProgramLogWindow = gWindowManager->AddWindow(
      "Program Log", std::make_unique<glass::LogView>(&gLog));

  // Set default positions and sizes for windows.
  gLoggerWindow->SetDefaultPos(35, 40);
  gLoggerWindow->SetDefaultSize(500, 420);

  gAnalyzerWindow->SetDefaultPos(550, 40);
  gAnalyzerWindow->SetDefaultSize(670, 530);

  gProgramLogWindow->SetDefaultPos(40, 580);
  gProgramLogWindow->SetDefaultSize(1180, 125);
  gProgramLogWindow->DisableRenamePopup();

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

    ImGui::EndMainMenuBar();

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

  glass::DestroyContext();
  gui::DestroyContext();
}

#endif  // RUNNING_SYSID_TESTS
