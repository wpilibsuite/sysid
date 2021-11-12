// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/view/Generator.h"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>

#include <fmt/format.h>
#include <glass/Context.h>
#include <imgui_stdlib.h>
#include <wpi/Logger.h>
#include <wpi/StringExtras.h>

#include "sysid/Util.h"
#include "sysid/analysis/AnalysisType.h"
#include "sysid/generation/ConfigManager.h"

using namespace sysid;

Generator::Generator(wpi::Logger& logger) : m_logger{logger} {
  // Create configuration manager to generate JSONs.
  m_manager = std::make_unique<ConfigManager>(m_settings, m_logger);

  // Initialize persistent storage and assign pointers.
  auto& storage = glass::GetStorage();
  m_pUnitsPerRotation = storage.GetDoubleRef("Units Per Rotation", 1.0);
  m_pAnalysisType = storage.GetStringRef("Analysis Type", "Simple");

  // Initialize the deploy logger. First set the min log level so that debug
  // messages are not swallowed.
  m_deployLogger.set_min_level(wpi::WPI_LOG_DEBUG);

  // Set the on-log action.
  m_deployLogger.SetLogger([this](unsigned int level, const char* file,
                                  unsigned int line, const char* msg) {
    // Append the message and level to the log.
    std::scoped_lock lock{m_deployMutex};
    m_deployLog.push_back({msg, level});
  });

  // Initialize team number / IP field.
  m_pTeam =
      glass::GetStorage("NetworkTables Settings").GetStringRef("serverTeam");
}

/**
 * Finds the index of a certain string in a C-style array
 *
 * @tparam X The size of the array
 *
 * @param arr The array to search through
 * @param value The string value to find
 *
 * @return The index of the string value
 */
template <size_t X>
static int GetNewIdx(const char* const (&arr)[X], std::string_view value) {
  auto it = std::find(std::cbegin(arr), std::cend(arr), value);
  if (it == std::cend(arr)) {
    throw std::runtime_error(fmt::format("{} not found", value));
  } else {
    return std::distance(std::cbegin(arr), it);
  }
}

/**
 * Finds the index of a certain string in a standard array
 *
 * @tparam X The size of the array
 *
 * @param arr The array to search through
 * @param value The string value to find
 *
 * @return The index of the string value
 */
template <size_t X>
static int GetNewIdx(const std::array<const char*, X>& arr,
                     std::string_view value) {
  auto it = std::find(std::begin(arr), std::end(arr), value);
  if (it == std::cend(arr)) {
    throw std::runtime_error(fmt::format("{} not found", value));
  } else {
    return std::distance(std::cbegin(arr), it);
  }
}

void Generator::RoboRIOEncoderSetup(bool drive) {
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
  ImGui::InputInt("A##1", &m_settings.primaryEncoderPorts[0], 0, 0);
  ImGui::SameLine(ImGui::GetFontSize() * 4);

  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
  ImGui::InputInt("B##1", &m_settings.primaryEncoderPorts[1], 0, 0);
  ImGui::SameLine();
  ImGui::Checkbox(drive ? "Left Encoder Inverted" : "Encoder Inverted",
                  &m_settings.primaryEncoderInverted);

  if (drive) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("A##2", &m_settings.secondaryEncoderPorts[0], 0, 0);
    ImGui::SameLine(ImGui::GetFontSize() * 4);

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("B##2", &m_settings.secondaryEncoderPorts[1], 0, 0);

    ImGui::SameLine();
    ImGui::Checkbox("Right Encoder Inverted",
                    &m_settings.secondaryEncoderInverted);
  }

  ImGui::Checkbox("Reduce Encoding", &m_settings.encoding);
  CreateTooltip(
      "This helps reduce encoder noise for high CPR encoders such as the "
      "CTRE Magnetic Encoder and REV Throughbore Encoder");
}

void Generator::CANCoderSetup(bool drive) {
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
  ImGui::InputInt(drive ? "L CANCoder Port" : "CANCoder Port",
                  &m_settings.primaryEncoderPorts[0], 0, 0);
  ImGui::SameLine();
  ImGui::Checkbox(drive ? "Left Encoder Inverted" : "Encoder Inverted",
                  &m_settings.primaryEncoderInverted);
  if (drive) {
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("R CANCoder Port", &m_settings.secondaryEncoderPorts[1], 0,
                    0);
    ImGui::SameLine();
    ImGui::Checkbox("Right Encoder Inverted",
                    &m_settings.secondaryEncoderInverted);
  }
}

void Generator::RegularEncoderSetup(bool drive) {
  ImGui::Checkbox(drive ? "Left Encoder Inverted" : "Encoder Inverted",
                  &m_settings.primaryEncoderInverted);
  if (drive) {
    ImGui::SameLine();
    ImGui::Checkbox("Right Encoder Inverted",
                    &m_settings.secondaryEncoderInverted);
  }
}

void Generator::UpdateFromConfig() {
  // Get Occupied Ports
  m_occupied = m_settings.motorControllers.size();

  // Idxs
  std::string_view mainMotorController{m_settings.motorControllers[0]};
  if (wpi::starts_with(mainMotorController, "Talon")) {
    if (mainMotorController == "TalonFX") {
      m_encoderIdx = GetNewIdx(ArrayConcat(kBuiltInEncoders, kGeneralEncoders),
                               m_settings.encoderType);
    } else {
      m_encoderIdx = GetNewIdx(ArrayConcat(kTalonSRXEncoders, kGeneralEncoders),
                               m_settings.encoderType);
    }

  } else if (wpi::starts_with(mainMotorController, "SPARK MAX")) {
    m_encoderIdx = GetNewIdx(ArrayConcat(kSparkMaxEncoders, kGeneralEncoders),
                             m_settings.encoderType);
  } else if (mainMotorController == "Venom") {
    m_encoderIdx = GetNewIdx(ArrayConcat(kBuiltInEncoders, kGeneralEncoders),
                             m_settings.encoderType);
  } else {
    m_encoderIdx = GetNewIdx(kGeneralEncoders, m_settings.encoderType);
  }

  m_gyroIdx = GetNewIdx(kGyros, m_settings.gyro);
  m_periodIdx = GetNewIdx(kCTREPeriods, std::to_string(m_settings.period));

  // Read in Gyro Constructors
  if (m_settings.gyro == "Pigeon") {
    auto pos = m_settings.gyroCtor.find("WPI_TalonSRX");
    if (pos != std::string_view::npos) {
      m_gyroPort = std::stoi(m_settings.gyroCtor.substr(pos + 13));
      m_isTalon = true;
    } else {
      m_gyroPort = std::stoi(m_settings.gyroCtor);
      m_isTalon = false;
    }
  } else if (m_settings.gyro == "ADXRS450") {
    m_gyroParam = GetNewIdx(sysid::kADXRS450Ctors, m_settings.gyroCtor);
  } else if (m_settings.gyro == "NavX") {
    m_gyroParam = GetNewIdx(sysid::kNavXCtors, m_settings.gyroCtor);
  } else {
    m_gyroPort = std::stoi(m_settings.gyroCtor);
  }

  // Set Analysis Type
  m_analysisIdx = m_settings.isDrive ? 1 : 0;
}

void Generator::Display() {
  // Add team / IP selection.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
  ImGui::InputText("Team/IP", m_pTeam);

  // Add Config Reading Button
  if (ImGui::Button("Load Config")) {
    m_loadConfigSelector = std::make_unique<pfd::open_file>(
        "Open Config", "",
        std::vector<std::string>{"JSON File", SYSID_PFD_JSON_EXT});
  }

  m_settings.isDrive = m_analysisIdx > 0;

  // Add deploy button.
  // FIXME: Open Romi project in simulation once desktop programs are supported.
  ImGui::SameLine();
  if (ImGui::Button("Deploy")) {
    // Create the deploy session,
    m_deploySession = std::make_unique<DeploySession>(
        *m_pTeam, m_analysisIdx == 1, m_manager->Generate(m_occupied),
        m_deployLogger);

    // Execute the deploy.
    m_deployRunner.ExecSync(
        [this](wpi::uv::Loop& lp) { m_deploySession->Execute(lp); });

    // Open the deploy popup.
    ImGui::OpenPopup("Deploy Status");
  }
  // Add analysis type selection.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
  ImGui::Combo("Analysis Type", &m_analysisIdx, kAnalysisTypes,
               IM_ARRAYSIZE(kAnalysisTypes));
  *m_pAnalysisType = kAnalysisTypes[m_analysisIdx];

  // If we are a Romi project, we can end here because there is no generation to
  // be done.
  if (*m_pAnalysisType == "Romi") {
    return;
  }

  // Add section for motor and motor controller selection.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Motor / Motor Controller Selection");

  // Add buttons to add and/or remove motor ports.
  ImGui::SameLine();
  if (ImGui::Button("+")) {
    m_occupied++;
  }
  if (m_occupied > 1) {
    ImGui::SameLine();
    if (ImGui::Button("-")) {
      m_occupied--;
    }
  }

  // Add motor port selection.
  bool drive = *m_pAnalysisType == "Drivetrain";

  // Iterate through the number of ports we have available (from m_occupied) and
  // add UI elements.
  for (size_t i = 0; i < m_occupied; ++i) {
    // Create aliases so it's easier to work on vectors.
    auto& pm = m_settings.primaryMotorPorts;
    auto& sm = m_settings.secondaryMotorPorts;
    auto& mc = m_settings.motorControllers;
    auto& pi = m_settings.primaryMotorsInverted;
    auto& si = m_settings.secondaryMotorsInverted;

    // Ensure that our vector contains i+1 elements.
    if (pm.size() == i) {
      pm.emplace_back((drive ? 2 : 1) * i);
      sm.emplace_back(pm.size() + i);
      mc.emplace_back(kMotorControllers[0]);
      pi.emplace_back(false);
      si.emplace_back(false);
    }

    // Make sure elements have unique IDs.
    ImGui::Spacing();
    ImGui::PushID(i);

    // Add primary (left for drivetrain) motor ports.
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    auto primaryName =
        fmt::format(drive ? "L Motor Port {}" : "Motor Port {}", i);
    ImGui::InputInt(primaryName.c_str(), &pm[i], 0, 0);

    // Add inverted setting.
    ImGui::SameLine();
    ImGui::Checkbox(drive ? "L Inverted" : "Inverted", &pi[i]);

    // Add right side drivetrain ports (if the analysis type is drivetrain).
    if (drive) {
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
      auto secondaryName = fmt::format("R Motor Port {}", i);
      ImGui::InputInt(secondaryName.c_str(), &sm[i], 0, 0);

      // Add inverted setting.
      ImGui::SameLine();
      ImGui::Checkbox("R Inverted", &si[i]);
    }
    // Add motor controller selector.
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
    auto motorControllerName = fmt::format(
        drive ? "Motor Controller Pair {}" : "Motor Controller {}", i);

    if (ImGui::BeginCombo(motorControllerName.c_str(), mc[i].c_str())) {
      for (size_t n = 0; n < IM_ARRAYSIZE(kMotorControllers); ++n) {
        bool selected = mc[i] == kMotorControllers[n];
        if (ImGui::Selectable(kMotorControllers[n], selected)) {
          mc[i] = kMotorControllers[n];
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::PopID();

    // If we selected Spark Max with Brushed mode, set our flag to true.
    m_isSparkMaxBrushed = mc[i] == "SPARK MAX (Brushed)";
  }

  // Add section for encoders.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Encoder Selection");

  // Reset Encoder Selection if a new motor controller is chosen
  std::string_view mainMotorController{m_settings.motorControllers[0]};
  if (m_prevMainMotorController != mainMotorController) {
    m_encoderIdx = 0;
  }
  m_prevMainMotorController = mainMotorController;

  // Add encoder selection.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
  if (wpi::starts_with(mainMotorController, "Talon")) {
    if (mainMotorController == "TalonFX") {
      GetEncoder(ArrayConcat(kBuiltInEncoders, kGeneralEncoders));
    } else {
      GetEncoder(ArrayConcat(kTalonSRXEncoders, kGeneralEncoders));
    }
    if (m_encoderIdx <= 1) {
      RegularEncoderSetup(drive);
    } else if (m_encoderIdx == 2) {
      CANCoderSetup(drive);
    } else {
      RoboRIOEncoderSetup(drive);
    }
  } else if (wpi::starts_with(mainMotorController, "SPARK MAX")) {
    GetEncoder(ArrayConcat(kSparkMaxEncoders, kGeneralEncoders));
    if (m_encoderIdx <= 1) {
      if (m_encoderIdx == 1 || wpi::contains(mainMotorController, "Brushed")) {
        // You're not allowed to invert the NEO encoder
        RegularEncoderSetup(drive);
      }
    } else if (m_encoderIdx == 2) {
      CANCoderSetup(drive);
    } else {
      RoboRIOEncoderSetup(drive);
    }
  } else if (mainMotorController == "Venom") {
    GetEncoder(ArrayConcat(kBuiltInEncoders, kGeneralEncoders));
    if (m_encoderIdx == 0) {
      RegularEncoderSetup(drive);
    } else if (m_encoderIdx == 1) {
      CANCoderSetup(drive);
    } else {
      RoboRIOEncoderSetup(drive);
    }
  } else {
    GetEncoder(kGeneralEncoders);
    if (m_encoderIdx == 0) {
      CANCoderSetup(drive);
    } else {
      RoboRIOEncoderSetup(drive);
    }
  }

  // Venom built-in encoders can't change sampling or measurement period.
  if (!(mainMotorController == "Venom" &&
        m_settings.encoderType == "Built-in")) {
    // Samples Per Average Setting
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 2);
    ImGui::InputInt("Samples Per Average", &m_settings.numSamples, 0, 0);
    CreateTooltip(
        "This helps reduce encoder noise by averaging collected samples "
        "together. A value from 5-10 is reccomended for encoders with high "
        "CPRs.");

    // Add Velocity Measurement Period
    if (m_settings.encoderType != "roboRIO") {
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
      ImGui::Combo("Time Measurement Window", &m_periodIdx, kCTREPeriods,
                   IM_ARRAYSIZE(kCTREPeriods));
      m_settings.period = std::stoi(std::string{kCTREPeriods[m_periodIdx]});
    }
  }

  // Add gyro selection if selected is drivetrain.
  if (drive) {
    // Create section for gyros.
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Text("Gyro");

    // Add gyro combo box.
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::Combo("Gyro", &m_gyroIdx, kGyros, IM_ARRAYSIZE(kGyros));

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);

    std::string_view gyroType{kGyros[m_gyroIdx]};

    // Handle each gyro and its special cases.
    if (gyroType == "Pigeon") {
      ImGui::InputInt("Gyro Parameter", &m_gyroPort, 0, 0);
      ImGui::SameLine();
      ImGui::Checkbox("Is Talon", &m_isTalon);
      CreateTooltip(
          "Check this checkbox if the Pigeon is hooked up to a TalonSRX");

      m_settings.gyroCtor = std::to_string(m_gyroPort);

      // Indicate Gyro is connected to TalonSRX
      if (m_isTalon) {
        m_settings.gyroCtor = "WPI_TalonSRX-" + m_settings.gyroCtor;
      }
    } else if (gyroType == "ADXRS450") {
      ImGui::Combo("Gyro Parameter", &m_gyroParam, kADXRS450Ctors,
                   IM_ARRAYSIZE(kADXRS450Ctors));
      m_settings.gyroCtor = std::string(kADXRS450Ctors[m_gyroParam]);
    } else if (gyroType == "NavX") {
      ImGui::Combo("Gyro Parameter", &m_gyroParam, kNavXCtors,
                   IM_ARRAYSIZE(kNavXCtors));
      m_settings.gyroCtor = std::string(kNavXCtors[m_gyroParam]);
    } else {
      ImGui::InputInt("Gyro Parameter", &m_gyroPort, 0, 0);

      // AnalogGyro port cannot be greater than 1.
      if (m_gyroPort > 1) {
        m_gyroPort = 1;
      }
      m_settings.gyroCtor = std::to_string(m_gyroPort);
    }

    // Avoid accessing bad gyro ctor indices.
    if (gyroType != m_settings.gyro) {
      m_gyroParam = 0;
    }
    m_settings.gyro = gyroType;
  }

  // Add section for other parameters.
  ImGui::Separator();
  ImGui::Spacing();
  ImGui::Text("Encoder Parameters");

  // Add encoder resolution.
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
  ImGui::InputDouble("Counts Per Revolution", &m_settings.cpr, 0.0, 0.0,
                     "%.2f");
  sysid::CreateTooltip(
      "This is the number of encoder counts per revolution for your encoder.\n"
      "Common values for this are here:\nCTRE Magnetic Encoder: 4096\nFalcon "
      "500 Integrated: 2048\nREV Throughbore: 8192\nNEO (and NEO 550) "
      "Integrated "
      "Encoders (REV already handles this value): 1");

  // Add gearing
  ImGui::InputDouble("Gearing", &m_settings.gearing, 0.0, 0.0, "%.2f");
  sysid::CreateTooltip(
      "This is the gearing between the encoder and the output shaft. For "
      "example, if the encoder is mounted to the magnetic shaft on the kit "
      "chassis, there is no gearing in between the encoder and the output "
      "shaft hence the gearing is 1. However, if the encoder was an integrated "
      "encoder on the motor in the kit chassis gearbox, the gearing would be "
      "10.71.");
  // Have a save button at the bottom
  if (ImGui::Button("Save")) {
    // Open a file popup
    m_saveConfigSelector = std::make_unique<pfd::save_file>(
        "Save Config", "config.json",
        std::vector<std::string>{"JSON File", SYSID_PFD_JSON_EXT},
        pfd::opt::force_path);
  }

  if (m_saveConfigSelector && m_saveConfigSelector->ready() &&
      !m_saveConfigSelector->result().empty()) {
    auto path = m_saveConfigSelector->result();
    m_manager->SaveJSON(path, m_occupied);
    m_saveConfigSelector.reset();
  }

  if (m_loadConfigSelector && m_loadConfigSelector->ready() &&
      !m_loadConfigSelector->result().empty()) {
    auto path = m_loadConfigSelector->result()[0];
    m_manager->ReadJSON(path);
    UpdateFromConfig();
    m_loadConfigSelector.reset();
  }

  // Define the deploy popup (and set default size).
  auto size = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowSize(ImVec2(size.x / 2, size.y * 0.8));

  if (ImGui::BeginPopupModal("Deploy Status")) {
    static ImVec4 kColorWarning{1.0f, 0.7f, 0.0f, 1.0f};
    static ImVec4 kColorError{1.0f, 0.4f, 0.4f, 1.0f};
    static ImVec4 kColorSuccess{0.2f, 1.0f, 0.2f, 1.0f};
    static ImVec4 kColorDebug{0.5f, 0.5f, 0.5f, 1.0f};

    // We are accessing a shared resource (deploy log), so lock mutex.
    std::scoped_lock lock{m_deployMutex};

    // Check whether the user is using SPARK MAX in Brushed mode. Display a
    // warning message if so.
    if (m_isSparkMaxBrushed) {
      ImGui::TextColored(
          kColorWarning,
          "You have selected SPARK MAX (Brushed)!\nMake sure that you "
          "are controlling a BRUSHED motor (not NEO / NEO 550)!");
    }

    // Get the deploy status.
    auto status = m_deploySession->GetStatus();

    // If there are no messages in the log and we are in progress, then we are
    // still discovering the roboRIO.
    if (status == DeploySession::Status::kInProgress && m_deployLog.empty()) {
      ImGui::Text("Discovering roboRIO %c",
                  "|/-\\"[static_cast<int>(ImGui::GetTime() / 0.05f) & 3]);
    }

    // Show log messages from the event loop runner.
    for (auto&& message : m_deployLog) {
      ImVec4 color{1.0f, 1.0f, 1.0f, 1.0f};
      switch (message.level) {
        case kLogSuccess:
          color = kColorSuccess;
          break;
        case wpi::WPI_LOG_ERROR:
          color = kColorError;
          break;
        case wpi::WPI_LOG_DEBUG:
          color = kColorDebug;
          break;
      }
      ImGui::TextColored(color, "%s", message.message.c_str());
    }

    // Show error if we had a discovery failure.
    if (status == DeploySession::Status::kDiscoveryFailure) {
      ImGui::TextColored(kColorError,
                         "Could not discover roboRIO.\nAre you connected to "
                         "the robot and is it on?");
    }

    // Check if we are done with the deploy. If we are, then we can show the
    // close button.
    if (status == DeploySession::Status::kDiscoveryFailure ||
        status == DeploySession::Status::kDone) {
      if (ImGui::Button("Close")) {
        m_deploySession.reset();
        m_deployLog.clear();
        ImGui::CloseCurrentPopup();
      }
    }
    ImGui::EndPopup();
  }
}
