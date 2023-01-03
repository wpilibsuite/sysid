// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/logging/SysIdLogger.h"

#include <cstddef>
#include <sstream>
#include <stdexcept>

// #include <CANVenom.h>
#include <ctre/Phoenix.h>
#include <fmt/core.h>
#include <frc/Notifier.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/Threads.h>
#include <frc/Timer.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <wpi/StringExtras.h>

using namespace sysid;

void SysIdLogger::InitLogging() {
  m_mechanism = frc::SmartDashboard::GetString("SysIdTest", "");

  if (IsWrongMechanism()) {
    frc::SmartDashboard::PutBoolean("SysIdWrongMech", true);
  }

  m_testType = frc::SmartDashboard::GetString("SysIdTestType", "");
  m_rotate = frc::SmartDashboard::GetBoolean("SysIdRotate", false);
  m_voltageCommand = frc::SmartDashboard::GetNumber("SysIdVoltageCommand", 0.0);
  m_startTime = frc::Timer::GetFPGATimestamp().value();
  m_data.clear();
  frc::SmartDashboard::PutString("SysIdTelemetry", "");
  m_ackNum = frc::SmartDashboard::GetNumber("SysIdAckNumber", 0);
}

double SysIdLogger::MeasureVoltage(
    const std::vector<std::unique_ptr<frc::MotorController>>& controllers,
    const std::vector<std::string>& controllerNames) {
  double sum = 0.0;
  for (size_t i = 0; i < controllers.size(); ++i) {
    auto&& controller = controllers[i].get();
    if (wpi::starts_with(controllerNames[i], "SPARK MAX")) {
      auto* smax = static_cast<rev::CANSparkMax*>(controller);
      sum += smax->GetBusVoltage() * smax->GetAppliedOutput();
      if constexpr (frc::RobotBase::IsSimulation()) {
        fmt::print("Recording SPARK MAX voltage\n");
      }
    } else if (wpi::starts_with(controllerNames[i], "Talon") ||
               wpi::starts_with(controllerNames[i], "Victor")) {
      auto* ctreController = dynamic_cast<WPI_BaseMotorController*>(controller);
      sum += ctreController->GetMotorOutputVoltage();
      if constexpr (frc::RobotBase::IsSimulation()) {
        fmt::print("Recording CTRE voltage\n");
      }
    } else if (controllerNames[i] == "Venom") {
      // auto* venom = static_cast<frc::CANVenom*>(controller);
      // sum += venom->GetOutputVoltage();
      if constexpr (frc::RobotBase::IsSimulation()) {
        fmt::print("Recording Venom voltage\n");
      }
    } else {
      sum += controllers[i]->Get() *
             frc::RobotController::GetBatteryVoltage().value();
      if constexpr (frc::RobotBase::IsSimulation()) {
        fmt::print("Recording General voltage\n");
      }
    }
  }

  return sum / controllers.size();
}

void SysIdLogger::SendData() {
  fmt::print("Collected: {} data points.\n", m_data.size());

  frc::SmartDashboard::PutBoolean("SysIdOverflow",
                                  m_data.size() >= kDataVectorSize);

  std::stringstream ss;
  for (size_t i = 0; i < m_data.size(); ++i) {
    ss << std::to_string(m_data[i]);
    if (i < m_data.size() - 1) {
      ss << ",";
    }
  }

  std::string type = m_testType == "Dynamic" ? "fast" : "slow";
  std::string direction = m_voltageCommand > 0 ? "forward" : "backward";
  std::string test = fmt::format("{}-{}", type, direction);

  frc::SmartDashboard::PutString("SysIdTelemetry",
                                 fmt::format("{};{}", test, ss.str()));
  frc::SmartDashboard::PutNumber("SysIdAckNumber", ++m_ackNum);

  Reset();
}

void SysIdLogger::ClearWhenReceived() {
  if (frc::SmartDashboard::GetNumber("SysIdAckNumber", 0.0) > m_ackNum) {
    frc::SmartDashboard::PutString("SysIdTelemetry", "");
    m_ackNum = frc::SmartDashboard::GetNumber("SysIdAckNumber", 0.0);
  }
}

void SysIdLogger::UpdateThreadPriority() {
  if constexpr (!frc::RobotBase::IsSimulation()) {
    if (!frc::Notifier::SetHALThreadPriority(true, kHALThreadPriority) ||
        !frc::SetCurrentThreadPriority(true, kThreadPriority)) {
      throw std::runtime_error("Setting the RT Priority failed\n");
    }
  }
}

SysIdLogger::SysIdLogger() {
  fmt::print("Initializing logger\n");
  m_data.reserve(kDataVectorSize);
  frc::LiveWindow::DisableAllTelemetry();
  frc::SmartDashboard::PutNumber("SysIdVoltageCommand", 0.0);
  frc::SmartDashboard::PutString("SysIdTestType", "");
  frc::SmartDashboard::PutString("SysIdTest", "");
  frc::SmartDashboard::PutBoolean("SysIdRotate", false);
  frc::SmartDashboard::PutBoolean("SysIdOverflow", false);
  frc::SmartDashboard::PutBoolean("SysIdWrongMech", false);
  frc::SmartDashboard::PutNumber("SysIdAckNumber", m_ackNum);
}

void SysIdLogger::UpdateData() {
  m_timestamp = frc::Timer::GetFPGATimestamp().value();

  // Don't let robot move if it's characterizing the wrong mechanism
  if (!IsWrongMechanism()) {
    if (m_testType == "Quasistatic") {
      m_motorVoltage = m_voltageCommand * (m_timestamp - m_startTime);
    } else if (m_testType == "Dynamic") {
      m_motorVoltage = m_voltageCommand;
    } else {
      m_motorVoltage = 0.0;
    }
  } else {
    m_motorVoltage = 0.0;
  }
}

void SysIdLogger::Reset() {
  m_motorVoltage = 0.0;
  m_timestamp = 0.0;
  m_startTime = 0.0;
  m_data.clear();
}
