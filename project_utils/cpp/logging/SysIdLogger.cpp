// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "logging/SysIdLogger.h"

#include <array>
#include <cstddef>
#include <sstream>
#include <stdexcept>

#include <frc/Notifier.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/Threads.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/Timer.h>

SysIdLogger::SysIdLogger() : m_data(kDataVectorSize) {
  frc::LiveWindow::GetInstance()->DisableAllTelemetry();
  frc::SmartDashboard::PutNumber("SysIdVoltageCommand", 0.0);
  frc::SmartDashboard::PutString("SysIdTestType", "");
  frc::SmartDashboard::PutString("SysIdTelemetry", "");
  frc::SmartDashboard::PutBoolean("SysIdRotate", false);
}

void SysIdLogger::InitLogging() {
  m_rotate = frc::SmartDashboard::GetBoolean("SysIdRotate", false);
  m_voltageCommand = frc::SmartDashboard::GetNumber("SysIdVoltageCommand", 0.0);
  m_testType = frc::SmartDashboard::GetString("SysIdTestType", "");
  m_startTime = frc2::Timer::GetFPGATimestamp().to<double>();
}

void SysIdLogger::SendData() {
  wpi::outs() << "Collected: " << m_data.size() << " Data points.\n";
  wpi::outs().flush();

  std::stringstream ss;
  for (const auto& pt : m_data) {
    ss << std::to_string(pt) << ", ";
  }

  frc::SmartDashboard::PutString("SysIdTelemetry", ss.str());

  // Clear everything after test
  m_data.clear();
  m_motorVoltage = 0.0;
  m_timestamp = 0.0;
  m_startTime = 0.0;
}

void SysIdLogger::UpdateThreadPriority() {
  if constexpr (!frc::RobotBase::IsSimulation()) {
    if (!frc::Notifier::SetHALThreadPriority(true, kHALThreadPriority) ||
        !frc::SetCurrentThreadPriority(true, kThreadPriority)) {
      throw std::runtime_error("Setting the RT Priority failed\n");
    }
  }
}

void SysIdLogger::UpdateData() {
  m_timestamp = frc2::Timer::GetFPGATimestamp().to<double>();
  if (m_testType == "Quasistatic") {
    m_motorVoltage = m_voltageCommand * (m_timestamp - m_startTime);
  } else if (m_testType == "Dynamic") {
    m_motorVoltage = m_voltageCommand;
  } else {
    m_motorVoltage = 0.0;
  }
}
