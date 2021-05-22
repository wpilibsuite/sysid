// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/ArmSim.h"

#include <cmath>

#include <frc/StateSpaceUtil.h>
#include <frc/system/NumericalIntegration.h>
#include <wpi/MathExtras.h>

using namespace sysid;

ArmSim::ArmSim(double Ks, double Kv, double Ka, double Kcos,
               double initialPosition, double initialVelocity) {
  // u = Ks sgn(x) + Kv x + Ka a + Kcos cos(theta)
  // Ka a = u - Ks sgn(x) - Kv x - Kcos cos(theta)
  // a = 1/Ka u - Ks/Ka sgn(x) - Kv/Ka x - Kcos/Ka cos(theta)
  // a = -Kv/Ka x + 1/Ka u - Ks/Ka sgn(x) - Kcos/Ka cos(theta)
  // a = Ax + Bu + c sgn(x) + d cos(theta)
  Reset(initialPosition, initialVelocity);
  m_A << -Kv / Ka;
  m_B << 1.0 / Ka;
  m_c << -Ks / Ka;
  m_d << -Kcos / Ka;
}

void ArmSim::Update(units::volt_t voltage, units::second_t dt) {
  // Returns arm acceleration under gravity
  auto f =
      [=](const Eigen::Matrix<double, 2, 1>& x,
          const Eigen::Matrix<double, 1, 1>& u) -> Eigen::Matrix<double, 2, 1> {
    return frc::MakeMatrix<2, 1>(
        x(1), (m_A * x.block<1, 1>(1, 0) + m_B * u + m_c * wpi::sgn(x(1)) +
               m_d * std::cos(x(0)))(0));
  };

  m_x = frc::RKF45(f, m_x, frc::MakeMatrix<1, 1>(voltage.to<double>()), dt);
}

double ArmSim::GetPosition() const {
  return m_x(0);
}

double ArmSim::GetVelocity() const {
  return m_x(1);
}

double ArmSim::GetAcceleration(units::volt_t voltage) const {
  auto u = frc::MakeMatrix<1, 1>(voltage.to<double>());
  return (m_A * m_x.block<1, 1>(1, 0) + m_B * u +
          m_c * wpi::sgn(GetVelocity()) + m_d * std::cos(m_x(0)))(0);
}

void ArmSim::Reset(double position, double velocity) {
  m_x << position, velocity;
}
