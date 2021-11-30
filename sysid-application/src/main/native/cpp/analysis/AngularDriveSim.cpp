// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/AngularDriveSim.h"

#include <frc/StateSpaceUtil.h>
#include <frc/system/Discretization.h>
#include <wpi/MathExtras.h>

using namespace sysid;

AngularDriveSim::AngularDriveSim(double Ks, double Kv, double Ka,
                                 double trackwidth, double initialPosition,
                                 double initialVelocity)
    // To convert to an angular version, ω = v / r
    : m_A{{0.0, 2.0 / trackwidth}, {0.0, -Kv / Ka}},
      m_B{0.0, 2.0 / trackwidth / Ka},
      m_c{0.0, -2.0 / trackwidth * Ks / Ka},
      m_trackwidth{trackwidth} {
  Reset(initialPosition, initialVelocity);
}

void AngularDriveSim::Update(units::volt_t voltage, units::second_t dt) {
  Eigen::Vector<double, 1> u{voltage.value()};

  // Given dx/dt = Ax + Bu + c sgn(x),
  // x_k+1 = e^(AT) x_k + A^-1 (e^(AT) - 1) (Bu + c sgn(x))
  Eigen::Matrix<double, 2, 2> Ad;
  Eigen::Matrix<double, 2, 1> Bd;
  frc::DiscretizeAB<2, 1>(m_A, m_B, dt, &Ad, &Bd);
  m_x = Ad * m_x + Bd * u +
        Bd * m_B.householderQr().solve(m_c * wpi::sgn(GetVelocity()));
}

double AngularDriveSim::GetPosition() const {
  return m_x(0) * m_trackwidth / 2;
}

double AngularDriveSim::GetVelocity() const {
  // The below comes from v = ω * r.
  return m_x(1) * m_trackwidth / 2;
}

double AngularDriveSim::GetAcceleration(units::volt_t voltage) const {
  Eigen::Vector<double, 1> u{voltage.value()};
  return (m_A * m_x + m_B * u + m_c * wpi::sgn(GetVelocity()))(1);
}

void AngularDriveSim::Reset(double position, double velocity) {
  m_x = Eigen::Vector<double, 2>{position, velocity};
}
