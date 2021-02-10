// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/analysis/FeedbackAnalysis.h"

#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "sysid/analysis/FeedbackControllerPreset.h"

using namespace sysid;

using Kv_t = decltype(1_V / 1_mps);
using Ka_t = decltype(1_V / 1_mps_sq);

std::tuple<double, double> sysid::CalculatePositionFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains) {
  // Get Ks, Kv, and Ka from the tuple.
  auto& [Ks, Kv, Ka] = feedforwardGains;
  static_cast<void>(Ks);

  // If acceleration requires no effort, velocity becomes an input for position
  // control. We choose an appropriate model in this case to avoid numerical
  // instabilities in the LQR.
  if (Ka > 1E-7) {
    // Create a position system from our feedforward gains.
    auto system = frc::LinearSystemId::IdentifyPositionSystem<units::meter>(
        Kv_t(Kv), Ka_t(Ka));
    // Create an LQR with 2 states to control -- position and velocity.
    frc::LinearQuadraticRegulator<2, 1> controller{
        system, {params.qp, params.qv}, {params.r}, preset.period};
    // Compensate for any latency from sensor measurements, filtering, etc.
    controller.LatencyCompensate(system, preset.period,
                                 preset.positionMeasurementDelay);

    return std::make_tuple(
        controller.K(0, 0) * preset.outputConversionFactor,
        controller.K(0, 1) * preset.outputConversionFactor /
            (preset.normalized ? 1 : preset.period.to<double>()));
  }

  // This is our special model to avoid instabilities in the LQR.
  auto system = frc::LinearSystem<1, 1, 1>(
      frc::MakeMatrix<1, 1>(0.0), frc::MakeMatrix<1, 1>(1.0),
      frc::MakeMatrix<1, 1>(1.0), frc::MakeMatrix<1, 1>(0.0));
  // Create an LQR with one state -- position.
  frc::LinearQuadraticRegulator<1, 1> controller{
      system, {params.qp}, {params.r}, preset.period};
  // Compensate for any latency from sensor measurements, filtering, etc.
  controller.LatencyCompensate(system, preset.period,
                               preset.positionMeasurementDelay);

  return std::make_tuple(
      Kv * controller.K(0, 0) * preset.outputConversionFactor, 0);
}

std::tuple<double, double> sysid::CalculateVelocityFeedbackGains(
    const FeedbackControllerPreset& preset, const LQRParameters& params,
    const FeedforwardGains& feedforwardGains) {
  // Get Ks, Kv, and Ka from the tuple.
  auto& [Ks, Kv, Ka] = feedforwardGains;
  static_cast<void>(Ks);

  // If acceleration for velocity control requires no effort, the feedback
  // control gains approach zero. We special-case it here because numerical
  // instabilities arise in LQR otherwise.
  if (Ka < 1E-7) {
    return std::make_tuple(0, 0);
  }

  // Create a velocity system from our feedforward gains.
  auto system = frc::LinearSystemId::IdentifyVelocitySystem<units::meter>(
      Kv_t(Kv), Ka_t(Ka));
  // Create an LQR controller with 1 state -- velocity.
  frc::LinearQuadraticRegulator<1, 1> controller{
      system, {params.qv}, {params.r}, preset.period};
  // Compensate for any latency from sensor measurements, filtering, etc.
  controller.LatencyCompensate(system, preset.period,
                               preset.velocityMeasurementDelay);

  return std::make_tuple(controller.K(0, 0) * preset.outputConversionFactor, 0);
}
