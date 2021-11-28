// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generation/SysIdSetup.h"

#include <stdexcept>

// #include <CANVenom.h>
#include <fmt/core.h>
#include <frc/Filesystem.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/Spark.h>
#include <wpi/SmallString.h>
#include <wpi/StringExtras.h>
#include <wpi/fs.h>

// Based on https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html
#define EXPAND_STRINGIZE(s) STRINGIZE(s)
#define STRINGIZE(s) #s

namespace sysid {

wpi::json GetConfigJson() {
  std::string path;

  if constexpr (frc::RobotBase::IsSimulation()) {
#if defined(PROJECT_ROOT_DIR) && defined(INTEGRATION)
    // TODO: Fix problems with this so that we don't need this ifdef
    path = fmt::format("{}/sysid-projects/deploy/config.json",
                       EXPAND_STRINGIZE(PROJECT_ROOT_DIR));
#endif
  } else {
    path = fmt::format("{}/config.json", frc::filesystem::GetDeployDirectory());
  }

  std::error_code ec;
  wpi::raw_fd_istream is{path, ec};
  if (ec) {
    fmt::print("File error: {}\n", path);
    throw std::runtime_error("Unable to read file");
  }

  wpi::json outJson;
  is >> outJson;
  return outJson;
}

void AddMotorController(
    int port, std::string_view controller, bool inverted,
    std::vector<std::unique_ptr<frc::MotorController>>* controllers) {
  if (controller == "TalonSRX" || controller == "VictorSPX" ||
      controller == "TalonFX") {
    if (controller == "TalonSRX") {
      fmt::print("Setup TalonSRX");
      //   controllers->push_back(std::make_unique<WPI_TalonSRX>(port));
    } else if (controller == "TalonFX") {
      fmt::print("Setup TalonFX");
      //   controllers->emplace_back(std::make_unique<WPI_TalonFX>(port));
    } else {
      fmt::print("Setup VictorSPX");
      //   controllers->emplace_back(std::make_unique<WPI_VictorSPX>(port));
    }

    // auto* ctreController =
    //     dynamic_cast<WPI_BaseMotorController*>(controllers->back().get());
    // ctreController->ConfigFactoryDefault();
    // ctreController->SetInverted(inverted);
    // ctreController->SetNeutralMode(motorcontrol::NeutralMode::Brake);
  } else if (controller == "SPARK MAX (Brushless)" ||
             controller == "SPARK MAX (Brushed)") {
    if (controller == "SPARK MAX (Brushless)") {
      fmt::print("Setup SPARK MAX (Brushless)");
      controllers->emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushless));
    } else {
      fmt::print("Setup SPARK MAX (Brushed)");
      controllers->emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushed));
    }

    auto* sparkMax = static_cast<rev::CANSparkMax*>(controllers->back().get());
    sparkMax->RestoreFactoryDefaults();
    sparkMax->SetInverted(inverted);
    sparkMax->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  } else if (controller == "Venom") {
    fmt::print("Setup Venom");
    //   controllers->emplace_back(std::make_unique<frc::CANVenom>(port));

    //   auto* venom = static_cast<frc::CANVenom*>(controllers->back().get());

    //   venom->SetInverted(inverted);
    //   venom->SetBrakeCoastMode(frc::CANVenom::BrakeCoastMode::kBrake);
  } else {
    fmt::print("Setup PWM");
    controllers->emplace_back(std::make_unique<frc::Spark>(port));
    auto* spark = static_cast<frc::Spark*>(controllers->back().get());
    spark->SetInverted(inverted);
  }
}

// static sensors::SensorVelocityMeasPeriod getCTREVelocityPeriod(int period) {
//   switch (period) {
//     case 1:
//       return sensors::SensorVelocityMeasPeriod::Period_1Ms;
//     case 2:
//       return sensors::SensorVelocityMeasPeriod::Period_2Ms;
//     case 5:
//       return sensors::SensorVelocityMeasPeriod::Period_5Ms;
//     case 10:
//       return sensors::SensorVelocityMeasPeriod::Period_10Ms;
//     case 25:
//       return sensors::SensorVelocityMeasPeriod::Period_25Ms;
//     case 50:
//       return sensors::SensorVelocityMeasPeriod::Period_50Ms;
//     default:
//       return sensors::SensorVelocityMeasPeriod::Period_100Ms;
//   }
// }

// static void SetupCTREEncoder(frc::MotorController* controller,
//                              FeedbackDevice feedbackDevice, int period,
//                              double cpr, int numSamples, bool
//                              encoderInverted, std::function<double()>&
//                              position, std::function<double()>& rate) {
//   auto* talonController = dynamic_cast<WPI_BaseMotorController*>(controller);
//   talonController->ConfigSelectedFeedbackSensor(feedbackDevice);
//   talonController->SetSensorPhase(encoderInverted);
//   talonController->ConfigVelocityMeasurementWindow(numSamples);

//   // Determine velocity measurement period
//   auto talonPeriod = getCTREVelocityPeriod(period);

//   talonController->ConfigVelocityMeasurementPeriod(talonPeriod);
//   position = [=] { return talonController->GetSelectedSensorPosition() / cpr;
//   }; rate = [=] {
//     return talonController->GetSelectedSensorVelocity() / cpr /
//            0.1;  // Conversion factor from 100 ms to seconds
//   };
// }

void SetupEncoders(
    std::string_view encoderType, bool isEncoding, int period, double cpr,
    int numSamples, std::string_view controllerName,
    frc::MotorController* controller, bool encoderInverted,
    const std::vector<int>& encoderPorts,
    // std::unique_ptr<CANCoder>& cancoder,
    std::unique_ptr<rev::SparkMaxRelativeEncoder>& revEncoderPort,
    std::unique_ptr<rev::SparkMaxAlternateEncoder>& revDataPort,
    std::unique_ptr<frc::Encoder>& encoder, std::function<double()>& position,
    std::function<double()>& rate) {
  if (encoderType == "Built-in") {
    if (wpi::starts_with(controllerName, "Talon")) {
      // FeedbackDevice feedbackDevice;
      if (controllerName == "TalonSRX") {
        fmt::print("Setup Built-in+TalonSRX");
        //   feedbackDevice = FeedbackDevice::QuadEncoder;
      } else {
        fmt::print("Setup Built-in+TalonFX");
        //   feedbackDevice = FeedbackDevice::IntegratedSensor;
      }
      // SetupCTREEncoder(controller, feedbackDevice, period, cpr, numSamples,
      //                  encoderInverted, position, rate);
    } else {  // Venom
      fmt::print("Setup Built-in+Venom");
      //   auto* venom = static_cast<frc::CANVenom*>(controller);
      //   position = [=] { return venom->GetPosition(); };
      //   rate = [=] {
      //     return venom->GetSpeed() /
      //            60;  // Conversion from RPM to rotations per second
      //   };
    }
  } else if (encoderType == "Encoder Port") {
    auto* sparkMax = static_cast<rev::CANSparkMax*>(controller);
    if (controllerName != "SPARK MAX (Brushless)") {
      fmt::print("Setup SPARK MAX (Brushed) Encoder Port");
      revEncoderPort =
          std::make_unique<rev::SparkMaxRelativeEncoder>(sparkMax->GetEncoder(
              rev::SparkMaxRelativeEncoder::Type::kQuadrature, cpr));
      revEncoderPort->SetInverted(encoderInverted);
    } else {
      fmt::print("Setup SPARK MAX (Brushless) Encoder Port");
      revEncoderPort =
          std::make_unique<rev::SparkMaxRelativeEncoder>(sparkMax->GetEncoder(
              rev::SparkMaxRelativeEncoder::Type::kHallSensor));
    }

    revEncoderPort->SetMeasurementPeriod(period);
    revEncoderPort->SetAverageDepth(numSamples);

    position = [&] { return revEncoderPort->GetPosition(); };
    rate = [&] { return revEncoderPort->GetVelocity() / 60; };
  } else if (encoderType == "Data Port") {
    fmt::print("Setup SPARK MAX Data Port");
    auto* sparkMax = static_cast<rev::CANSparkMax*>(controller);
    revDataPort = std::make_unique<rev::SparkMaxAlternateEncoder>(
        sparkMax->GetAlternateEncoder(
            rev::SparkMaxAlternateEncoder::Type::kQuadrature, cpr));
    revDataPort->SetInverted(encoderInverted);
    revDataPort->SetMeasurementPeriod(period);
    revDataPort->SetAverageDepth(numSamples);
    position = [&] { return revDataPort->GetPosition(); };
    rate = [&] { return revDataPort->GetVelocity() / 60; };
  } else if (encoderType == "Tachometer") {
    fmt::print("Setup Tachometer");
    // SetupCTREEncoder(controller, FeedbackDevice::Tachometer, period, cpr,
    //                  numSamples, encoderInverted, position, rate);
  } else if (encoderType == "CANCoder") {
    fmt::print("Setup CANCoder");
    // cancoder = std::make_unique<CANCoder>(encoderPorts[0]);
    // cancoder->ConfigSensorDirection(encoderInverted);

    // sensors::SensorVelocityMeasPeriod cancoderPeriod =
    //     getCTREVelocityPeriod(period);

    // cancoder->ConfigVelocityMeasurementPeriod(cancoderPeriod);
    // cancoder->ConfigVelocityMeasurementWindow(numSamples);

    // position = [&] { return cancoder->GetPosition() / cpr; };
    // rate = [&] { return cancoder->GetVelocity() / cpr; };
  } else {
    fmt::print("Setup roboRIO quadrature");
    if (isEncoding) {
      encoder = std::make_unique<frc::Encoder>(
          encoderPorts[0], encoderPorts[1], encoderInverted,
          frc::CounterBase::EncodingType::k1X);
    } else {
      encoder = std::make_unique<frc::Encoder>(encoderPorts[0], encoderPorts[1],
                                               encoderInverted);
    }

    encoder->SetDistancePerPulse(1 / cpr);
    encoder->SetReverseDirection(encoderInverted);
    encoder->SetSamplesToAverage(numSamples);
    position = [&] { return encoder->GetDistance(); };
    rate = [&] { return encoder->GetRate(); };
  }
}
void SetMotorControllers(
    units::volt_t motorVoltage,
    const std::vector<std::unique_ptr<frc::MotorController>>& controllers) {
  for (auto&& controller : controllers) {
    controller->SetVoltage(motorVoltage);
  }
}

}  // namespace sysid
