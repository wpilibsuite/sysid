// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "generation/SysIdSetup.h"

#include <stdexcept>

#include <CANVenom.h>
#include <frc/Filesystem.h>
#include <frc/Spark.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <wpi/FileSystem.h>
#include <wpi/SmallString.h>

// Based on https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html
#define EXPAND_STRINGIZE(s) STRINGIZE(s)
#define STRINGIZE(s) #s

// C++20 shim for std::string_view::starts_with()
static constexpr bool starts_with(std::string_view obj,
                                  std::string_view x) noexcept {
  return obj.substr(0, x.size()) == x;
}

wpi::json GetConfigJson() {
  wpi::SmallString<128> path;
  wpi::raw_svector_ostream os{path};

  if constexpr (frc::RobotBase::IsSimulation()) {
#if defined(PROJECT_ROOT_DIR) && defined(INTEGRATION)
    // TODO: Fix problems with this so that we don't need this ifdef
    os << EXPAND_STRINGIZE(PROJECT_ROOT_DIR)
       << "/sysid-projects/deploy/config.json";
#endif
  } else {
    frc::filesystem::GetDeployDirectory(path);
    os << "/config.json";
  }

  std::error_code ec;
  wpi::raw_fd_istream is{path.c_str(), ec};
  if (ec) {
    wpi::outs() << "File error: " << path.c_str() << "\n";
    wpi::outs().flush();
    throw std::runtime_error("Unable to read file");
  }

  wpi::json outJson;
  is >> outJson;
  return outJson;
}

void AddMotorController(
    int port, std::string_view controller, bool inverted,
    std::vector<std::unique_ptr<frc::SpeedController>>* controllers) {
  if (controller == "TalonSRX" || controller == "VictorSPX" ||
      controller == "TalonFX") {
    if (controller == "TalonSRX") {
      controllers->push_back(std::make_unique<WPI_TalonSRX>(port));
    } else if (controller == "TalonFX") {
      controllers->emplace_back(std::make_unique<WPI_TalonFX>(port));
    } else {
      controllers->emplace_back(std::make_unique<WPI_VictorSPX>(port));
    }
    dynamic_cast<WPI_BaseMotorController*>(controllers->back().get())
        ->SetInverted(inverted);
    dynamic_cast<WPI_BaseMotorController*>(controllers->back().get())
        ->SetNeutralMode(motorcontrol::NeutralMode::Brake);
  } else if (controller == "SPARK MAX (Brushless)" ||
             controller == "SPARK MAX (Brushed)") {
    if (controller == "SPARK MAX (Brushless)") {
      controllers->emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushless));
    } else {
      controllers->emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushed));
    }
    static_cast<rev::CANSparkMax*>(controllers->back().get())
        ->SetInverted(inverted);
    static_cast<rev::CANSparkMax*>(controllers->back().get())
        ->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  } else if (controller == "Venom") {
    controllers->emplace_back(std::make_unique<frc::CANVenom>(port));
    static_cast<frc::CANVenom*>(controllers->back().get())
        ->SetInverted(inverted);
    static_cast<frc::CANVenom*>(controllers->back().get())
        ->SetBrakeCoastMode(frc::CANVenom::BrakeCoastMode::kBrake);
  } else {
    controllers->emplace_back(std::make_unique<frc::Spark>(port));
    static_cast<frc::Spark*>(controllers->back().get())->SetInverted(inverted);
  }
}

void SetupEncoders(std::string_view encoderType, bool isEncoding, int period,
                   double cpr, int numSamples, std::string_view controllerName,
                   frc::SpeedController* controller, bool encoderInverted,
                   const std::vector<int>& encoderPorts,
                   std::unique_ptr<CANCoder>& cancoder,
                   std::unique_ptr<frc::Encoder>& encoder,
                   std::function<double()>& position,
                   std::function<double()>& rate) {
  if (encoderType == "Built-In") {
    wpi::outs() << "Initializing talon \n";
    wpi::outs().flush();
    if (starts_with(controllerName, "Talon")) {
      if (controllerName == "TalonSRX") {
        dynamic_cast<WPI_BaseMotorController*>(controller)
            ->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
      } else {
        dynamic_cast<WPI_BaseMotorController*>(controller)
            ->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
      }
      dynamic_cast<WPI_BaseMotorController*>(controller)
          ->SetSensorPhase(encoderInverted);
      dynamic_cast<WPI_BaseMotorController*>(controller)
          ->ConfigVelocityMeasurementWindow(numSamples);

      motorcontrol::VelocityMeasPeriod talonPeriod;
      if (period == 1) {
        talonPeriod = motorcontrol::VelocityMeasPeriod::Period_1Ms;
      } else if (period == 2) {
        talonPeriod = motorcontrol::VelocityMeasPeriod::Period_2Ms;
      } else if (period == 5) {
        talonPeriod = motorcontrol::VelocityMeasPeriod::Period_5Ms;
      } else if (period == 10) {
        talonPeriod = motorcontrol::VelocityMeasPeriod::Period_10Ms;
      } else if (period == 25) {
        talonPeriod = motorcontrol::VelocityMeasPeriod::Period_25Ms;
      } else if (period == 50) {
        talonPeriod = motorcontrol::VelocityMeasPeriod::Period_50Ms;
      } else {
        talonPeriod = motorcontrol::VelocityMeasPeriod::Period_100Ms;
      }

      dynamic_cast<WPI_BaseMotorController*>(controller)
          ->ConfigVelocityMeasurementPeriod(talonPeriod);
      wpi::outs() << "controller:" << controller << "\n";
      wpi::outs().flush();
      position = [=] {
        return dynamic_cast<WPI_BaseMotorController*>(controller)
                   ->GetSelectedSensorPosition() /
               cpr;
      };
      rate = [=] {
        return dynamic_cast<WPI_BaseMotorController*>(controller)
                   ->GetSelectedSensorVelocity() /
               cpr / 0.1;  // Conversion factor from 100 ms to seconds
      };
    } else if (controllerName == "Venom") {
      position = [=] {
        return dynamic_cast<frc::CANVenom*>(controller)->GetPosition();
      };
      rate = [=] {
        return dynamic_cast<frc::CANVenom*>(controller)->GetSpeed() /
               60;  // Conversion from RPM to rotations per second
      };
    } else {
      if (controllerName != "SPARK MAX (Brushless)") {
        static_cast<rev::CANSparkMax*>(controller)
            ->GetEncoder(rev::CANEncoder::EncoderType::kQuadrature, cpr);
        static_cast<rev::CANSparkMax*>(controller)
            ->GetEncoder()
            .SetInverted(encoderInverted);
      }

      static_cast<rev::CANSparkMax*>(controller)
          ->GetEncoder()
          .SetMeasurementPeriod(period);
      static_cast<rev::CANSparkMax*>(controller)
          ->GetEncoder()
          .SetAverageDepth(numSamples);

      position = [=] {
        return static_cast<rev::CANSparkMax*>(controller)
            ->GetEncoder()
            .GetPosition();
      };
      rate = [=] {
        return static_cast<rev::CANSparkMax*>(controller)
                   ->GetEncoder()
                   .GetVelocity() /
               60;
      };
    }
  } else if (encoderType == "CANCoder / Alternate") {
    if (starts_with(controllerName, "SPARK MAX")) {
      static_cast<rev::CANSparkMax*>(controller)
          ->GetAlternateEncoder(
              rev::CANEncoder::AlternateEncoderType::kQuadrature, cpr);
      static_cast<rev::CANSparkMax*>(controller)
          ->GetAlternateEncoder(
              rev::CANEncoder::AlternateEncoderType::kQuadrature, cpr)
          .SetInverted(encoderInverted);
      static_cast<rev::CANSparkMax*>(controller)
          ->GetAlternateEncoder(
              rev::CANEncoder::AlternateEncoderType::kQuadrature, cpr)
          .SetMeasurementPeriod(period);
      static_cast<rev::CANSparkMax*>(controller)
          ->GetAlternateEncoder(
              rev::CANEncoder::AlternateEncoderType::kQuadrature, cpr)
          .SetAverageDepth(numSamples);
      position = [=] {
        return static_cast<rev::CANSparkMax*>(controller)
            ->GetAlternateEncoder(
                rev::CANEncoder::AlternateEncoderType::kQuadrature, cpr)
            .GetPosition();
      };
      rate = [=] {
        return static_cast<rev::CANSparkMax*>(controller)
                   ->GetAlternateEncoder(
                       rev::CANEncoder::AlternateEncoderType::kQuadrature, cpr)
                   .GetVelocity() /
               60;
      };
    } else {
      cancoder = std::make_unique<CANCoder>(encoderPorts[0]);
      cancoder->ConfigSensorDirection(encoderInverted);

      sensors::SensorVelocityMeasPeriod cancoderPeriod;
      if (period == 1) {
        cancoderPeriod = sensors::SensorVelocityMeasPeriod::Period_1Ms;
      } else if (period == 2) {
        cancoderPeriod = sensors::SensorVelocityMeasPeriod::Period_2Ms;
      } else if (period == 5) {
        cancoderPeriod = sensors::SensorVelocityMeasPeriod::Period_5Ms;
      } else if (period == 10) {
        cancoderPeriod = sensors::SensorVelocityMeasPeriod::Period_10Ms;
      } else if (period == 25) {
        cancoderPeriod = sensors::SensorVelocityMeasPeriod::Period_25Ms;
      } else if (period == 50) {
        cancoderPeriod = sensors::SensorVelocityMeasPeriod::Period_50Ms;
      } else {
        cancoderPeriod = sensors::SensorVelocityMeasPeriod::Period_100Ms;
      }

      cancoder->ConfigVelocityMeasurementPeriod(cancoderPeriod);
      cancoder->ConfigVelocityMeasurementWindow(numSamples);

      position = [&] { return cancoder->GetPosition() / cpr; };
      rate = [&] { return cancoder->GetVelocity() / cpr; };
    }
  } else {
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
    const std::vector<std::unique_ptr<frc::SpeedController>>& controllers) {
  for (auto&& controller : controllers) {
    controller->SetVoltage(motorVoltage);
  }
}
