// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/generation/SysIdSetup.h"

#include <stdexcept>

// #include <CANVenom.h>
#include <fmt/core.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/Filesystem.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/Spark.h>
// #include <frc/romi/RomiGyro.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <wpi/SmallString.h>
#include <wpi/StringExtras.h>
#include <wpi/fs.h>

#ifdef __FRC_ROBORIO__
#include "AHRS.h"
#endif

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
      fmt::print("Setup TalonSRX\n");
      controllers->push_back(std::make_unique<WPI_TalonSRX>(port));
    } else if (controller == "TalonFX") {
      fmt::print("Setup TalonFX\n");
      controllers->emplace_back(std::make_unique<WPI_TalonFX>(port));
    } else {
      fmt::print("Setup VictorSPX\n");
      controllers->emplace_back(std::make_unique<WPI_VictorSPX>(port));
    }

    auto* ctreController =
        dynamic_cast<WPI_BaseMotorController*>(controllers->back().get());
    ctreController->ConfigFactoryDefault();
    ctreController->SetInverted(inverted);
    ctreController->SetNeutralMode(motorcontrol::NeutralMode::Brake);
  } else if (controller == "SPARK MAX (Brushless)" ||
             controller == "SPARK MAX (Brushed)") {
    if (controller == "SPARK MAX (Brushless)") {
      fmt::print("Setup SPARK MAX (Brushless)\n");
      controllers->emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushless));
      controllers->emplace_back(std::make_unique<frc::Spark>(port));
    } else {
      fmt::print("Setup SPARK MAX (Brushed)\n");
      controllers->emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushed));
      controllers->emplace_back(std::make_unique<frc::Spark>(port));
    }

    auto* sparkMax = static_cast<rev::CANSparkMax*>(controllers->back().get());
    sparkMax->RestoreFactoryDefaults();
    sparkMax->SetInverted(inverted);
    sparkMax->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  } else if (controller == "Venom") {
    fmt::print("Setup Venom\n");
    // controllers->emplace_back(std::make_unique<frc::CANVenom>(port));

    // auto* venom = static_cast<frc::CANVenom*>(controllers->back().get());

    // venom->SetInverted(inverted);
    // venom->SetBrakeCoastMode(frc::CANVenom::BrakeCoastMode::kBrake);
    controllers->emplace_back(std::make_unique<frc::Spark>(port));
  } else {
    fmt::print("Setup PWM\n");
    controllers->emplace_back(std::make_unique<frc::Spark>(port));
    auto* spark = static_cast<frc::Spark*>(controllers->back().get());
    spark->SetInverted(inverted);
    controllers->emplace_back(std::make_unique<frc::Spark>(port));
  }
}

static sensors::SensorVelocityMeasPeriod getCTREVelocityPeriod(int period) {
  switch (period) {
    case 1:
      return sensors::SensorVelocityMeasPeriod::Period_1Ms;
    case 2:
      return sensors::SensorVelocityMeasPeriod::Period_2Ms;
    case 5:
      return sensors::SensorVelocityMeasPeriod::Period_5Ms;
    case 10:
      return sensors::SensorVelocityMeasPeriod::Period_10Ms;
    case 25:
      return sensors::SensorVelocityMeasPeriod::Period_25Ms;
    case 50:
      return sensors::SensorVelocityMeasPeriod::Period_50Ms;
    default:
      return sensors::SensorVelocityMeasPeriod::Period_100Ms;
  }
}

static void SetupCTREEncoder(frc::MotorController* controller,
                             FeedbackDevice feedbackDevice, int period,
                             double cpr, int numSamples, bool encoderInverted,
                             std::function<double()>& position,
                             std::function<double()>& rate) {
  auto* talonController = dynamic_cast<WPI_BaseMotorController*>(controller);
  talonController->ConfigSelectedFeedbackSensor(feedbackDevice);
  talonController->SetSensorPhase(encoderInverted);
  talonController->ConfigVelocityMeasurementWindow(numSamples);

  // Determine velocity measurement period
  auto talonPeriod = getCTREVelocityPeriod(period);

  talonController->ConfigVelocityMeasurementPeriod(talonPeriod);
  position = [=] { return talonController->GetSelectedSensorPosition() / cpr; };
  rate = [=] {
    return talonController->GetSelectedSensorVelocity() / cpr /
           0.1;  // Conversion factor from 100 ms to seconds
  };
}

void SetDefaultDataCollection(std::function<double()>& position,
                              std::function<double()>& rate) {
  fmt::print("Setting default\n");
  position = [&] { return 0.0; };
  rate = [&] { return 0.0; };
}

void SetupEncoders(
    std::string_view encoderType, bool isEncoding, int period, double cpr,
    double gearing, int numSamples, std::string_view controllerName,
    frc::MotorController* controller, bool encoderInverted,
    const std::vector<int>& encoderPorts, std::unique_ptr<CANCoder>& cancoder,
    std::unique_ptr<rev::SparkMaxRelativeEncoder>& revEncoderPort,
    std::unique_ptr<rev::SparkMaxAlternateEncoder>& revDataPort,
    std::unique_ptr<frc::Encoder>& encoder, std::function<double()>& position,
    std::function<double()>& rate) {
  double combinedCPR = cpr * gearing;
  frc::SmartDashboard::PutNumber("SysIdCPR", cpr);
  frc::SmartDashboard::PutNumber("SysIdGearing", gearing);
  frc::SmartDashboard::PutNumber("SysIdConversionFactor", combinedCPR);

#ifndef __FRC_ROBORIO__
  fmt::print("Setting default rates\n");
  SetDefaultDataCollection(position, rate);
#endif
  if (encoderType == "Built-in") {
    if (wpi::starts_with(controllerName, "Talon")) {
      FeedbackDevice feedbackDevice;
      if (controllerName == "TalonSRX") {
        fmt::print("Setup Built-in+TalonSRX\n");
        feedbackDevice = FeedbackDevice::QuadEncoder;
      } else {
        fmt::print("Setup Built-in+TalonFX\n");
        feedbackDevice = FeedbackDevice::IntegratedSensor;
      }
      SetupCTREEncoder(controller, feedbackDevice, period, combinedCPR,
                       numSamples, encoderInverted, position, rate);
    } else {  // Venom
      fmt::print("Setup Built-in+Venom\n");
      // auto* venom = static_cast<frc::CANVenom*>(controller);
      // position = [=] { return venom->GetPosition() / gearing; };
      // rate = [=] {
      //     return venom->GetSpeed() / gearing /
      //            60;  // Conversion from RPM to rotations per second
      // };
    }
  } else if (encoderType == "Encoder Port") {
    auto* sparkMax = static_cast<rev::CANSparkMax*>(controller);
    if (controllerName != "SPARK MAX (Brushless)") {
      fmt::print("Setup SPARK MAX (Brushed) Encoder Port\n");
      revEncoderPort =
          std::make_unique<rev::SparkMaxRelativeEncoder>(sparkMax->GetEncoder(
              rev::SparkMaxRelativeEncoder::Type::kQuadrature, cpr));
      revEncoderPort->SetInverted(encoderInverted);
    } else {
      fmt::print("Setup SPARK MAX (Brushless) Encoder Port\n");
      revEncoderPort =
          std::make_unique<rev::SparkMaxRelativeEncoder>(sparkMax->GetEncoder(
              rev::SparkMaxRelativeEncoder::Type::kHallSensor));
    }

    revEncoderPort->SetMeasurementPeriod(period);
    revEncoderPort->SetAverageDepth(numSamples);

    position = [=, &revEncoderPort] {
      return revEncoderPort->GetPosition() / gearing;
    };
    rate = [=, &revEncoderPort] {
      return revEncoderPort->GetVelocity() / gearing / 60;
    };
  } else if (encoderType == "Data Port") {
    fmt::print("Setup SPARK MAX Data Port\n");
    auto* sparkMax = static_cast<rev::CANSparkMax*>(controller);
    revDataPort = std::make_unique<rev::SparkMaxAlternateEncoder>(
        sparkMax->GetAlternateEncoder(
            rev::SparkMaxAlternateEncoder::Type::kQuadrature, cpr));
    revDataPort->SetInverted(encoderInverted);
    revDataPort->SetMeasurementPeriod(period);
    revDataPort->SetAverageDepth(numSamples);
    position = [=, &revDataPort] {
      return revDataPort->GetPosition() / gearing;
    };
    rate = [=, &revDataPort] {
      return revDataPort->GetVelocity() / gearing / 60;
    };
  } else if (encoderType == "Tachometer") {
    fmt::print("Setup Tachometer\n");
    SetupCTREEncoder(controller, FeedbackDevice::Tachometer, period,
                     combinedCPR, numSamples, encoderInverted, position, rate);
  } else if (encoderType == "CANCoder") {
    fmt::print("Setup CANCoder\n");
    cancoder = std::make_unique<CANCoder>(encoderPorts[0]);
    cancoder->ConfigSensorDirection(encoderInverted);

    sensors::SensorVelocityMeasPeriod cancoderPeriod =
        getCTREVelocityPeriod(period);

    cancoder->ConfigVelocityMeasurementPeriod(cancoderPeriod);
    cancoder->ConfigVelocityMeasurementWindow(numSamples);

    position = [=, &cancoder] { return cancoder->GetPosition() / combinedCPR; };
    rate = [=, &cancoder] { return cancoder->GetVelocity() / combinedCPR; };
  } else {
    fmt::print("Setup roboRIO quadrature\n");
    if (isEncoding) {
      encoder = std::make_unique<frc::Encoder>(
          encoderPorts[0], encoderPorts[1], encoderInverted,
          frc::CounterBase::EncodingType::k1X);
    } else {
      encoder = std::make_unique<frc::Encoder>(encoderPorts[0], encoderPorts[1],
                                               encoderInverted);
    }

    encoder->SetDistancePerPulse(1 / combinedCPR);
    encoder->SetReverseDirection(encoderInverted);
    encoder->SetSamplesToAverage(numSamples);
    position = [&] { return encoder->GetDistance(); };
    rate = [&] { return encoder->GetRate(); };
  }
}

static frc::SPI::Port GetSPIPort(std::string_view name) {
  if (name == "SPI (Onboard CS0)") {
    fmt::print("Setup SPI (Onboard CS0)\n");
    return frc::SPI::Port::kOnboardCS0;
  } else if (name == "SPI (Onboard CS1)") {
    fmt::print("Setup SPI (Onboard CS1)\n");
    return frc::SPI::Port::kOnboardCS1;
  } else if (name == "SPI (Onboard CS2)") {
    fmt::print("Setup SPI (Onboard CS2)\n");
    return frc::SPI::Port::kOnboardCS2;
  } else if (name == "SPI (Onboard CS3)") {
    fmt::print("Setup SPI (Onboard CS3)\n");
    return frc::SPI::Port::kOnboardCS3;
  } else if (name == "SPI (MXP)") {
    fmt::print("Setup SPI (MXP)\n");
    return frc::SPI::Port::kMXP;
  } else {
    throw std::runtime_error(
        fmt::format("{} is not a supported SPI Port\n", name));
  }
}

void SetupGyro(
    std::string_view gyroType, std::string_view gyroCtor,
    const std::vector<int>& leftPorts, const std::vector<int>& rightPorts,
    const std::vector<std::string>& controllerNames,
    const std::vector<std::unique_ptr<frc::MotorController>>& leftControllers,
    const std::vector<std::unique_ptr<frc::MotorController>>& rightControllers,
    std::unique_ptr<frc::Gyro>& gyro,
    std::unique_ptr<frc::ADIS16448_IMU>& ADIS16448Gyro,
    std::unique_ptr<frc::ADIS16470_IMU>& ADIS16470Gyro,
    std::unique_ptr<BasePigeon>& pigeon,
    std::unique_ptr<WPI_TalonSRX>& tempTalon,
    std::function<double()>& gyroPosition, std::function<double()>& gyroRate) {
#ifndef __FRC_ROBORIO__
  sysid::SetDefaultDataCollection(gyroPosition, gyroRate);
#endif
  if (wpi::starts_with(gyroType, "Pigeon")) {
    std::string portStr;
    if (wpi::contains(gyroCtor, "WPI_TalonSRX")) {
      portStr = wpi::split(gyroCtor, "-").second;
    } else {
      portStr = gyroCtor;
    }

    // converts gyroCtor into port #
    int srxPort = std::stoi(portStr);
    if (wpi::contains(gyroCtor, "WPI_TalonSRX")) {
      std::vector<int> ports = leftPorts;
      ports.insert(ports.end(), rightPorts.begin(), rightPorts.end());
      // Check if there is a Talon Port in Left Ports
      auto findPort = std::find(ports.begin(), ports.end(), srxPort);

      WPI_TalonSRX* talon;
      if (findPort != ports.end()) {
        // If the object already exists, find it and store it
        if (std::distance(ports.begin(), findPort) <
            std::distance(leftPorts.begin(), leftPorts.end())) {
          talon = dynamic_cast<WPI_TalonSRX*>(
              leftControllers.at(findPort - ports.begin()).get());
        } else {
          talon = dynamic_cast<WPI_TalonSRX*>(
              rightControllers.at(findPort - ports.begin() - leftPorts.size())
                  .get());
        }
        portStr = fmt::format("{} (plugged to drive motorcontroller)", portStr);
      } else {
        // If it isn't tied to an existing Talon, create a new object
        tempTalon = std::make_unique<WPI_TalonSRX>(srxPort);
        talon = tempTalon.get();
        portStr = fmt::format("{} (plugged to other motorcontroller)", portStr);
      }
      pigeon = std::make_unique<PigeonIMU>(talon);
      fmt::print("Setup Pigeon, {}\n", portStr);
    } else {
      portStr = fmt::format("{} (CAN)", portStr);

      if (gyroType == "Pigeon") {
        pigeon = std::make_unique<PigeonIMU>(srxPort);
        fmt::print("Setup Pigeon, {}\n", portStr);
      } else {
        pigeon = std::make_unique<Pigeon2>(srxPort);
        fmt::print("Setup Pigeon2, {}\n", portStr);
      }
    }

    // setup functions
    gyroPosition = [&] {
      double xyz[3];
      pigeon->GetAccumGyro(xyz);
      units::degree_t pos{xyz[2]};
      return units::radian_t{pos}.value();
    };

    gyroRate = [&] {
      double xyz_dps[3];
      pigeon->GetRawGyro(xyz_dps);
      units::degrees_per_second_t rate{xyz_dps[2]};
      return units::radians_per_second_t{rate}.value();
    };
    SetDefaultDataCollection(gyroPosition, gyroRate);
  } else if (wpi::starts_with(gyroType, "ADIS")) {
    auto port = GetSPIPort(gyroCtor);
    if (gyroType == "ADIS16448") {
      fmt::print("Setup ADIS16448\n");
      // Calibration time of 8 seconds:
      // https://github.com/juchong/ADIS16448-RoboRIO-Driver-Examples/blob/master/c%2B%2B/src/main/cpp/Robot.cpp#L48
      ADIS16448Gyro = std::make_unique<frc::ADIS16448_IMU>(
          frc::ADIS16448_IMU::IMUAxis::kZ, port,
          frc::ADIS16448_IMU::CalibrationTime::_8s);
      gyroPosition = [&] {
        return units::radian_t{ADIS16448Gyro->GetAngle()}.value();
      };

      gyroRate = [&] {
        return units::radians_per_second_t{ADIS16448Gyro->GetRate()}.value();
      };
    } else if (gyroType == "ADIS16470") {
      fmt::print("Setup ADIS16470\n");
      // Calibration time of 8 seconds:
      // https://github.com/juchong/ADIS16470-RoboRIO-Driver-Examples/blob/master/c%2B%2B/src/main/cpp/Robot.cpp#L52
      ADIS16470Gyro = std::make_unique<frc::ADIS16470_IMU>(
          frc::ADIS16470_IMU::IMUAxis::kZ, port,
          frc::ADIS16470_IMU::CalibrationTime::_8s);
      gyroPosition = [&] {
        return units::radian_t{ADIS16470Gyro->GetAngle()}.value();
      };

      gyroRate = [&] {
        return units::radians_per_second_t{ADIS16470Gyro->GetRate()}.value();
      };
    }
  } else if (gyroType != "None") {
    if (gyroType == "ADXRS450") {
      auto port = GetSPIPort(gyroCtor);
      fmt::print("Setup ADXRS450\n");
      gyro = std::make_unique<frc::ADXRS450_Gyro>(port);

    } else if (gyroType == "NavX") {
      if (gyroCtor == "SerialPort (USB)") {
        fmt::print("Setup NavX, SerialPort (USB)\n");
#ifdef __FRC_ROBORIO__
        gyro = std::make_unique<AHRS>(frc::SerialPort::Port::kUSB);
#endif
      } else if (gyroCtor == "I2C (MXP)") {
        fmt::print("Setup NavX, I2C (MXP)\n");
#ifdef __FRC_ROBORIO__
        gyro = std::make_unique<AHRS>(frc::I2C::Port::kMXP);
#endif
      } else if (gyroCtor == "SerialPort (MXP)") {
        fmt::print("Setup NavX, SerialPort (MXP)\n");
#ifdef __FRC_ROBORIO__
        gyro = std::make_unique<AHRS>(frc::SerialPort::Port::kMXP);
#endif
      } else {
        fmt::print("Setup NavX, SPI (MXP)\n");
#ifdef __FRC_ROBORIO__
        gyro = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
#endif
      }
      //     // FIXME: Update Romi Gyro once vendordep is out
    } else if (gyroType == "Romi") {
      fmt::print("Setup Romi\n");
      // #ifndef __FRC_ROBORIO__
      //             gyro = std::make_unique<frc::RomiGyro>();
      // #endif
    } else if (gyroType == "Analog Gyro") {
      try {
        fmt::print("Setup Analog Gyro, {}\n", gyroCtor);
        gyro =
            std::make_unique<frc::AnalogGyro>(std::stoi(std::string{gyroCtor}));
      } catch (std::invalid_argument& e) {
        fmt::print("Setup Analog Gyro, 0\n");
        gyro = std::make_unique<frc::AnalogGyro>(0);
      }
    }
#ifdef __FRC_ROBORIO__
    gyroPosition = [&] {
      return units::radian_t{units::degree_t{gyro->GetAngle()}}.value();
    };

    gyroRate = [&] {
      return units::radians_per_second_t{
          units::degrees_per_second_t{gyro->GetRate()}}
          .value();
    };
#endif
    // Default behaviour is to make the gyro functions return zero
  } else {
    fmt::print("Setup None\n");
    SetDefaultDataCollection(gyroPosition, gyroRate);
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
