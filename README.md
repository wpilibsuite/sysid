# SysId: System Identification for Robot Mechanisms
[![Build](https://github.com/wpilibsuite/sysid/actions/workflows/build.yml/badge.svg)](https://github.com/wpilibsuite/sysid/actions/workflows/build.yml)

This is the C++ version of [frc-characterization](https://github.com/wpilibsuite/frc-characterization). It uses the [wpimath](https://github.com/wpilibsuite/allwpilib/tree/main/wpimath) backend for generating feedforward and feedback gains.

## Supported Vendor Hardware

This is a list of the vendors that are currently supported by SysId for 2023:

### Vendors

- CTRE: TalonSRX, TalonFX, VictorSPX, CANCoder, and Pigeon IMU
- NavX Gyro (No integration tests)
- Playing With Fusion Venom
- REV: SparkMax Motor Controller

### Hardware Supported by WPILib

- ADIS16448 IMU
- ADIS16470 IMU
- ADXRS450 Gyro
- Analog Gyro
- Encoders plugged into the roboRIO
- PWM Motor Controllers

## Downloading and Running Development Versions of SysId (Analysis Only)

The SysId GitHub repository uses GitHub Actions to build and test each commit.

### To download development version of SysId from the main branch (most stable):

1. Click [here](https://github.com/wpilibsuite/sysid/actions/workflows/build.yml?query=branch%3Amain).
2. Select the run of your choosing (the top most run is the latest).
3. Scroll down to the artifacts section, and select the appropriate build for your operating system.

### To download development version of SysId from all branches:

1. Click [here](https://github.com/wpilibsuite/sysid/actions/workflows/build.yml).
2. Select the run of your choosing (the top most run is the latest).
3. Scroll down to artifacts section, and select the appropriate build for your operating system.

### To run development versions of SysId

1. Extract the files from the downloaded zip file.
2. Extract the files from the zip file located in the root of the extracted folder.
3. Open the folder named after your operating system (ex: Linux, Windows, macOS).
4. Open the folder named for your architecture.
5. Run the executable named `sysid`.

## Downloading and Running Full Development Versions of SysId (Generation, Logging, Analysis)

Clone the SysId Github repository to get all the necessary code.

Go to the directory that the repository is cloned in and run `./gradlew run` to start the executable.

### Generating And Running SysId Robot Code

1. Add sysid vendordep to your robot code and call the logging function.
2. Deploy your robot code.
3. Connect the logger to your robot and perform the required tests.

## Building and Running SysId

SysId uses Gradle to build. To build debug and release versions of the main executable and run tests, run `./gradlew build`. During development, you can use `./gradlew run` to build and run the debug executable.

### Requirements

- [JDK 11](https://adoptium.net/temurin/releases/?version=11)
    - Note that the JRE is insufficient; the full JDK is required
    - On Ubuntu, run `sudo apt install openjdk-11-jdk`
    - On Windows, install the JDK 11 .msi from the link above
    - On macOS, install the JDK 11 .pkg from the link above
- C++ compiler
    - On Linux, install GCC 11 or greater
    - On Windows, install [Visual Studio Community 2022](https://visualstudio.microsoft.com/vs/community/) and select the C++ programming language during installation (Gradle can't use the build tools for Visual Studio)
    - On macOS, install the Xcode command-line build tools via `xcode-select --install`. Xcode 13 or later is required.
- roboRIO C++ compiler
    - Run the latest WPILib installer from [here](https://github.com/wpilibsuite/allwpilib/releases/latest)

## Logging Projects

### Time Plotting Script

There is a python script in the `scripts` directory that will plot sysid data with respect to time. This is mainly intended for developers in the case that a dataset breaks sysid.

In order to run it, pandas and matplotlib must be installed via pip.

Then run `python3 scripts/time_plots.py "file path"` to get the time plots for your desired datasets.

Matplotlib windows will appear containing the different plots.
