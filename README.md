# SysId: System Identification for Robot Mechanisms
[![Build](https://github.com/wpilibsuite/sysid/actions/workflows/build.yml/badge.svg)](https://github.com/wpilibsuite/sysid/actions/workflows/build.yml)

This is the C++ version of [frc-characterization](https://github.com/wpilibsuite/frc-characterization). It uses the [wpimath](https://github.com/wpilibsuite/allwpilib/tree/main/wpimath) backend for generating feedforward and feedback gains.

## Downloading and Running Development Versions of SysId

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

## Building and Running SysId

SysId uses Gradle to build. To build debug and release versions of the main executable and run tests, run `./gradlew build`. During development, you can use `./gradlew run` to build and run the debug executable.

SysId also has integration tests, which involves launching a robot program with simulation physics, characterizing it and verifying the gains. These tests are not enabled by default; instead, you need to pass the `-PwithIntegration` flag into Gradle. To run just the integration tests, you can use `./gradlew runIntegrationTests -PwithIntegration`.

There is also a robot project in `integration_test_project` that you can use to test out SysId. To launch the robot program, simply `cd` into it and run `./gradlew simulateCpp`.

### Requirements

- [JDK 11](https://adoptopenjdk.net/)
    - Note that the JRE is insufficient; the full JDK is required
    - On Ubuntu, run `sudo apt install openjdk-11-jdk`
    - On Windows, install the JDK 11 .msi from the link above
    - On macOS, install the JDK 11 .pkg from the link above
- C++ compiler
    - On Linux, install GCC 7 or greater
    - On Windows, install [Visual Studio Community 2019](https://visualstudio.microsoft.com/vs/community/) and select the C++ programming language during installation (Gradle can't use the build tools for Visual Studio 2019)
    - On macOS, install the Xcode command-line build tools via `xcode-select --install`

## Logging Projects
SysId comes with projects that interface with the telemetry manager to provide the necessary data for analysis. These projects are stored in the `base_projects` folder and take in a `config.json` file in the `src/main/deploy` directory to setup the robot hardware for analysis.

There is a `Drivetrain` project for drivetrain analysis and a `GeneralMechanism` project for simple-motor, elevator, and arm analyses.

### Running the Projects
The executable generated from building is currently hardcoded to save to the proper project and these projects can be run normally from VSCode or the command line.
