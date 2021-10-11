// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <string_view>

#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTableInstance.h>

#include "sysid/Util.h"

/**
 * Launches a robot simulation using std::system
 *
 * @param projectDirectory the relative path of the robot project folder from
 * the root directory.
 */
void LaunchSim(std::string_view projectDirectory);

/**
 * Connects to a Simulation NT Entry and configures the kill variable to not
 * prematurely kill the simulated code. This assumes that the simulated code
 * uses a NT kill entry to end the simulation.
 *
 * @param nt the Network Tables Intstance
 * @param kill the Network Tables Entry responsible for killing the simulated
 * code.
 */
void Connect(nt::NetworkTableInstance nt, nt::BooleanPublisher& kill);

/**
 * Connects to a Simulation NT Entry and configures the kill variable to kill
 * the robot code. This assumes that the simulated code uses a NT kill entry to
 * end the simulation.
 *
 * @param nt the Network Tables Instance
 * @param kill the Network Tables Entry responsible for killing the simulated
 *             code.
 *
 * @return captured console output during the robot simulation.
 */
std::string KillNT(nt::NetworkTableInstance nt, nt::BooleanPublisher& kill);
