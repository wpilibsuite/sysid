// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <wpi/Logger.h>
#include <wpi/json.h>
#include <wpinet/uv/Loop.h>

namespace sysid {
// Define an integer for a successful message in the log (shown in green on the
// GUI).
static constexpr unsigned int kLogSuccess = 31;

/**
 * Represents a single deploy session.
 *
 * An instance of this class must be kept alive in memory until GetStatus()
 * returns kDiscoveryFailure or kDone. Otherwise, the deploy will fail!
 */
class DeploySession {
 public:
  /**
   * Represents the status of the deploy session.
   */
  enum class Status { kInProgress, kDiscoveryFailure, kDone };

  /**
   * Constructs an instance of the deploy session.
   *
   * @param team   The team number (or an IP address/hostname).
   * @param drive  Whether the drive program should be deployed to the roboRIO.
   *               If this is set to false, the mechanism project will be
   *               deployed.
   * @param config The generation configuration file to be sent to the roboRIO.
   * @param logger A reference to a logger where log messages should be sent.
   */
  DeploySession(std::string_view team, bool drive, wpi::json config,
                wpi::Logger& logger);

  /**
   * Executes the deploy. This can be called from any thread.
   *
   * @param lp A reference to a libuv event loop to run the deploy on.
   */
  void Execute(wpi::uv::Loop& lp);

  /**
   * Returns the state of the deploy session.
   *
   * @return the deploy session state as a Status
   */
  Status GetStatus() const;

  /**
   * Returns a list of hostnames and/or addresses to try while discovering the
   * roboRIO.
   *
   * @param team The team number.
   * @return A vector of hostnames/addresses to try during discovery.
   */
  static std::vector<std::string> GetAddressesToTry(int team);

  /**
   * Returns the stored JSON object
   *
   * @return The stored JSON object
   */
  const wpi::json& GetJSON() const { return m_config; }

 private:
  // General deploy parameters from the constructor.
  bool m_drive;
  wpi::json m_config;

  // Logger reference where log messages will be sent.
  wpi::Logger& m_logger;

  // List of addresses that we can try during the discovery process.
  std::vector<std::string> m_addresses;

  // Whether we have an active SSH connection to the roboRIO and deploying
  // artifacts.
  std::atomic_bool m_connected = false;

  // The number of hostnames that have completed their resolution/connection
  // attempts.
  std::atomic_uint m_visited = 0;
};
}  // namespace sysid
