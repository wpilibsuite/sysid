// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/deploy/DeploySession.h"

#include <memory>
#include <mutex>
#include <string_view>

#include <fmt/core.h>
#include <wpi/SmallString.h>
#include <wpi/StringExtras.h>
#include <wpi/uv/Error.h>
#include <wpi/uv/GetAddrInfo.h>
#include <wpi/uv/Work.h>
#include <wpi/uv/util.h>

#include "sysid/deploy/SshSession.h"

using namespace sysid;

// Macros to make logging easier.
#define INFO(fmt, ...) WPI_INFO(m_logger, fmt, __VA_ARGS__)
#define DEBUG(fmt, ...) WPI_DEBUG(m_logger, fmt, __VA_ARGS__)
#define ERROR(fmt, ...) WPI_DEBUG(m_logger, fmt, __VA_ARGS__)
#define SUCCESS(fmt, ...) WPI_LOG(m_logger, kLogSuccess, fmt, __VA_ARGS__)

// roboRIO SSH constants.
static constexpr int kPort = 22;
static constexpr std::string_view kUsername = "admin";
static constexpr std::string_view kPassword = "";

// These are auto-generated by the build system.
namespace sysid {
std::string_view GetResource_frcUserProgramDrive_out();
std::string_view GetResource_frcUserProgramMechanism_out();
}  // namespace sysid

// Lock for Execute(). Only one instance of Execute() should be running at a
// time (even across separate instances).
wpi::mutex s_mutex;

DeploySession::DeploySession(std::string_view team, bool drive,
                             wpi::json config, wpi::Logger& logger)
    : m_drive{drive}, m_config(std::move(config)), m_logger{logger} {
  // Check whether we have an IP/hostname or team number.
  auto maybeTeam = wpi::parse_integer<int>(team, 10);

  // Depending on whether we have a team number, get a list of plausible
  // addresses or just set the list to the provided IP/hostname.
  m_addresses = maybeTeam.has_value() ? GetAddressesToTry(maybeTeam.value())
                                      : std::vector{std::string{team}};
}

void DeploySession::Execute(wpi::uv::Loop& lp) {
  // Lock mutex.
  std::scoped_lock lock{s_mutex};

  // First, we will attempt to resolve the hostnames of all the plausible
  // addresses. For every successful resolution, we will attempt a connection.
  // The first address to connect wins and the deploy will execute.
  for (auto&& host : m_addresses) {
    DEBUG("Attempting to resolve {} hostname.", host);

    // Create a request object for the call.
    auto req = std::make_shared<wpi::uv::GetAddrInfoReq>();

    // Set the error handler for the request.
    req->error = [this, &host, &lp](wpi::uv::Error e) {
      m_visited.fetch_add(1);
      lp.error(e);
      DEBUG("Could not resolve {}: {}.", host, e.str());
    };

    // Set the address-resolved handler for the request.
    req->resolved.connect([this, &host, &lp](const addrinfo& i) {
      // Convert to IP address.
      wpi::SmallString<16> ip;
      wpi::uv::AddrToName(reinterpret_cast<sockaddr_in*>(i.ai_addr)->sin_addr,
                          &ip);
      DEBUG("Resolved {} to {}.", host, ip);

      // Attempt an SSH connection in a separate worker thread. If the
      // connection is successful, continue the deploy from there.
      // First, create a request object for the call.
      auto wreq = std::make_shared<wpi::uv::WorkReq>();

      // Connect after work handler.
      wreq->afterWork.connect([this] { m_visited.fetch_add(1); });

      // Connect work handler.
      // We capture ip by value because the ref might be gone when the
      // callback is invoked.
      wreq->work.connect([this, &host, ip] {
        DEBUG("Trying to establish SSH connection to {}.", host);
        try {
          // Check if a connection has already been established. If it
          // has, then we don't want to waste time and resources trying
          // to establish another one.
          if (m_connected) {
            return;
          }

          // Try to start a connection.
          SshSession session{ip.str(), kPort, kUsername, kPassword, m_logger};
          session.Open();
          DEBUG("SSH connection to {} was successful.", host);

          // If we were successful, continue with the deploy (after
          // making sure that no other thread also reached this location
          // at the same time).
          if (m_connected.exchange(true)) {
            return;
          }

          SUCCESS("{}", "roboRIO Connected!");

          try {
            // As far as I can tell, the order of this doesn't matter.
            // It only comments out some stuff for the LabVIEW runtime that
            // apparently isn't needed, and dramatically reduces memory usage.
            // See https://github.com/wpilibsuite/EclipsePlugins/pull/154
            session.Execute(
                "sed -i -e 's/^StartupDLLs/;StartupDLLs/' "
                "/etc/natinst/share/ni-rt.ini");

            // Pre-Deploy
            session.Execute(
                ". /etc/profile.d/natinst-path.sh; "
                "/usr/local/frc/bin/frcKillRobot.sh "
                "-t 2> /dev/null");
            session.Execute("rm -f \"/home/lvuser/frcUserProgram\"");
            session.Execute("mkdir -p /home/lvuser/deploy");
            session.Execute(
                "chmod -R 777 /home/lvuser/deploy || true; chown -R admin:ni "
                "/home/lvuser/deploy");

            // Deploy
            session.Put("/home/lvuser/frcUserProgram",
                        m_drive ? GetResource_frcUserProgramDrive_out()
                                : GetResource_frcUserProgramMechanism_out());
            session.Put("/home/lvuser/deploy/config.json", m_config.dump());

            // Post-Deploy
            session.Execute(
                "chmod -R 777 /home/lvuser/deploy || true; chown -R lvuser:ni "
                "/home/lvuser/deploy");
            session.Execute(
                "echo ' \"/home/lvuser/frcUserProgram\" ' > "
                "/home/lvuser/robotCommand");
            session.Execute(
                "chmod +x /home/lvuser/robotCommand; chown lvuser "
                "/home/lvuser/robotCommand");
            session.Execute(
                "chmod +x \"/home/lvuser/frcUserProgram\"; chown lvuser "
                "\"/home/lvuser/frcUserProgram\"");
            session.Execute(
                "setcap cap_sys_nice+eip \"/home/lvuser/frcUserProgram\"");
            session.Execute("sync");
            session.Execute("ldconfig");
            session.Execute(
                ". /etc/profile.d/natinst-path.sh; "
                "/usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null");

            SUCCESS("{}", "Deploy Complete!");
          } catch (const SshSession::SshException& e) {
            ERROR("An exception occurred: {}", e.what());
          }
        } catch (const SshSession::SshException& e) {
          DEBUG("SSH connection to {} failed.", host);
        }
      });

      // Queue the work.
      wpi::uv::QueueWork(lp, wreq);
    });

    // Resolve.
    wpi::uv::GetAddrInfo(lp, req, host);
  }
}

std::vector<std::string> DeploySession::GetAddressesToTry(int team) {
  return {
      // These 3 should catch the roboRIO when it's connected to a radio or
      // a computer via USB.
      fmt::format("roborio-{}-FRC.local", team),
      fmt::format("10.{}.{}.2", team / 100, team % 100), "172.22.11.2",

      // The remaining cases are for weird environments, like a home network,
      // practice field, or otherwise.
      fmt::format("roborio-{}-FRC", team),
      fmt::format("roborio-{}-FRC.lan", team),
      fmt::format("roborio-{}.frc-field.local", team)};
}

DeploySession::Status DeploySession::GetStatus() const {
  // If we haven't visited everything yet, then we are still in progress.
  if (m_visited < m_addresses.size()) {
    return Status::kInProgress;
  }

  // If we have visited all locations, then there was either a discovery error
  // or we are done (indicated by whether m_connected is set).
  return m_connected ? Status::kDone : Status::kDiscoveryFailure;
}
