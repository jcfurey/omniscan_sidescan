// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#pragma once

#include "omniscan_sidescan/protocol.hpp"

#include <atomic>
#include <deque>
#include <exception>
#include <functional>
#include <optional>
#include <string>

namespace omniscan_sidescan
{
// A control-plane update is not a device failure. Reads may be resumed on the
// same stream; an interrupted configuration leaves uncertain hardware state.
struct SettingsSuperseded : SonarError {using SonarError::SonarError;};

struct ConnectionOptions
{
  struct Sync {uint8_t channel, count;};
  std::string address;
  uint16_t port{51200};
  double connect_timeout{5.0};
  double configuration_timeout{2.0};
  std::optional<Sync> sync;
};

// One worker owns a Connection. No socket, parser or protocol state is shared
// between physical heads. Blocking operations check cancellation every 50 ms.
class Connection
{
public:
  using ClockNs = std::function<int64_t()>;
  using Interrupt = std::function<bool()>;
  Connection(ConnectionOptions options, const std::atomic_bool & cancelled, ClockNs clock_ns);
  ~Connection();
  Connection(const Connection &) = delete;
  Connection & operator=(const Connection &) = delete;

  void initialize(const Params & params, const Interrupt & interrupted = {});
  void configure(const Params & params, const Interrupt & interrupted = {});
  Profile read_profile(double timeout, const Interrupt & interrupted = {});
  void close() noexcept;
  uint64_t parser_errors() const {return parser_.errors();}
  bool sync_verified() const {return sync_verified_;}

private:
  void connect(const Interrupt & interrupted);
  void check_cancelled() const;
  void check_interrupted(const Interrupt & interrupted) const;
  bool wait(
    short events, Deadline deadline, bool ignore_cancel = false,
    const Interrupt & interrupted = {});
  void write(std::span<const uint8_t> bytes, Deadline deadline, bool ignore_cancel = false);
  std::optional<Packet> read(
    Deadline deadline, bool block = true,
    const Interrupt & interrupted = {});
  void information(Deadline deadline, const Interrupt & interrupted = {});
  void synchronize(const Interrupt & interrupted);
  void check_nack(const Packet & packet) const;

  ConnectionOptions options_;
  const std::atomic_bool & cancelled_;
  ClockNs clock_ns_;
  int socket_{-1};
  protocol::Parser parser_;
  std::deque<Packet> pending_;
  Params params_;
  std::exception_ptr read_error_;
  std::optional<Params> commanded_ping_params_;
  bool configured_{false};
  bool sync_verified_{false};
};
}  // namespace omniscan_sidescan
