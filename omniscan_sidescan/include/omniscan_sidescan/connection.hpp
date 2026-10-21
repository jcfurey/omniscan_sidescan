// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#pragma once

#include "omniscan_sidescan/protocol.hpp"

#include <atomic>
#include <deque>
#include <functional>
#include <optional>
#include <string>

namespace omniscan_sidescan
{
struct ConnectionOptions
{
  std::string address;
  uint16_t port{51200};
  double connect_timeout{5.0};
  double configuration_timeout{2.0};
};

// One worker owns a Connection. No socket, parser or protocol state is shared
// between physical heads. Blocking operations check cancellation every 50 ms.
class Connection
{
public:
  using ClockNs = std::function<int64_t()>;
  Connection(ConnectionOptions options, const std::atomic_bool & cancelled, ClockNs clock_ns);
  ~Connection();
  Connection(const Connection &) = delete;
  Connection & operator=(const Connection &) = delete;

  void initialize(const Params & params);
  void configure(const Params & params);
  Profile read_profile(double timeout);
  void close() noexcept;
  uint64_t parser_errors() const {return parser_.errors();}

private:
  void connect();
  void check_cancelled() const;
  bool wait(short events, Deadline deadline, bool ignore_cancel = false);
  void write(std::span<const uint8_t> bytes, Deadline deadline, bool ignore_cancel = false);
  std::optional<Packet> read(Deadline deadline, bool block = true);
  void information(Deadline deadline);
  void check_nack(const Packet & packet) const;

  ConnectionOptions options_;
  const std::atomic_bool & cancelled_;
  ClockNs clock_ns_;
  int socket_{-1};
  protocol::Parser parser_;
  std::deque<Packet> pending_;
  Params params_;
  bool configured_{false};
};
}  // namespace omniscan_sidescan
