// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#pragma once

#include "omniscan_sidescan/connection.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace omniscan_sidescan
{
struct Sample
{
  Profile profile;
  uint64_t generation{}, sequence{};
};

struct ChannelSnapshot
{
  std::string state{"disconnected"}, error;
  std::shared_ptr<const Sample> sample;
  uint64_t desired_generation{};
  int64_t applied_generation{-1};
  std::optional<Params> applied_params;
  uint64_t connection_attempts{}, connections{}, profiles_received{}, duplicate_profiles{},
    parser_errors{};
};

struct ChannelOptions
{
  ConnectionOptions connection;
  uint64_t max_consecutive_failures{10};
  double reconnect_initial_delay{0.25}, reconnect_max_delay{10.0};
};

class Channel
{
public:
  using Notify = std::function<void()>;
  using Log = std::function<void(const std::string &, bool)>;
  Channel(
    ChannelOptions options, Params params, uint64_t generation, double read_timeout,
    Connection::ClockNs clock, Notify notify, Log log);
  ~Channel();
  Channel(const Channel &) = delete;
  Channel & operator=(const Channel &) = delete;

  void start();
  void stop();
  void join();
  void configure(Params params, uint64_t generation, double read_timeout);
  ChannelSnapshot snapshot() const;

private:
  struct Settings {Params params; uint64_t generation; double read_timeout;};
  Settings settings() const;
  void status(const std::string & state, const std::string & error = "", bool reset = false);
  void run();
  void acquire(Connection & connection, uint64_t generation);

  ChannelOptions options_;
  Settings settings_;
  Connection::ClockNs clock_;
  Notify notify_;
  Log log_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::atomic_bool cancelled_{false};
  ChannelSnapshot snapshot_;
  uint64_t sequence_{}, session_parser_errors_{};
  std::thread thread_;
};
}  // namespace omniscan_sidescan
