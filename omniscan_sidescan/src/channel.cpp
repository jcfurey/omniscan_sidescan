// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/channel.hpp"

#include <algorithm>
#include <utility>

namespace omniscan_sidescan
{
Channel::Channel(
  ChannelOptions options, Params params, uint64_t generation, double read_timeout,
  Connection::ClockNs clock, Notify notify, Log log)
: options_(std::move(options)), settings_{params, generation, read_timeout},
  clock_(std::move(clock)), notify_(std::move(notify)), log_(std::move(log))
{
  snapshot_.desired_generation = generation;
}

Channel::~Channel() {stop(); join();}

void Channel::start()
{
  cancelled_.store(false);
  thread_ = std::thread(&Channel::run, this);
}

void Channel::stop() {cancelled_.store(true); wake_.notify_all();}
void Channel::join() {if (thread_.joinable()) {thread_.join();}}

void Channel::configure(Params params, uint64_t generation, double read_timeout)
{
  std::lock_guard lock(mutex_);
  settings_ = {params, generation, read_timeout};
  if (generation != snapshot_.desired_generation) {
    snapshot_.sample.reset();
    if (snapshot_.state == "streaming" || snapshot_.state == "waiting") {
      snapshot_.state = "configuring";
    }
  }
  snapshot_.desired_generation = generation;
}

ChannelSnapshot Channel::snapshot() const {std::lock_guard lock(mutex_); return snapshot_;}
Channel::Settings Channel::settings() const {std::lock_guard lock(mutex_); return settings_;}

void Channel::status(const std::string & state, const std::string & error, bool reset)
{
  bool changed;
  {
    std::lock_guard lock(mutex_);
    changed = snapshot_.state != state || snapshot_.error != error;
    snapshot_.state = state;
    snapshot_.error = error;
    snapshot_.sample.reset();
    if (reset) {snapshot_.applied_generation = -1; snapshot_.applied_params.reset();}
  }
  // Never invoke external callbacks under the channel lock. ROS parameter
  // callbacks acquire the node lock before queueing channel settings.
  if (changed && log_) {log_(state + (error.empty() ? "" : ": " + error), !error.empty());}
  if (notify_) {notify_();}
}

void Channel::run()
{
  double delay = options_.reconnect_initial_delay;
  try {
    while (!cancelled_.load()) {
      const auto sequence_before = sequence_;
      {
        Connection connection(options_.connection, cancelled_, clock_);
        try {
          {
            std::lock_guard lock(mutex_);
            ++snapshot_.connection_attempts;
            session_parser_errors_ = snapshot_.parser_errors;
          }
          status("connecting", "", true);
          auto desired = settings();
          connection.initialize(desired.params);
          {
            std::lock_guard lock(mutex_);
            snapshot_.applied_generation = static_cast<int64_t>(desired.generation);
            snapshot_.applied_params = desired.params;
            ++snapshot_.connections;
          }
          status("configuring");
          acquire(connection, desired.generation);
        } catch (const SonarError & error) {
          if (!cancelled_.load()) {status("retrying", error.what(), true);}
        }
        {
          std::lock_guard lock(mutex_);
          snapshot_.parser_errors = session_parser_errors_ + connection.parser_errors();
        }
      }  // Stop streaming and close this head before its independent backoff.
      const bool streamed = sequence_ > sequence_before;
      if (streamed) {delay = options_.reconnect_initial_delay;}
      std::unique_lock lock(mutex_);
      const auto stopped = [this] {return cancelled_.load();};
      if (wake_.wait_for(lock, std::chrono::duration<double>(delay), stopped)) {
        break;
      }
      if (!streamed) {delay = std::min(delay * 2, options_.reconnect_max_delay);}
    }
    status("stopped", "", true);
  } catch (const std::exception & error) {
    status("error", error.what(), true);
  }
}

void Channel::acquire(Connection & connection, uint64_t generation)
{
  uint64_t failures = 0;
  std::optional<uint32_t> last_ping, last_timestamp;
  while (!cancelled_.load()) {
    auto desired = settings();
    if (generation != desired.generation) {
      status("configuring");
      connection.configure(desired.params);
      generation = desired.generation;
      {
        std::lock_guard lock(mutex_);
        snapshot_.applied_generation = static_cast<int64_t>(generation);
        snapshot_.applied_params = desired.params;
      }
    }
    try {
      auto profile = connection.read_profile(desired.read_timeout);
      {
        std::lock_guard lock(mutex_);
        ++snapshot_.profiles_received;
        snapshot_.parser_errors = session_parser_errors_ + connection.parser_errors();
      }
      if (last_ping) {
        const uint32_t ping_delta = profile.ping_number - *last_ping;
        const uint32_t time_delta = profile.timestamp_ms - *last_timestamp;
        if (ping_delta >= 0x80000000u || time_delta >= 0x80000000u) {
          throw ConnectionError("device counters moved backwards; reconnecting");
        }
        if (ping_delta == 0) {
          {std::lock_guard lock(mutex_); ++snapshot_.duplicate_profiles;}
          throw DataError("duplicate ping counter");
        }
      }
      last_ping = profile.ping_number;
      last_timestamp = profile.timestamp_ms;
      failures = 0;
      auto sample = std::make_shared<Sample>(Sample{std::move(profile), generation, ++sequence_});
      bool changed;
      {
        std::lock_guard lock(mutex_);
        if (generation != settings_.generation) {continue;}
        changed = snapshot_.state != "streaming";
        snapshot_.state = "streaming";
        snapshot_.error.clear();
        snapshot_.sample = std::move(sample);
      }
      if (changed && log_) {log_("streaming", false);}
      if (notify_) {notify_();}
    } catch (const TimeoutError & error) {
      {
        std::lock_guard lock(mutex_);
        snapshot_.parser_errors = session_parser_errors_ + connection.parser_errors();
      }
      status("waiting", error.what());
      if (options_.max_consecutive_failures && ++failures >= options_.max_consecutive_failures) {
        throw ConnectionError("persistent profile timeouts");
      }
    } catch (const DataError & error) {
      {
        std::lock_guard lock(mutex_);
        snapshot_.parser_errors = session_parser_errors_ + connection.parser_errors();
      }
      status("waiting", error.what());
      if (options_.max_consecutive_failures && ++failures >= options_.max_consecutive_failures) {
        throw ConnectionError("persistent invalid profiles");
      }
    }
  }
}
}  // namespace omniscan_sidescan
