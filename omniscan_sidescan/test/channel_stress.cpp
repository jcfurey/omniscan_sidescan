// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/channel.hpp"

#include <array>
#include <iostream>
#include <thread>

namespace os = omniscan_sidescan;

namespace
{
void wait_streaming(const std::vector<std::unique_ptr<os::Channel>> & channels, uint64_t generation)
{
  const auto deadline = os::SteadyClock::now() + std::chrono::seconds(10);
  while (os::SteadyClock::now() < deadline) {
    bool ready = true;
    for (const auto & channel : channels) {
      const auto snapshot = channel->snapshot();
      ready = ready && snapshot.state == "streaming" && snapshot.sample &&
        snapshot.sample->generation == generation;
      if (snapshot.state == "error") {throw std::runtime_error(snapshot.error);}
    }
    if (ready) {return;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("channels did not converge to the committed configuration");
}
}  // namespace

// ROS-free stress helper, also compiled directly with thread/address sanitizers.
// The Python test supplies four independent protocol simulators over real TCP.
int main(int argc, char ** argv)
{
  try {
    if (argc != 5) {throw std::invalid_argument("expected four TCP ports");}
    std::atomic<uint64_t> notifications{}, observations{}, violations{};
    std::vector<std::unique_ptr<os::Channel>> channels;
    os::Params params;
    params.num_results = 200;
    for (size_t i = 0; i < 4; ++i) {
      os::ChannelOptions options;
      options.connection.address = "127.0.0.1";
      const auto port = std::stoul(argv[i + 1]);
      if (port == 0 || port > 65535) {throw std::invalid_argument("invalid TCP port");}
      options.connection.port = static_cast<uint16_t>(port);
      options.connection.connect_timeout = 0.5;
      options.connection.configuration_timeout = 1.0;
      options.reconnect_initial_delay = 0.005;
      options.reconnect_max_delay = 0.04;
      channels.push_back(std::make_unique<os::Channel>(options, params, 0, 0.15,
        [] {
          return std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count();
          }, [&notifications] {++notifications;}, os::Channel::Log{}));
    }
    std::vector<std::jthread> observers;
    for (size_t reader = 0; reader < 4; ++reader) {
      observers.emplace_back([&](std::stop_token stop) {
          std::array<uint64_t, 4> last_sequence{}, last_generation{};
          while (!stop.stop_requested()) {
            for (size_t i = 0; i < channels.size(); ++i) {
              const auto s = channels[i]->snapshot();
              ++observations;
              if (s.desired_generation < last_generation[i]) {++violations;}
              last_generation[i] = s.desired_generation;
              if (s.state == "streaming" && !s.sample) {++violations;}
              if (!s.sample) {continue;}
              const auto & sample = *s.sample;
              if (s.state != "streaming" || s.applied_generation < 0 ||
              sample.generation != s.desired_generation ||
              sample.generation != static_cast<uint64_t>(s.applied_generation) ||
              sample.sequence < last_sequence[i] || sample.profile.num_results != 200 ||
              sample.profile.raw.size() != 200 || sample.profile.scaled.size() != 200 ||
              sample.profile.length_mm != 5000 + sample.generation * 100)
              {++violations;}
              last_sequence[i] = sample.sequence;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
          }
        });
    }
    uint64_t generation = 0;
    for (size_t round = 0; round < 3; ++round) {
      for (auto & channel : channels) {
        channel->start();
      }
      wait_streaming(channels, generation);
      for (size_t update = 0; update < 20; ++update) {
        params.length_mm = 5000 + ++generation * 100;
        for (auto & channel : channels) {
          channel->configure(params, generation, generation % 2 ? 60.0 : 0.15);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
      }
      wait_streaming(channels, generation);
      for (auto & channel : channels) {
        channel->stop();
      }
      for (auto & channel : channels) {
        channel->join();
      }
      for (const auto & channel : channels) {
        const auto s = channel->snapshot();
        if (s.state != "stopped" || s.sample || s.applied_generation != -1) {++violations;}
      }
    }
    for (auto & observer : observers) {
      observer.request_stop();
    }
    observers.clear();
    std::cout << "{\"channels\":4,\"rounds\":3,\"generation\":" << generation <<
      ",\"observations\":" << observations.load() << ",\"notifications\":" <<
      notifications.load() <<
      ",\"violations\":" << violations.load() << "}\n";
    return violations.load() == 0 ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
