// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/publication.hpp"
#include "omniscan_sidescan/resolver.hpp"

#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <future>
#include <thread>

namespace os = omniscan_sidescan;
using namespace std::chrono_literals;

namespace
{
class DnsServer
{
public:
  explicit DnsServer(bool respond = false)
  {
    socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {throw std::runtime_error("DNS test socket failed");}
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      ::close(socket_);
      throw std::runtime_error("DNS test bind failed");
    }
    socklen_t size = sizeof(address);
    ::getsockname(socket_, reinterpret_cast<sockaddr *>(&address), &size);
    servers = "127.0.0.1:" + std::to_string(ntohs(address.sin_port));
    if (respond) {
      thread_ = std::jthread([this](std::stop_token stop) {
            while (!stop.stop_requested()) {
              pollfd descriptor{socket_, POLLIN, 0};
              if (::poll(&descriptor, 1, 10) <= 0) {continue;}
              std::array<uint8_t, 512> buffer{};
              sockaddr_storage peer{};
              socklen_t size = sizeof(peer);
              const auto count = ::recvfrom(socket_, buffer.data(), buffer.size(), 0,
                reinterpret_cast<sockaddr *>(&peer), &size);
              if (count < 16) {continue;}
              size_t end = 12;
              while (end < static_cast<size_t>(count) && buffer[end]) {end += buffer[end] + 1;}
              if (end + 5 > static_cast<size_t>(count)) {continue;}
              end += 5;
            // Copy only the question. Queries may also contain an EDNS OPT
            // record, which must not precede our answer in the response.
              std::vector<uint8_t> response(buffer.begin(), buffer.begin() + end);
              const bool ipv4 = response[response.size() - 3] == 1;
              response[2] = 0x81; response[3] = 0x80;
              response[6] = 0; response[7] = ipv4 ? 1 : 0;
              response[8] = response[9] = response[10] = response[11] = 0;
              if (ipv4) {
                const std::array<uint8_t, 16> answer{
                  0xc0, 0x0c, 0, 1, 0, 1, 0, 0, 0, 0, 0, 4, 127, 0, 0, 1};
                response.insert(response.end(), answer.begin(), answer.end());
              }
              ::sendto(socket_, response.data(), response.size(), 0,
              reinterpret_cast<sockaddr *>(&peer), size);
            }
        });
    }
  }
  ~DnsServer()
  {
    thread_.request_stop();
    if (thread_.joinable()) {thread_.join();}
    ::close(socket_);
  }
  std::string servers;

private:
  int socket_;
  std::jthread thread_;
};

struct Gate
{
  std::promise<void> promise;
  std::shared_future<void> ready{promise.get_future()};
  ~Gate() {open();}
  void open() {if (!opened.exchange(true)) {promise.set_value();}}
  std::atomic_bool opened{};
};
}  // namespace

TEST(Resolver, NumericIPv4IPv6AndLocalhost)
{
  for (const auto * name : {"127.0.0.1", "::1", "localhost"}) {
    auto addresses = os::resolve(name, 51200, os::SteadyClock::now() + 1s, [] {});
    ASSERT_FALSE(addresses.empty());
    for (const auto & endpoint : addresses) {
      EXPECT_TRUE(endpoint.address.ss_family == AF_INET || endpoint.address.ss_family == AF_INET6);
    }
  }
}

TEST(Resolver, IndependentDnsResponse)
{
  DnsServer server(true);
  auto addresses = os::resolve("sonar.example.", 51200, os::SteadyClock::now() + 1s,
      [] {}, server.servers);
  ASSERT_EQ(addresses.size(), 1u);
  const auto & address = reinterpret_cast<const sockaddr_in &>(addresses.front().address);
  EXPECT_EQ(address.sin_family, AF_INET);
  EXPECT_EQ(ntohl(address.sin_addr.s_addr), INADDR_LOOPBACK);
  EXPECT_EQ(ntohs(address.sin_port), 51200);
}

TEST(Resolver, UnresponsiveDnsUsesConnectionDeadline)
{
  DnsServer server;
  const auto started = os::SteadyClock::now();
  EXPECT_THROW(os::resolve("sonar.example.", 51200, started + 120ms, [] {}, server.servers),
    os::TimeoutError);
  EXPECT_GE(os::SteadyClock::now() - started, 100ms);
  EXPECT_LT(os::SteadyClock::now() - started, 400ms);
}

TEST(Resolver, CancelsConcurrentLongLookupsWithoutLingeringThreads)
{
  DnsServer server;
  std::atomic_bool stop{};
  std::atomic<int> cancelled{};
  std::vector<std::jthread> workers;
  const auto started = os::SteadyClock::now();
  for (size_t i = 0; i < 4; ++i) {
    workers.emplace_back([&] {
        try {
          os::resolve("sonar.example.", 51200, started + 60s,
          [&] {if (stop.load()) {throw os::ConnectionError("cancelled");}}, server.servers);
        } catch (const os::ConnectionError &) {
          ++cancelled;
        }
      });
  }
  std::this_thread::sleep_for(60ms);
  stop.store(true);
  workers.clear();
  EXPECT_EQ(cancelled.load(), 4);
  EXPECT_LT(os::SteadyClock::now() - started, 400ms);
}

TEST(Publication, StalledOutputCoalescesWithoutBlockingAnotherOutput)
{
  os::Publication stalled, healthy;
  Gate release;
  std::promise<void> entered, other_finished, latest_finished;
  auto entered_future = entered.get_future();
  auto other_future = other_finished.get_future();
  auto latest_future = latest_finished.get_future();
  stalled.start(); healthy.start();
  ASSERT_TRUE(stalled.submit([&] {entered.set_value(); release.ready.wait();}));
  ASSERT_EQ(entered_future.wait_for(1s), std::future_status::ready);
  std::atomic<int> last{};
  for (int i = 1; i <= 10000; ++i) {
    ASSERT_TRUE(stalled.submit([&, i] {
        last.store(i);
        if (i == 10000) {latest_finished.set_value();}
      }));
  }
  EXPECT_EQ(stalled.coalesced(), 9999u);
  ASSERT_TRUE(healthy.submit([&] {other_finished.set_value();}));
  EXPECT_EQ(other_future.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(last.load(), 0);
  release.open();
  EXPECT_EQ(latest_future.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(last.load(), 10000);
  stalled.stop(); healthy.stop(); stalled.join(); healthy.join();
}

TEST(Publication, StopDiscardsPendingWorkAndAllowsRestart)
{
  os::Publication output;
  Gate release;
  std::promise<void> entered;
  auto entered_future = entered.get_future();
  std::atomic_bool pending_ran{};
  output.start();
  output.submit([&] {entered.set_value(); release.ready.wait();});
  ASSERT_EQ(entered_future.wait_for(1s), std::future_status::ready);
  output.submit([&] {pending_ran.store(true);});
  output.stop();
  EXPECT_FALSE(output.submit([] {}));
  release.open(); output.join();
  EXPECT_FALSE(pending_ran.load());
  output.start();
  std::promise<void> restarted;
  auto restarted_future = restarted.get_future();
  output.submit([&] {restarted.set_value();});
  EXPECT_EQ(restarted_future.wait_for(1s), std::future_status::ready);
  output.stop(); output.join();
}

TEST(Publication, PauseDrainsInFlightAndReusesTheSameThreadOnResume)
{
  os::Publication output;
  Gate release;
  std::promise<std::thread::id> entered, resumed;
  auto entered_future = entered.get_future();
  auto resumed_future = resumed.get_future();
  std::atomic_bool pending_ran{};
  output.start();
  output.submit([&] {entered.set_value(std::this_thread::get_id()); release.ready.wait();});
  ASSERT_EQ(entered_future.wait_for(1s), std::future_status::ready);
  const auto thread = entered_future.get();
  output.submit([&] {pending_ran.store(true);});
  output.pause();
  EXPECT_FALSE(output.submit([] {}));
  auto idle = std::async(std::launch::async, [&] {output.wait_idle();});
  EXPECT_EQ(idle.wait_for(20ms), std::future_status::timeout);
  release.open();
  ASSERT_EQ(idle.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(pending_ran.load());
  output.resume();
  output.submit([&] {resumed.set_value(std::this_thread::get_id());});
  ASSERT_EQ(resumed_future.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(resumed_future.get(), thread);
  output.stop(); output.join();
}
