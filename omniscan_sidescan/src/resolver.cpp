// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/resolver.hpp"

#include <ares.h>
#include <netdb.h>
#include <poll.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <exception>
#include <map>
#include <memory>

namespace omniscan_sidescan
{
namespace
{
void require_success(int status)
{
  if (status != ARES_SUCCESS) {
    throw ConnectionError(std::string("resolve: ") + ares_strerror(status));
  }
}

struct Library
{
  Library() {require_success(ares_library_init(ARES_LIB_INIT_ALL));}
  ~Library() {ares_library_cleanup();}
};

struct Query
{
  bool done{};
  int status{ARES_SUCCESS};
  std::exception_ptr error;
  std::map<ares_socket_t, short> sockets;
  std::unique_ptr<ares_addrinfo, decltype(&ares_freeaddrinfo)> result{nullptr, ares_freeaddrinfo};
};

void socket_state(void * argument, ares_socket_t socket, int readable, int writable) noexcept
{
  auto & query = *static_cast<Query *>(argument);
  try {
    if (!readable && !writable) {query.sockets.erase(socket);} else {
      query.sockets[socket] = (readable ? POLLIN : 0) | (writable ? POLLOUT : 0);
    }
  } catch (...) {
    query.error = std::current_exception();
  }
}

void resolved(void * argument, int status, int, ares_addrinfo * result) noexcept
{
  auto & query = *static_cast<Query *>(argument);
  query.done = true;
  query.status = status;
  query.result.reset(result);
}
}  // namespace

std::vector<Endpoint> resolve(
  const std::string & host, uint16_t port, Deadline deadline,
  const std::function<void()> & check, const std::string & servers)
{
  check();
  const auto service = std::to_string(port);
  addrinfo numeric_hints{};
  numeric_hints.ai_family = AF_UNSPEC;
  numeric_hints.ai_socktype = SOCK_STREAM;
  numeric_hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
  addrinfo * numeric = nullptr;
  const auto numeric_status = getaddrinfo(host.c_str(), service.c_str(), &numeric_hints, &numeric);
  std::unique_ptr<addrinfo, decltype(& freeaddrinfo)> numeric_result(numeric, freeaddrinfo);
  std::vector<Endpoint> endpoints;
  const auto append = [&endpoints](const sockaddr * address, socklen_t size) {
      if (size > sizeof(sockaddr_storage)) {throw ConnectionError("resolve: oversized address");}
      Endpoint endpoint;
      std::memcpy(&endpoint.address, address, size);
      endpoint.size = size;
      endpoints.push_back(endpoint);
    };
  if (numeric_status == 0) {
    for (auto * address = numeric; address; address = address->ai_next) {
      append(address->ai_addr, address->ai_addrlen);
    }
    return endpoints;
  }
  static const Library library;
  Query query;
  ares_options options{};
  options.sock_state_cb = socket_state;
  options.sock_state_cb_data = &query;
  ares_channel_t * raw_channel = nullptr;
  require_success(ares_init_options(&raw_channel, &options, ARES_OPT_SOCK_STATE_CB));
  // Destruction cancels outstanding queries before their callback state dies.
  std::unique_ptr<ares_channel_t, decltype(& ares_destroy)> channel(raw_channel, ares_destroy);
  if (!servers.empty()) {
    require_success(ares_set_servers_ports_csv(channel.get(), servers.c_str()));
  }
  ares_addrinfo_hints hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  // Avoid c-ares probing resolved hosts to sort them; TCP uses the same deadline.
  hints.ai_flags = ARES_AI_NOSORT | ARES_AI_NUMERICSERV;
  ares_getaddrinfo(channel.get(), host.c_str(), service.c_str(), &hints, resolved, &query);
  while (!query.done) {
    check();
    if (query.error) {std::rethrow_exception(query.error);}
    const auto remaining = std::chrono::duration<double,
        std::milli>(deadline - SteadyClock::now()).count();
    if (remaining <= 0) {throw TimeoutError("DNS lookup timed out within connect_timeout");}
    std::vector<pollfd> descriptors;
    for (const auto & [socket, events] : query.sockets) {
      descriptors.push_back({socket, events, 0});
    }
    timeval maximum{0, static_cast<suseconds_t>(std::clamp(remaining, 1.0, 50.0) * 1000)};
    timeval next{};
    const auto * timeout = ares_timeout(channel.get(), &maximum, &next);
    const auto milliseconds = timeout->tv_sec * 1000 + (timeout->tv_usec + 999) / 1000;
    const auto count = ::poll(descriptors.data(), descriptors.size(),
        static_cast<int>(milliseconds));
    if (count < 0) {
      if (errno == EINTR) {continue;}
      throw ConnectionError(std::string("DNS poll: ") + std::strerror(errno));
    }
    if (count == 0) {ares_process_fd(channel.get(), ARES_SOCKET_BAD, ARES_SOCKET_BAD);}
    for (const auto & descriptor : descriptors) {
      if (descriptor.revents) {
        const auto read = descriptor.revents &
          (POLLIN | POLLERR | POLLHUP) ? descriptor.fd : ARES_SOCKET_BAD;
        const auto write = descriptor.revents & POLLOUT ? descriptor.fd : ARES_SOCKET_BAD;
        ares_process_fd(channel.get(), read, write);
      }
    }
  }
  check();
  require_success(query.status);
  for (auto * address = query.result->nodes; address; address = address->ai_next) {
    append(address->ai_addr, address->ai_addrlen);
  }
  if (endpoints.empty()) {throw ConnectionError("resolve: no TCP addresses");}
  return endpoints;
}
}  // namespace omniscan_sidescan
