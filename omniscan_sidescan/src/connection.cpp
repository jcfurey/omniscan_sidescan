// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/connection.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <memory>
#include <utility>

namespace omniscan_sidescan
{
namespace
{
Deadline after(double seconds)
{
  return SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(seconds));
}

std::string system_error(const char * action)
{
  return std::string(action) + ": " + std::strerror(errno);
}
}  // namespace

Connection::Connection(
  ConnectionOptions options, const std::atomic_bool & cancelled, ClockNs clock_ns)
: options_(std::move(options)), cancelled_(cancelled), clock_ns_(std::move(clock_ns))
{
  if (options_.address.empty() || options_.port == 0 ||
    !std::isfinite(options_.connect_timeout) || options_.connect_timeout <= 0 ||
    !std::isfinite(options_.configuration_timeout) || options_.configuration_timeout <= 0)
  {
    throw std::invalid_argument("invalid connection options");
  }
}

Connection::~Connection() {close();}

void Connection::check_cancelled() const
{
  if (cancelled_.load()) {throw ConnectionError("connection cancelled");}
}

bool Connection::wait(short events, Deadline deadline, bool ignore_cancel)
{
  while (true) {
    if (!ignore_cancel) {check_cancelled();}
    const auto remaining = std::chrono::duration<double, std::milli>(
      deadline - SteadyClock::now()).count();
    if (remaining <= 0) {return false;}
    pollfd descriptor{socket_, events, 0};
    const auto result = ::poll(&descriptor, 1, static_cast<int>(std::clamp(remaining, 1.0, 50.0)));
    if (result < 0) {
      if (errno == EINTR) {continue;}
      throw ConnectionError(system_error("poll"));
    }
    // POLLHUP may accompany readable bytes. Let recv consume these first and
    // report EOF; connecting sockets report their error through SO_ERROR.
    if (result > 0) {return true;}
  }
}

void Connection::connect()
{
  check_cancelled();
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * addresses = nullptr;
  // Numeric addresses resolve locally. Hostnames follow the operating system
  // resolver policy, just as the Python reference implementation does.
  const auto result = ::getaddrinfo(
    options_.address.c_str(), std::to_string(options_.port).c_str(), &hints, &addresses);
  if (result != 0) {throw ConnectionError(std::string("resolve: ") + gai_strerror(result));}
  std::unique_ptr<addrinfo, decltype(& freeaddrinfo)> resolved(addresses, freeaddrinfo);
  const auto deadline = after(options_.connect_timeout);
  std::string error = "TCP connect timed out";
  for (auto * address = addresses; address; address = address->ai_next) {
    check_cancelled();
    socket_ = ::socket(address->ai_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {error = system_error("socket"); continue;}
    try {
      const auto connected = ::connect(socket_, address->ai_addr, address->ai_addrlen);
      if (connected == 0) {return;}
      if (errno == EINPROGRESS && wait(POLLOUT, deadline)) {
        int status = 0;
        socklen_t size = sizeof(status);
        if (::getsockopt(socket_, SOL_SOCKET, SO_ERROR, &status, &size) == 0 && status == 0) {
          return;
        }
        error = std::string("connect: ") + std::strerror(status ? status : errno);
      } else if (errno != EINPROGRESS) {error = system_error("connect");}
    } catch (...) {
      ::close(socket_);
      socket_ = -1;
      throw;
    }
    ::close(socket_);
    socket_ = -1;
    if (SteadyClock::now() >= deadline) {break;}
  }
  throw ConnectionError(error);
}

void Connection::write(std::span<const uint8_t> bytes, Deadline deadline, bool ignore_cancel)
{
  while (!bytes.empty()) {
    if (!ignore_cancel) {check_cancelled();}
    if (SteadyClock::now() >= deadline) {throw TimeoutError("TCP write timed out");}
    const auto sent = ::send(socket_, bytes.data(), bytes.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent > 0) {bytes = bytes.subspan(static_cast<size_t>(sent)); continue;}
    if (sent < 0 && errno == EINTR) {continue;}
    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (wait(POLLOUT, deadline, ignore_cancel)) {continue;}
      throw TimeoutError("TCP write timed out");
    }
    throw ConnectionError(system_error("send"));
  }
}

std::optional<Packet> Connection::read(Deadline deadline, bool block)
{
  while (true) {
    check_cancelled();
    if (!pending_.empty()) {
      Packet packet = std::move(pending_.front());
      pending_.pop_front();
      return packet;
    }
    parser_.expire(SteadyClock::now());
    // Process bytes left after expiry even when the peer has gone quiet.
    auto packets = parser_.feed({}, SteadyClock::now(), clock_ns_());
    for (auto & packet : packets) {
      pending_.push_back(std::move(packet));
    }
    if (!pending_.empty()) {continue;}
    std::array<uint8_t, 4096> buffer{};
    const auto count = ::recv(socket_, buffer.data(), buffer.size(), MSG_DONTWAIT);
    if (count > 0) {
      packets = parser_.feed(
        std::span(buffer.data(), static_cast<size_t>(count)), SteadyClock::now(), clock_ns_());
      for (auto & packet : packets) {
        pending_.push_back(std::move(packet));
      }
      if (SteadyClock::now() >= deadline && pending_.empty()) {return {};}
      continue;
    }
    if (count == 0) {throw ConnectionError("TCP peer closed the connection");}
    if (errno == EINTR) {continue;}
    if (errno != EAGAIN && errno != EWOULDBLOCK) {throw ConnectionError(system_error("recv"));}
    if (!block || !wait(POLLIN, deadline)) {return {};}
  }
}

void Connection::check_nack(const Packet & packet) const
{
  if (packet.id == protocol::nack) {
    const auto rejected = protocol::u16(packet.payload, 0);
    const std::string text(packet.payload.begin() + 2, packet.payload.end());
    throw ConfigurationError("device rejected command " + std::to_string(rejected) + ": " + text);
  }
}

void Connection::information(Deadline deadline)
{
  write(protocol::request(protocol::device_information), deadline);
  while (auto packet = read(deadline)) {
    check_nack(*packet);
    if (packet->id == protocol::device_information) {return;}
    if (SteadyClock::now() >= deadline) {break;}
  }
  throw ConfigurationError("device information request timed out");
}

void Connection::initialize(const Params & params)
{
  if (socket_ >= 0) {throw std::logic_error("connection is already initialized");}
  params.validate();
  connect();
  information(after(options_.configuration_timeout));
  configure(params);
}

void Connection::configure(const Params & params)
{
  params.validate();
  if (configured_ && params == params_) {return;}
  const auto deadline = after(options_.configuration_timeout);
  if (!configured_ || params.speed_of_sound_mm != params_.speed_of_sound_mm) {
    write(protocol::sound_command(params), deadline);
  }
  auto without_speed = params;
  without_speed.speed_of_sound_mm = params_.speed_of_sound_mm;
  if (!configured_ || without_speed != params_) {write(protocol::ping_command(params), deadline);}
  // An ordered request forms a stream barrier: profiles buffered before the
  // command cannot confirm it. Commands need not ACK in the vendor protocol.
  information(deadline);
  while (auto packet = read(deadline)) {
    check_nack(*packet);
    if (packet->id == protocol::mono_profile) {
      try {
        auto profile = protocol::decode_profile(*packet);
        if (protocol::matches(profile, params)) {
          params_ = params;
          configured_ = true;
          return;
        }
      } catch (const DataError &) {
        // Malformed data cannot confirm a configuration; allow a later valid
        // profile to do so within the same bounded deadline.
      }
    }
    if (SteadyClock::now() >= deadline) {break;}
  }
  throw ConfigurationError("profiles did not confirm the requested configuration");
}

Profile Connection::read_profile(double timeout)
{
  if (!std::isfinite(timeout) || timeout <= 0) {
    throw std::invalid_argument("invalid profile timeout");
  }
  const auto deadline = after(timeout);
  std::optional<Packet> newest;
  while (auto packet = read(deadline, !newest.has_value())) {
    check_nack(*packet);
    if (packet->id == protocol::mono_profile) {newest = std::move(packet);}
    if (SteadyClock::now() >= deadline) {break;}
  }
  if (!newest) {throw TimeoutError("profile read timed out");}
  auto profile = protocol::decode_profile(*newest);
  if (!protocol::matches(profile, params_)) {
    throw ConfigurationError("profile differs from confirmed settings");
  }
  return profile;
}

void Connection::close() noexcept
{
  if (socket_ < 0) {return;}
  if (configured_) {
    try {
      write(protocol::ping_command(params_, false), after(0.1), true);
    } catch (...) {
    }
  }
  ::shutdown(socket_, SHUT_RDWR);
  ::close(socket_);
  socket_ = -1;
  configured_ = false;
}
}  // namespace omniscan_sidescan
