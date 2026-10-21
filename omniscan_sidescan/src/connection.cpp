// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/connection.hpp"
#include "omniscan_sidescan/resolver.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
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
  if (options_.sync &&
    (options_.sync->count == 0 || options_.sync->channel >= options_.sync->count))
  {
    throw std::invalid_argument("invalid sync channel group");
  }
}

Connection::~Connection() {close();}

void Connection::check_cancelled() const
{
  if (cancelled_.load()) {throw ConnectionError("connection cancelled");}
}

void Connection::check_interrupted(const Interrupt & interrupted) const
{
  check_cancelled();
  if (interrupted && interrupted()) {throw SettingsSuperseded("settings superseded");}
}

bool Connection::wait(
  short events, Deadline deadline, bool ignore_cancel,
  const Interrupt & interrupted)
{
  while (true) {
    if (!ignore_cancel) {check_interrupted(interrupted);}
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

void Connection::connect(const Interrupt & interrupted)
{
  const auto deadline = after(options_.connect_timeout);
  const auto addresses = resolve(options_.address, options_.port, deadline,
      [this, &interrupted] {check_interrupted(interrupted);});
  std::string error = "TCP connect timed out";
  for (const auto & address : addresses) {
    check_interrupted(interrupted);
    if (SteadyClock::now() >= deadline) {break;}
    socket_ = ::socket(address.address.ss_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {error = system_error("socket"); continue;}
    try {
      const auto connected = ::connect(socket_,
          reinterpret_cast<const sockaddr *>(&address.address), address.size);
      if (connected == 0) {return;}
      if (errno == EINPROGRESS && wait(POLLOUT, deadline, false, interrupted)) {
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

std::optional<Packet> Connection::read(
  Deadline deadline, bool block,
  const Interrupt & interrupted)
{
  while (true) {
    check_interrupted(interrupted);
    if (read_error_) {std::rethrow_exception(read_error_);}
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
    if (count == 0 || (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
      read_error_ = std::make_exception_ptr(ConnectionError(
          count == 0 ? "TCP peer closed the connection" : system_error("recv")));
      // A terminal error while draining must not discard the final complete
      // profile. Deliver it once; the next operation reports the broken stream.
      if (!block) {return {};}
      std::rethrow_exception(read_error_);
    }
    if (errno == EINTR) {continue;}
    if (!block) {return {};}
    // Frame assembly must expire even if the peer goes quiet during a long
    // profile/configuration wait. Keep the caller's overall deadline intact.
    const auto wake_deadline = std::min(deadline, parser_.assembly_deadline().value_or(deadline));
    if (!wait(POLLIN, wake_deadline, false, interrupted) && SteadyClock::now() >= deadline) {
      return {};
    }
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

void Connection::information(Deadline deadline, const Interrupt & interrupted)
{
  check_interrupted(interrupted);
  write(protocol::request(protocol::device_information), deadline);
  while (auto packet = read(deadline, true, interrupted)) {
    check_nack(*packet);
    if (packet->id == protocol::device_information) {return;}
    if (SteadyClock::now() >= deadline) {break;}
  }
  throw ConfigurationError("device information request timed out");
}

void Connection::initialize(const Params & params, const Interrupt & interrupted)
{
  if (socket_ >= 0) {throw std::logic_error("connection is already initialized");}
  params.validate();
  parser_ = {};
  pending_.clear();
  read_error_ = {};
  connect(interrupted);
  information(after(options_.configuration_timeout), interrupted);
  synchronize(interrupted);
  configure(params, interrupted);
}

void Connection::synchronize(const Interrupt & interrupted)
{
  sync_verified_ = false;
  if (!options_.sync) {return;}
  const auto expected = *options_.sync;
  const auto deadline = after(options_.configuration_timeout);
  check_interrupted(interrupted);
  write(protocol::sync_command(expected.channel, expected.count), deadline);
  // The ordered information barrier drains any readback from an earlier
  // session before requesting and checking this assignment explicitly.
  information(deadline, interrupted);
  write(protocol::request(protocol::sync_channel), deadline);
  while (auto packet = read(deadline, true, interrupted)) {
    check_nack(*packet);
    if (packet->id == protocol::sync_channel) {
      if (packet->payload[0] != expected.channel || packet->payload[1] != expected.count) {
        throw ConfigurationError("synchronization assignment readback differs from request");
      }
      sync_verified_ = true;
      return;
    }
    if (SteadyClock::now() >= deadline) {break;}
  }
  throw ConfigurationError("synchronization assignment readback timed out");
}

void Connection::configure(const Params & params, const Interrupt & interrupted)
{
  params.validate();
  check_interrupted(interrupted);
  if (configured_ && params == params_) {return;}
  const auto deadline = after(options_.configuration_timeout);
  if (!configured_ || params.speed_of_sound_mm != params_.speed_of_sound_mm) {
    write(protocol::sound_command(params), deadline);
  }
  auto without_speed = params;
  without_speed.speed_of_sound_mm = params_.speed_of_sound_mm;
  if (!configured_ || without_speed != params_) {
    write(protocol::ping_command(params), deadline);
    // The device may start pinging before it confirms the configuration.
    // Retain the complete command for cleanup without declaring it applied.
    commanded_ping_params_ = params;
  }
  // An ordered request forms a stream barrier: profiles buffered before the
  // command cannot confirm it. Commands need not ACK in the vendor protocol.
  information(deadline, interrupted);
  while (auto packet = read(deadline, true, interrupted)) {
    check_nack(*packet);
    if (packet->id == protocol::mono_profile) {
      try {
        auto profile = protocol::decode_profile(*packet);
        if (protocol::matches(profile, params) &&
          (!options_.sync || profile.channel_number == options_.sync->channel))
        {
          params_ = params;
          configured_ = true;
          // Confirmation is also a usable measurement. Let the next read
          // deliver it or select a newer buffered profile, keeping its receipt.
          pending_.push_front(std::move(*packet));
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

Profile Connection::read_profile(double timeout, const Interrupt & interrupted)
{
  if (!std::isfinite(timeout) || timeout <= 0) {
    throw std::invalid_argument("invalid profile timeout");
  }
  const auto deadline = after(timeout);
  std::optional<Packet> newest;
  while (auto packet = read(deadline, !newest.has_value(), interrupted)) {
    check_nack(*packet);
    if (packet->id == protocol::mono_profile) {newest = std::move(packet);}
    if (SteadyClock::now() >= deadline) {break;}
  }
  if (!newest) {throw TimeoutError("profile read timed out");}
  auto profile = protocol::decode_profile(*newest);
  if (!protocol::matches(profile, params_) ||
    (options_.sync && profile.channel_number != options_.sync->channel))
  {
    throw ConfigurationError("profile differs from confirmed settings");
  }
  return profile;
}

void Connection::close() noexcept
{
  if (socket_ < 0) {return;}
  if (commanded_ping_params_) {
    try {
      // A rejected update may also be rejected with enable=false. Prefer the
      // last confirmed parameters; startup only has its commanded parameters.
      const auto & stop_params = configured_ ? params_ : *commanded_ping_params_;
      const auto deadline = after(0.1);
      write(protocol::ping_command(stop_params, false), deadline, true);
      // Closing with unread data can reset TCP and discard the queued stop.
      // Half-close writes, then drain until EOF within the same cleanup budget.
      ::shutdown(socket_, SHUT_WR);
      std::array<uint8_t, 4096> discarded{};
      while (SteadyClock::now() < deadline) {
        const auto count = ::recv(socket_, discarded.data(), discarded.size(), MSG_DONTWAIT);
        if (count > 0 || (count < 0 && errno == EINTR)) {continue;}
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) &&
          wait(POLLIN, deadline, true)) {continue;}
        break;
      }
    } catch (...) {
    }
  }
  ::shutdown(socket_, SHUT_RDWR);
  ::close(socket_);
  socket_ = -1;
  commanded_ping_params_.reset();
  configured_ = false;
  sync_verified_ = false;
}
}  // namespace omniscan_sidescan
