// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#pragma once

#include "omniscan_sidescan/protocol.hpp"

#include <sys/socket.h>

#include <functional>
#include <string>

namespace omniscan_sidescan
{
struct Endpoint
{
  sockaddr_storage address{};
  socklen_t size{};
};

// DNS and /etc/hosts lookup without a blocking libc resolver or helper threads.
// check() may throw to cancel. Numeric addresses bypass DNS entirely. The optional
// server list allows deterministic testing against a local DNS server.
std::vector<Endpoint> resolve(
  const std::string & host, uint16_t port, Deadline deadline,
  const std::function<void()> & check, const std::string & servers = "");
}  // namespace omniscan_sidescan
