// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include <rcl/publisher.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>

// Test-only interposition at the actual rcl -> DDS boundary. The driver has no
// fault-injection parameters or production dependency on this shared library.
extern "C" rcl_ret_t rcl_publish(
  const rcl_publisher_t * publisher, const void * message, rmw_publisher_allocation_t * allocation)
{
  using Publish = decltype(&rcl_publish);
  static auto publish = reinterpret_cast<Publish>(dlsym(RTLD_NEXT, "rcl_publish"));
  const auto * gate = std::getenv("OMNISCAN_STALL_GATE");
  const auto * ready = std::getenv("OMNISCAN_STALL_READY");
  const auto * selected_topic = std::getenv("OMNISCAN_STALL_TOPIC");
  const auto * topic = rcl_publisher_get_topic_name(publisher);
  if (!selected_topic) {selected_topic = "/port/profile";}
  if (gate && ready && topic && std::strstr(topic, selected_topic) && ::access(gate, F_OK) == 0) {
    const auto descriptor = ::open(ready, O_WRONLY | O_CREAT | O_CLOEXEC, 0600);
    if (descriptor >= 0) {::close(descriptor);}
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (::access(gate, F_OK) == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return publish(publisher, message, allocation);
}
