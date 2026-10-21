// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace omniscan_sidescan
{
// One in-flight publication plus one replaceable latest task. Each output owns
// its worker, so DDS backpressure cannot reach acquisition or another output.
class Publication
{
public:
  using Task = std::function<void()>;
  ~Publication();
  void start();
  bool submit(Task task);
  void discard_pending();
  void pause();
  void resume();
  void wait_idle();
  void stop();
  void join();
  uint64_t coalesced() const {return coalesced_.load();}
  uint64_t failures() const {return failures_.load();}

private:
  void run();
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopped_{true};
  bool accepting_{false}, running_{false};
  Task pending_;
  std::thread thread_;
  std::atomic<uint64_t> coalesced_{0}, failures_{0};
};
}  // namespace omniscan_sidescan
