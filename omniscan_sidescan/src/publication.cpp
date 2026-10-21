// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/publication.hpp"

#include <stdexcept>
#include <utility>

namespace omniscan_sidescan
{
Publication::~Publication() {stop(); join();}

void Publication::start()
{
  std::lock_guard lock(mutex_);
  if (thread_.joinable()) {throw std::logic_error("publication worker is already started");}
  stopped_ = false;
  accepting_ = true;
  thread_ = std::thread(&Publication::run, this);
}

bool Publication::submit(Task task)
{
  std::lock_guard lock(mutex_);
  if (stopped_ || !accepting_) {return false;}
  if (pending_) {++coalesced_;}
  pending_ = std::move(task);
  wake_.notify_one();
  return true;
}

void Publication::discard_pending() {std::lock_guard lock(mutex_); pending_ = {};}

void Publication::pause()
{
  std::lock_guard lock(mutex_);
  accepting_ = false;
  pending_ = {};
}

void Publication::resume()
{
  std::lock_guard lock(mutex_);
  if (stopped_) {throw std::logic_error("publication worker is stopped");}
  accepting_ = true;
}

void Publication::wait_idle()
{
  std::unique_lock lock(mutex_);
  wake_.wait(lock, [this] {return !running_;});
}

void Publication::stop()
{
  std::lock_guard lock(mutex_);
  stopped_ = true;
  accepting_ = false;
  pending_ = {};
  wake_.notify_all();
}

void Publication::join() {if (thread_.joinable()) {thread_.join();}}

void Publication::run()
{
  while (true) {
    Task task;
    {
      std::unique_lock lock(mutex_);
      wake_.wait(lock, [this] {return stopped_ || pending_;});
      if (stopped_) {return;}
      task = std::move(pending_);
      pending_ = {};
      running_ = true;
    }
    // No application mutex is held across the middleware call.
    try {
      task();
    } catch (...) {
      ++failures_;
    }
    {
      std::lock_guard lock(mutex_);
      running_ = false;
      wake_.notify_all();
    }
  }
}
}  // namespace omniscan_sidescan
