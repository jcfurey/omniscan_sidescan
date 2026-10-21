// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace omniscan_sidescan
{
using SteadyClock = std::chrono::steady_clock;
using Deadline = SteadyClock::time_point;

struct SonarError : std::runtime_error {using std::runtime_error::runtime_error;};
struct ConnectionError : SonarError {using SonarError::SonarError;};
struct TimeoutError : SonarError {using SonarError::SonarError;};
struct DataError : SonarError {using SonarError::SonarError;};
struct ConfigurationError : SonarError {using SonarError::SonarError;};

struct Params
{
  int64_t speed_of_sound_mm{1482000};
  int64_t start_mm{0};
  int64_t length_mm{5000};
  int64_t msec_per_ping{0};
  double pulse_len_percent{0.002};
  double filter_duration_percent{0.0015};
  int64_t gain_index{-1};
  int64_t num_results{600};

  void validate() const;
  bool operator==(const Params &) const = default;
};

struct Packet
{
  uint16_t id{};
  std::vector<uint8_t> payload;
  Deadline received{};
  int64_t timestamp_ns{};
};

struct Profile
{
  uint32_t ping_number{}, start_mm{}, length_mm{}, timestamp_ms{}, ping_hz{};
  uint16_t gain_index{}, num_results{}, sos_dmps{};
  uint8_t channel_number{};
  float pulse_duration_sec{}, analog_gain{}, max_pwr_db{}, min_pwr_db{};
  float transducer_heading_deg{}, vehicle_heading_deg{};
  std::vector<uint16_t> raw;
  std::vector<float> scaled;
  Deadline received{};
  int64_t timestamp_ns{};
};

namespace protocol
{
constexpr uint16_t nack = 2;
constexpr uint16_t device_information = 4;
constexpr uint16_t general_request = 6;
constexpr uint16_t speed_of_sound = 116;
constexpr uint16_t sync_channel = 169;
constexpr uint16_t set_sync_channel = 170;
constexpr uint16_t ping_params = 2197;
constexpr uint16_t mono_profile = 2198;
constexpr size_t max_results = 1200;

uint16_t u16(std::span<const uint8_t> data, size_t offset);
std::vector<uint8_t> encode(uint16_t id, std::span<const uint8_t> payload);
std::vector<uint8_t> request(uint16_t id);
std::vector<uint8_t> sound_command(const Params & params);
std::vector<uint8_t> sync_command(uint8_t channel, uint8_t count);
std::vector<uint8_t> ping_command(const Params & params, bool enabled = true);
Profile decode_profile(const Packet & packet);
bool matches(const Profile & profile, const Params & params);

// Feed bounded chunks. Partial frames expire so a plausible truncated header
// cannot permanently hide subsequent packets. Timestamps describe receipt of
// the complete packet, not the sonar's acoustic acquisition time.
class Parser
{
public:
  std::vector<Packet> feed(
    std::span<const uint8_t> bytes, Deadline received, int64_t timestamp_ns);
  void expire(Deadline now);
  std::optional<Deadline> assembly_deadline() const;
  uint64_t errors() const {return errors_;}
  size_t buffered_bytes() const {return buffer_.size();}

private:
  struct Receipt
  {
    uint64_t end_offset;
    Deadline received;
    int64_t timestamp_ns;
  };
  void consume(size_t count);
  std::vector<uint8_t> buffer_;
  std::deque<Receipt> receipts_;
  uint64_t buffer_offset_{};
  Deadline partial_since_{};
  uint64_t errors_{};
};
}  // namespace protocol
}  // namespace omniscan_sidescan
