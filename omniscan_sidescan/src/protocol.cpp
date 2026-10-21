// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/protocol.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <string>

namespace omniscan_sidescan
{
namespace
{
template<typename T>
void limit(const char * name, T value, T minimum, T maximum)
{
  if (!(value >= minimum && value <= maximum)) {
    throw std::invalid_argument(std::string(name) + " outside supported range");
  }
}

uint32_t u32(std::span<const uint8_t> data, size_t offset)
{
  return static_cast<uint32_t>(protocol::u16(data, offset)) |
         (static_cast<uint32_t>(protocol::u16(data, offset + 2)) << 16);
}

float f32(std::span<const uint8_t> data, size_t offset)
{
  return std::bit_cast<float>(u32(data, offset));
}

void put16(std::vector<uint8_t> & data, size_t offset, uint16_t value)
{
  data.at(offset) = static_cast<uint8_t>(value);
  data.at(offset + 1) = static_cast<uint8_t>(value >> 8);
}

void put32(std::vector<uint8_t> & data, size_t offset, uint32_t value)
{
  put16(data, offset, static_cast<uint16_t>(value));
  put16(data, offset + 2, static_cast<uint16_t>(value >> 16));
}

void put_float(std::vector<uint8_t> & data, size_t offset, double value)
{
  put32(data, offset, std::bit_cast<uint32_t>(static_cast<float>(value)));
}

bool valid_size(uint16_t id, size_t size)
{
  switch (id) {
    case 1: return size == 2;  // ACK
    case 2: return size >= 2 && size <= 4096;  // NACK with optional text
    case 3: case 10: return size <= 4096;  // ASCII / JSON
    case 4: return size == 6;
    case 5: return size == 4;
    case 6: return size == 2;
    case 100: return size == 1;
    case 116: return size == 4;
    case 2197: return size == 36;
    case 2198: return size >= 52 && size <= 52 + 2 * protocol::max_results && size % 2 == 0;
    default: return false;
  }
}
}  // namespace

void Params::validate() const
{
  limit<int64_t>("speed_of_sound_mm", speed_of_sound_mm, 1482000, 1500000);
  limit<int64_t>("start_mm", start_mm, 0, 5000);
  limit<int64_t>("length_mm", length_mm, 1, 300000);
  limit<int64_t>("msec_per_ping", msec_per_ping, 0, 10);
  limit("pulse_len_percent", pulse_len_percent, 0.002, 0.004);
  limit("filter_duration_percent", filter_duration_percent, 0.0015, 0.0016);
  limit<int64_t>("gain_index", gain_index, -1, 7);
  limit<int64_t>("num_results", num_results, 200, 1200);
}

namespace protocol
{
uint16_t u16(std::span<const uint8_t> data, size_t offset)
{
  if (offset >= data.size() || data.size() - offset < 2) {
    throw DataError("truncated protocol field");
  }
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8);
}

std::vector<uint8_t> encode(uint16_t id, std::span<const uint8_t> payload)
{
  if (payload.size() > 4096) {throw DataError("packet exceeds supported payload size");}
  std::vector<uint8_t> bytes(payload.size() + 10, 0);
  bytes[0] = 'B';
  bytes[1] = 'R';
  put16(bytes, 2, static_cast<uint16_t>(payload.size()));
  put16(bytes, 4, id);
  std::copy(payload.begin(), payload.end(), bytes.begin() + 8);
  uint16_t checksum = 0;
  for (size_t i = 0; i < bytes.size() - 2; ++i) {
    checksum += bytes[i];
  }
  put16(bytes, bytes.size() - 2, checksum);
  return bytes;
}

std::vector<uint8_t> request(uint16_t id)
{
  return encode(general_request, std::array<uint8_t, 2>{
        static_cast<uint8_t>(id), static_cast<uint8_t>(id >> 8)});
}

std::vector<uint8_t> sound_command(const Params & params)
{
  params.validate();
  std::vector<uint8_t> data(4);
  put32(data, 0, static_cast<uint32_t>(params.speed_of_sound_mm));
  return encode(speed_of_sound, data);
}

std::vector<uint8_t> ping_command(const Params & params, bool enabled)
{
  params.validate();
  std::vector<uint8_t> data(36, 0);
  put32(data, 0, static_cast<uint32_t>(params.start_mm));
  put32(data, 4, static_cast<uint32_t>(params.length_mm));
  put32(data, 8, static_cast<uint32_t>(params.msec_per_ping));
  put_float(data, 20, params.pulse_len_percent);
  put_float(data, 24, params.filter_duration_percent);
  put16(data, 28, static_cast<uint16_t>(params.gain_index));
  put16(data, 30, static_cast<uint16_t>(params.num_results));
  data[32] = enabled ? 1 : 0;
  return encode(ping_params, data);
}

Profile decode_profile(const Packet & packet)
{
  const auto & d = packet.payload;
  if (packet.id != mono_profile || d.size() < 52) {throw DataError("invalid profile packet");}
  Profile p;
  p.ping_number = u32(d, 0);
  p.start_mm = u32(d, 4);
  p.length_mm = u32(d, 8);
  p.timestamp_ms = u32(d, 12);
  p.ping_hz = u32(d, 16);
  p.gain_index = u16(d, 20);
  p.num_results = u16(d, 22);
  p.sos_dmps = u16(d, 24);
  p.channel_number = d[26];
  p.pulse_duration_sec = f32(d, 28);
  p.analog_gain = f32(d, 32);
  p.max_pwr_db = f32(d, 36);
  p.min_pwr_db = f32(d, 40);
  p.transducer_heading_deg = f32(d, 44);
  p.vehicle_heading_deg = f32(d, 48);
  if (p.num_results < 200 || p.num_results > max_results || d.size() != 52 + 2u * p.num_results) {
    throw DataError("unsupported or inconsistent profile sample count");
  }
  for (auto value : {p.pulse_duration_sec, p.analog_gain, p.max_pwr_db,
      p.min_pwr_db, p.transducer_heading_deg, p.vehicle_heading_deg})
  {
    if (!std::isfinite(value)) {throw DataError("nonfinite profile metadata");}
  }
  if (p.min_pwr_db > p.max_pwr_db || p.length_mm == 0 || p.sos_dmps == 0 ||
    p.ping_hz == 0 || p.gain_index > 7 || p.pulse_duration_sec <= 0)
  {
    throw DataError("invalid profile geometry, gain or dB bounds");
  }
  p.raw.resize(p.num_results);
  p.scaled.resize(p.num_results);
  const double scale = (static_cast<double>(p.max_pwr_db) - p.min_pwr_db) / 65535.0;
  for (size_t i = 0; i < p.num_results; ++i) {
    p.raw[i] = u16(d, 52 + 2 * i);
    p.scaled[i] = static_cast<float>(p.min_pwr_db + p.raw[i] * scale);
    if (!std::isfinite(p.scaled[i])) {throw DataError("nonfinite scaled power");}
  }
  p.received = packet.received;
  p.timestamp_ns = packet.timestamp_ns;
  return p;
}

bool matches(const Profile & p, const Params & params)
{
  return p.start_mm == params.start_mm && p.length_mm == params.length_mm &&
         p.num_results == params.num_results &&
         std::abs(static_cast<int64_t>(p.sos_dmps) * 100 - params.speed_of_sound_mm) < 100 &&
         (params.gain_index == -1 || p.gain_index == params.gain_index);
}

void Parser::expire(Deadline now)
{
  const auto deadline = assembly_deadline();
  if (deadline && now >= *deadline) {
    if (!buffer_.empty()) {consume(1);}
    partial_since_ = {};
    ++errors_;
  }
}

std::optional<Deadline> Parser::assembly_deadline() const
{
  if (partial_since_ == Deadline{}) {return {};}
  return partial_since_ + std::chrono::seconds(1);
}

void Parser::consume(size_t count)
{
  buffer_.erase(buffer_.begin(), buffer_.begin() + count);
  buffer_offset_ += count;
  while (!receipts_.empty() && receipts_.front().end_offset <= buffer_offset_) {
    receipts_.pop_front();
  }
}

std::vector<Packet> Parser::feed(
  std::span<const uint8_t> bytes, Deadline received, int64_t timestamp_ns)
{
  std::vector<Packet> packets;
  expire(received);
  // Even callers feeding a large capture cannot grow the retained buffer past
  // one bounded partial frame plus one receive chunk.
  do {
    const auto count = std::min<size_t>(4096, bytes.size());
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.begin() + count);
    if (count) {
      receipts_.push_back({buffer_offset_ + buffer_.size(), received, timestamp_ns});
    }
    bytes = bytes.subspan(count);
    size_t cursor = 0;
    while (cursor < buffer_.size()) {
      if (buffer_[cursor] != 'B') {++cursor; continue;}
      if (buffer_.size() - cursor < 2) {break;}
      if (buffer_[cursor + 1] != 'R') {++cursor; continue;}
      if (buffer_.size() - cursor < 8) {break;}
      const std::span<const uint8_t> remaining(buffer_.data() + cursor, buffer_.size() - cursor);
      const auto size = u16(remaining, 2);
      const auto id = u16(remaining, 4);
      if (!valid_size(id, size)) {++errors_; ++cursor; continue;}
      const size_t frame_size = size + 10;
      if (remaining.size() < frame_size) {break;}
      uint16_t checksum = 0;
      for (size_t i = 0; i < frame_size - 2; ++i) {
        checksum += remaining[i];
      }
      if (checksum != u16(remaining, frame_size - 2)) {++errors_; ++cursor; continue;}
      // Recovery behind a truncated packet must preserve the receipt time of
      // the frame's last byte: resynchronization cannot make old data fresh.
      const auto frame_end = buffer_offset_ + cursor + frame_size;
      const auto receipt = std::find_if(receipts_.begin(), receipts_.end(),
          [frame_end](const Receipt & item) {return item.end_offset >= frame_end;});
      packets.push_back(Packet{id,
          std::vector<uint8_t>(remaining.begin() + 8, remaining.begin() + 8 + size),
          receipt->received, receipt->timestamp_ns});
      cursor += frame_size;
    }
    if (cursor) {
      consume(cursor);
      partial_since_ = {};
    }
    if (buffer_.empty()) {partial_since_ = {};} else if (partial_since_ == Deadline{}) {
      partial_since_ = received;
    }
  } while (!bytes.empty());
  return packets;
}
}  // namespace protocol
}  // namespace omniscan_sidescan
