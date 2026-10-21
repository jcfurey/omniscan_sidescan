// Copyright 2026 The omniscan_sidescan contributors
// SPDX-License-Identifier: MIT
#include "omniscan_sidescan/protocol.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

namespace os = omniscan_sidescan;
namespace wire = omniscan_sidescan::protocol;

namespace
{
// An independent little-endian fixture, following the vendor's 52-byte profile
// layout. No native driver encoder is used to construct profile metadata.
os::Packet profile_fixture()
{
  os::Packet packet{2198, std::vector<uint8_t>(52 + 400, 0), os::SteadyClock::now(), 123456789};
  const auto set32 = [&packet](size_t offset, uint32_t value) {
      for (size_t i = 0; i < 4; ++i) {
        packet.payload[offset + i] = static_cast<uint8_t>(value >> (8 * i));
      }
    };
  set32(0, 0x12345678);
  set32(4, 1234);
  set32(8, 7000);
  set32(12, 0xfedcba98);
  set32(16, 450000);
  packet.payload[20] = 3;
  packet.payload[22] = 200;
  packet.payload[24] = 0xe4;  // 14820 dm/s
  packet.payload[25] = 0x39;
  packet.payload[26] = 2;
  set32(28, std::bit_cast<uint32_t>(0.0002f));
  set32(32, std::bit_cast<uint32_t>(-1.0f));  // Analog gain units are undocumented.
  set32(36, std::bit_cast<uint32_t>(-10.0f));
  set32(40, std::bit_cast<uint32_t>(-90.0f));
  set32(44, std::bit_cast<uint32_t>(180.0f));
  set32(48, std::bit_cast<uint32_t>(12.5f));
  packet.payload[54] = 0xff;
  packet.payload[55] = 0xff;
  packet.payload[56] = 0;
  packet.payload[57] = 0x80;
  return packet;
}
}  // namespace

TEST(Protocol, CanonicalSoundSpeedGoldenPacket)
{
  os::Params params;
  params.speed_of_sound_mm = 1500000;
  // Canonical API ID 116; deliberately independent of driver constants.
  const std::vector<uint8_t> expected{
    0x42, 0x52, 4, 0, 0x74, 0, 0, 0, 0x60, 0xe3, 0x16, 0, 0x65, 2};
  EXPECT_EQ(wire::sound_command(params), expected);
  EXPECT_EQ(wire::request(4), (std::vector<uint8_t>{0x42, 0x52, 2, 0, 6, 0, 0, 0, 4, 0, 0xa0, 0}));
}

TEST(Protocol, PingCommandLayoutAndDisable)
{
  os::Params params;
  params.start_mm = 1234;
  params.length_mm = 7000;
  params.num_results = 1200;
  auto bytes = wire::ping_command(params, false);
  EXPECT_EQ(bytes.size(), 46u);
  EXPECT_EQ(wire::u16(bytes, 4), 2197);
  EXPECT_EQ(wire::u16(bytes, 8), 1234);
  EXPECT_EQ(wire::u16(bytes, 12), 7000);
  EXPECT_EQ(wire::u16(bytes, 36), 65535);  // Signed automatic gain -1.
  EXPECT_EQ(wire::u16(bytes, 38), 1200);
  EXPECT_EQ(bytes[40], 0);
  EXPECT_EQ(wire::ping_command(params)[40], 1);
}

TEST(Protocol, SyncCommandsMatchIndependentVendorLayout)
{
  EXPECT_EQ(wire::sync_command(0, 4), (std::vector<uint8_t>{
    0x42, 0x52, 2, 0, 0xaa, 0, 0, 0, 0, 4, 0x44, 1}));
  EXPECT_EQ(wire::request(169), (std::vector<uint8_t>{
    0x42, 0x52, 2, 0, 6, 0, 0, 0, 0xa9, 0, 0x45, 1}));
  EXPECT_THROW(wire::sync_command(4, 4), std::invalid_argument);
  EXPECT_THROW(wire::sync_command(0, 0), std::invalid_argument);
}

TEST(Protocol, PingPeriodUsesFullUnsigned32BitWireRange)
{
  os::Params params;
  for (const int64_t period : {100LL, 1000LL, 4294967295LL}) {
    params.msec_per_ping = period;
    auto bytes = wire::ping_command(params);
    const auto low = wire::u16(bytes, 16);
    const auto high = wire::u16(bytes, 18);
    EXPECT_EQ(static_cast<uint64_t>(low) + (static_cast<uint64_t>(high) << 16), period);
  }
  params.msec_per_ping = 4294967296LL;
  EXPECT_THROW(params.validate(), std::invalid_argument);
  params.msec_per_ping = -1;
  EXPECT_THROW(params.validate(), std::invalid_argument);
}

TEST(Protocol, IndependentProfileFixtureAndScaling)
{
  const auto fixture = profile_fixture();
  const auto p = wire::decode_profile(fixture);
  EXPECT_EQ(p.ping_number, 0x12345678u);
  EXPECT_EQ(p.timestamp_ms, 0xfedcba98u);
  EXPECT_EQ(p.start_mm, 1234u);
  EXPECT_EQ(p.length_mm, 7000u);
  EXPECT_EQ(p.ping_hz, 450000u);
  EXPECT_EQ(p.sos_dmps, 14820);
  EXPECT_EQ(p.channel_number, 2);
  EXPECT_EQ(p.gain_index, 3);
  EXPECT_FLOAT_EQ(p.analog_gain, -1);
  EXPECT_FLOAT_EQ(p.transducer_heading_deg, 180);
  ASSERT_EQ(p.raw.size(), 200u);
  EXPECT_EQ(p.raw[0], 0);
  EXPECT_EQ(p.raw[1], 65535);
  EXPECT_FLOAT_EQ(p.scaled[0], -90);
  EXPECT_FLOAT_EQ(p.scaled[1], -10);
  EXPECT_NEAR(p.scaled[2], -49.99939, 0.00001);
  EXPECT_EQ(p.timestamp_ns, fixture.timestamp_ns);
  EXPECT_EQ(p.received, fixture.received);
  os::Params desired;
  desired.start_mm = 1234;
  desired.length_mm = 7000;
  desired.num_results = 200;
  EXPECT_TRUE(wire::matches(p, desired));
  desired.speed_of_sound_mm += 99;
  EXPECT_TRUE(wire::matches(p, desired));
  desired.speed_of_sound_mm += 1;
  EXPECT_FALSE(wire::matches(p, desired));
}

TEST(Protocol, RejectsInvalidSamplesAndNonfiniteMetadata)
{
  auto packet = profile_fixture();
  packet.payload.pop_back();
  EXPECT_THROW(wire::decode_profile(packet), os::DataError);
  packet = profile_fixture();
  packet.payload[22] = 199;
  EXPECT_THROW(wire::decode_profile(packet), os::DataError);
  for (const size_t offset : {28, 32, 36, 40, 44, 48}) {
    packet = profile_fixture();
    const auto nan = std::bit_cast<std::array<uint8_t, 4>>(std::numeric_limits<float>::quiet_NaN());
    for (size_t i = 0; i < 4; ++i) {
      packet.payload[offset + i] = nan[i];
    }
    EXPECT_THROW(wire::decode_profile(packet), os::DataError) << offset;
  }
}

TEST(Protocol, PowerScalingStaysBoundedAcrossFiniteRangesAndSampleCounts)
{
  const auto large = std::numeric_limits<float>::max();
  const auto small = std::numeric_limits<float>::denorm_min();
  const std::array<std::pair<float, float>, 8> ranges{{
    {-90, 0}, {-123.456f, -10.2f}, {5, 5}, {-large, large},
    {-large, 0}, {0, small}, {-large, small}, {-90, 0.00000001f}}};
  for (const uint16_t count : {200, 600, 1200}) {
    for (const auto & [low, high] : ranges) {
      auto packet = profile_fixture();
      packet.payload.resize(52 + 2 * count);
      packet.payload[22] = static_cast<uint8_t>(count);
      packet.payload[23] = static_cast<uint8_t>(count >> 8);
      const auto store_float = [&packet](size_t offset, float value) {
          const auto bits = std::bit_cast<uint32_t>(value);
          for (size_t i = 0; i < 4; ++i) {
            packet.payload[offset + i] = static_cast<uint8_t>(bits >> (8 * i));
          }
        };
      store_float(36, high);
      store_float(40, low);
      for (size_t i = 0; i < count; ++i) {
        const auto raw = static_cast<uint16_t>(i * 65535 / (count - 1));
        packet.payload[52 + 2 * i] = static_cast<uint8_t>(raw);
        packet.payload[53 + 2 * i] = static_cast<uint8_t>(raw >> 8);
      }
      const auto profile = wire::decode_profile(packet);
      EXPECT_EQ(profile.scaled.front(), low);
      EXPECT_EQ(profile.scaled.back(), high);
      for (size_t i = 0; i < count; ++i) {
        EXPECT_EQ(profile.raw[i], i * 65535 / (count - 1));
        EXPECT_TRUE(std::isfinite(profile.scaled[i]));
        EXPECT_GE(profile.scaled[i], low);
        EXPECT_LE(profile.scaled[i], high);
        if (i) {EXPECT_GE(profile.scaled[i], profile.scaled[i - 1]);}
      }
    }
  }
}

TEST(Protocol, RandomChunkBoundariesPreserveFramesAndOriginalReceiptTimes)
{
  struct Expected {uint16_t id; std::vector<uint8_t> payload; size_t end;};
  const std::array<std::pair<uint16_t, size_t>, 6> shapes{{
    {4, 6}, {2, 4096}, {3, 0}, {10, 1024}, {2198, 452}, {2198, 2452}}};
  std::mt19937 rng(2198);
  std::vector<Expected> expected;
  std::vector<uint8_t> stream;
  for (size_t i = 0; i < 500; ++i) {
    // Non-header noise exercises synchronization without inventing packets.
    for (size_t noise = rng() % 17; noise > 0; --noise) {
      stream.push_back(static_cast<uint8_t>(rng() % 64));
    }
    const auto [id, size] = shapes[rng() % shapes.size()];
    std::vector<uint8_t> payload(size);
    for (auto & byte : payload) {
      byte = static_cast<uint8_t>(rng());
    }
    const auto frame = wire::encode(id, payload);
    stream.insert(stream.end(), frame.begin(), frame.end());
    expected.push_back({id, std::move(payload), stream.size()});
  }
  wire::Parser parser;
  auto received = os::SteadyClock::now();
  int64_t stamp = 77000000;
  size_t delivered = 0;
  for (size_t cursor = 0; cursor < stream.size(); ) {
    const auto count = std::min<size_t>(stream.size() - cursor, 1 + rng() % 5000);
    received += std::chrono::milliseconds(1);
    stamp += 1000000;
    const auto packets = parser.feed(std::span(stream).subspan(cursor, count), received, stamp);
    for (const auto & packet : packets) {
      ASSERT_LT(delivered, expected.size());
      const auto & frame = expected[delivered++];
      EXPECT_GT(frame.end, cursor);
      EXPECT_LE(frame.end, cursor + count);
      EXPECT_EQ(packet.id, frame.id);
      EXPECT_EQ(packet.payload, frame.payload);
      EXPECT_EQ(packet.received, received);
      EXPECT_EQ(packet.timestamp_ns, stamp);
    }
    cursor += count;
    EXPECT_LE(parser.buffered_bytes(), 4105u);
    EXPECT_TRUE(parser.feed({}, received, stamp + 1).empty());
  }
  EXPECT_EQ(delivered, expected.size());
  EXPECT_EQ(parser.errors(), 0u);
  EXPECT_EQ(parser.buffered_bytes(), 0u);
  EXPECT_FALSE(parser.assembly_deadline());
}

TEST(Protocol, FragmentationCoalescingChecksumAndResynchronization)
{
  wire::Parser parser;
  const auto now = os::SteadyClock::now();
  const auto bytes = wire::encode(4, std::array<uint8_t, 6>{3, 1, 1, 2, 3, 0});
  std::vector<os::Packet> packets;
  for (size_t i = 0; i < bytes.size(); ++i) {
    auto result = parser.feed(std::span(bytes).subspan(i, 1), now, 77);
    packets.insert(packets.end(), result.begin(), result.end());
  }
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets.front().id, 4);
  EXPECT_EQ(packets.front().timestamp_ns, 77);
  auto corrupt = bytes;
  corrupt.back() ^= 1;
  corrupt.insert(corrupt.end(), bytes.begin(), bytes.end());
  corrupt.insert(corrupt.end(), bytes.begin(), bytes.end());
  EXPECT_EQ(parser.feed(corrupt, now, 78).size(), 2u);
  EXPECT_EQ(parser.errors(), 1u);
  EXPECT_EQ(parser.buffered_bytes(), 0u);
}

TEST(Protocol, RejectsOversizedHeaderAndExpiresTruncatedFrame)
{
  wire::Parser parser;
  EXPECT_FALSE(parser.assembly_deadline());
  auto now = os::SteadyClock::now();
  const std::array<uint8_t, 8> huge{'B', 'R', 0xff, 0xff, 0x96, 8, 0, 0};
  parser.feed(huge, now, 0);
  EXPECT_EQ(parser.errors(), 1u);
  EXPECT_EQ(parser.buffered_bytes(), 0u);
  // A legal NACK header with a missing payload hides a later complete packet
  // until the bounded frame assembly deadline expires.
  const std::array<uint8_t, 8> truncated{'B', 'R', 0, 4, 2, 0, 0, 0};
  parser.feed(truncated, now, 0);
  ASSERT_TRUE(parser.assembly_deadline());
  EXPECT_EQ(*parser.assembly_deadline(), now + std::chrono::seconds(1));
  const auto good = wire::encode(4, std::array<uint8_t, 6>{3, 1, 1, 2, 3, 0});
  const auto received = now;
  EXPECT_TRUE(parser.feed(good, now, 77).empty());
  EXPECT_EQ(*parser.assembly_deadline(), now + std::chrono::seconds(1));
  now += std::chrono::seconds(2);
  const auto recovered = parser.feed({}, now, 42);
  ASSERT_EQ(recovered.size(), 1u);
  EXPECT_EQ(recovered.front().id, 4);
  EXPECT_EQ(recovered.front().timestamp_ns, 77);
  EXPECT_EQ(recovered.front().received, received);
  EXPECT_EQ(parser.errors(), 2u);
  EXPECT_FALSE(parser.assembly_deadline());
}

TEST(Protocol, RandomNoiseRemainsBoundedAndCanRecover)
{
  wire::Parser parser;
  std::mt19937 rng(450);
  std::vector<uint8_t> noise(100000);
  for (auto & byte : noise) {
    byte = static_cast<uint8_t>(rng());
                                                                }
  auto now = os::SteadyClock::now();
  parser.feed(noise, now, 0);
  EXPECT_LE(parser.buffered_bytes(), 4105u);
  now += std::chrono::seconds(2);
  const auto good = wire::encode(4, std::array<uint8_t, 6>{3, 1, 1, 2, 3, 0});
  EXPECT_EQ(parser.feed(good, now, 0).size(), 1u);
}

TEST(Protocol, ParameterBoundsRejectNaNAndInvalidWireRanges)
{
  os::Params p;
  p.validate();
  p.num_results = 1201;
  EXPECT_THROW(p.validate(), std::invalid_argument);
  p = os::Params{};
  p.pulse_len_percent = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(wire::ping_command(p), std::invalid_argument);
  p = os::Params{};
  p.start_mm = -1;
  EXPECT_THROW(wire::ping_command(p), std::invalid_argument);
}
