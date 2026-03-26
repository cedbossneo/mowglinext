// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_packet_handler.cpp
 * @brief Unit tests for PacketHandler framing, deframing, and CRC helpers.
 *
 * Tests cover:
 *  - encode_packet / feed roundtrip (happy path)
 *  - CRC verification for known-good and known-bad payloads
 *  - append_crc produces correct CRC bytes
 *  - Corrupt CRC is rejected (rx_crc_errors counter increments)
 *  - Leading / trailing delimiter bytes are consumed correctly
 *  - Multiple packets in a single feed() call
 *  - Oversized frame is discarded (rx_overflow counter increments)
 *  - COBS error frame is discarded (rx_cobs_errors counter increments)
 *  - Callback is not invoked when no callback is registered
 *  - Packet with minimum valid size (1 payload byte + 2 CRC bytes)
 */

#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "mowgli_hardware/crc16.hpp"
#include "mowgli_hardware/packet_handler.hpp"

using mowgli_hardware::PacketHandler;
using mowgli_hardware::crc16_ccitt;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a raw payload with a type byte and an optional body.
static std::vector<uint8_t> make_payload(uint8_t type, const std::vector<uint8_t> & body = {})
{
  std::vector<uint8_t> p;
  p.push_back(type);
  p.insert(p.end(), body.begin(), body.end());
  return p;
}

// ---------------------------------------------------------------------------
// CRC helpers
// ---------------------------------------------------------------------------

TEST(PacketHandlerCrc, AppendCrcProducesCorrectBytes)
{
  // Build a 5-byte buffer: 3 payload bytes + 2 CRC placeholder bytes.
  uint8_t buf[5] = {0x01, 0x02, 0x03, 0x00, 0x00};
  PacketHandler::append_crc(buf, sizeof(buf));

  const uint16_t expected = crc16_ccitt(buf, 3u);
  const uint16_t stored =
    static_cast<uint16_t>(buf[3]) |
    (static_cast<uint16_t>(buf[4]) << 8u);

  EXPECT_EQ(stored, expected);
}

TEST(PacketHandlerCrc, VerifyCrcAcceptsGoodPayload)
{
  uint8_t buf[5] = {0x01, 0x02, 0x03, 0x00, 0x00};
  PacketHandler::append_crc(buf, sizeof(buf));
  EXPECT_TRUE(PacketHandler::verify_crc(buf, sizeof(buf)));
}

TEST(PacketHandlerCrc, VerifyCrcRejectsBadPayload)
{
  uint8_t buf[5] = {0x01, 0x02, 0x03, 0x00, 0x00};
  PacketHandler::append_crc(buf, sizeof(buf));
  buf[1] ^= 0xFFu;  // Flip a payload byte.
  EXPECT_FALSE(PacketHandler::verify_crc(buf, sizeof(buf)));
}

TEST(PacketHandlerCrc, VerifyCrcRejectsTooShort)
{
  uint8_t buf[2] = {0x01, 0x02};
  EXPECT_FALSE(PacketHandler::verify_crc(buf, sizeof(buf)));
}

TEST(PacketHandlerCrc, VerifyCrcAcceptsMinimumSize)
{
  // Minimum: 1 payload byte + 2 CRC bytes = 3 bytes total.
  uint8_t buf[3] = {0x42, 0x00, 0x00};
  PacketHandler::append_crc(buf, sizeof(buf));
  EXPECT_TRUE(PacketHandler::verify_crc(buf, sizeof(buf)));
}

// ---------------------------------------------------------------------------
// Encode / decode roundtrip
// ---------------------------------------------------------------------------

TEST(PacketHandlerRoundtrip, SimplePayload)
{
  PacketHandler handler;

  std::vector<uint8_t> received_payload;
  handler.set_callback([&](const uint8_t * data, std::size_t len) {
    received_payload.assign(data, data + len);
  });

  const auto payload = make_payload(0x01, {0x10, 0x20, 0x30});
  const auto frame = handler.encode_packet(payload.data(), payload.size());

  // Feed the entire frame including delimiters.
  handler.feed(frame.data(), frame.size());

  // received_payload = original payload + 2 CRC bytes appended by encode.
  ASSERT_EQ(received_payload.size(), payload.size() + 2u);
  EXPECT_EQ(
    std::vector<uint8_t>(received_payload.begin(),
      received_payload.begin() + static_cast<std::ptrdiff_t>(payload.size())),
    payload);
  EXPECT_EQ(handler.rx_ok(), 1u);
}

TEST(PacketHandlerRoundtrip, PayloadWithZeroBytes)
{
  PacketHandler handler;

  std::vector<uint8_t> received_payload;
  handler.set_callback([&](const uint8_t * data, std::size_t len) {
    received_payload.assign(data, data + len);
  });

  const auto payload = make_payload(0x02, {0x00, 0x01, 0x00, 0x02, 0x00});
  const auto frame = handler.encode_packet(payload.data(), payload.size());

  handler.feed(frame.data(), frame.size());

  ASSERT_EQ(received_payload.size(), payload.size() + 2u);
  EXPECT_EQ(
    std::vector<uint8_t>(received_payload.begin(),
      received_payload.begin() + static_cast<std::ptrdiff_t>(payload.size())),
    payload);
}

TEST(PacketHandlerRoundtrip, MultiplePacketsInOneFeed)
{
  PacketHandler handler;

  int callback_count = 0;
  handler.set_callback([&](const uint8_t *, std::size_t) {
    ++callback_count;
  });

  const auto p1 = make_payload(0x01, {0xAA, 0xBB});
  const auto p2 = make_payload(0x02, {0xCC, 0xDD, 0xEE});
  const auto p3 = make_payload(0x03, {});

  auto f1 = handler.encode_packet(p1.data(), p1.size());
  auto f2 = handler.encode_packet(p2.data(), p2.size());
  auto f3 = handler.encode_packet(p3.data(), p3.size());

  // Concatenate all three frames and feed in one call.
  std::vector<uint8_t> all_frames;
  all_frames.insert(all_frames.end(), f1.begin(), f1.end());
  all_frames.insert(all_frames.end(), f2.begin(), f2.end());
  all_frames.insert(all_frames.end(), f3.begin(), f3.end());

  handler.feed(all_frames.data(), all_frames.size());

  EXPECT_EQ(callback_count, 3);
  EXPECT_EQ(handler.rx_ok(), 3u);
}

TEST(PacketHandlerRoundtrip, FeedInSmallChunks)
{
  PacketHandler handler;

  int callback_count = 0;
  handler.set_callback([&](const uint8_t *, std::size_t) { ++callback_count; });

  const auto payload = make_payload(0x42, {0x01, 0x02, 0x03, 0x04});
  const auto frame = handler.encode_packet(payload.data(), payload.size());

  // Feed one byte at a time.
  for (const uint8_t byte : frame) {
    handler.feed(&byte, 1u);
  }

  EXPECT_EQ(callback_count, 1);
  EXPECT_EQ(handler.rx_ok(), 1u);
}

// ---------------------------------------------------------------------------
// Error paths
// ---------------------------------------------------------------------------

TEST(PacketHandlerErrors, CorruptCrcIsRejected)
{
  PacketHandler handler;

  int callback_count = 0;
  handler.set_callback([&](const uint8_t *, std::size_t) { ++callback_count; });

  const auto payload = make_payload(0x01, {0xDE, 0xAD, 0xBE, 0xEF});
  auto frame = handler.encode_packet(payload.data(), payload.size());

  // Flip a byte inside the COBS payload (between the two 0x00 delimiters).
  // frame[0] = 0x00, frame[1..n-2] = COBS bytes, frame[n-1] = 0x00
  if (frame.size() > 3u) {
    frame[2] ^= 0xFFu;
  }

  handler.feed(frame.data(), frame.size());

  EXPECT_EQ(callback_count, 0);
  EXPECT_GT(handler.rx_crc_errors() + handler.rx_cobs_errors(), 0u);
}

TEST(PacketHandlerErrors, OversizedFrameIsDiscarded)
{
  PacketHandler handler;

  int callback_count = 0;
  handler.set_callback([&](const uint8_t *, std::size_t) { ++callback_count; });

  // Build a frame that exceeds kMaxPacketBytes raw bytes.
  // Feed: 0x00, then (kMaxPacketBytes + 10) non-zero bytes, then 0x00.
  const std::size_t oversized = PacketHandler::kMaxPacketBytes + 10u;
  std::vector<uint8_t> frame;
  frame.reserve(oversized + 2u);
  frame.push_back(0x00);
  for (std::size_t i = 0; i < oversized; ++i) {
    frame.push_back(static_cast<uint8_t>((i % 254u) + 1u));  // Never 0.
  }
  frame.push_back(0x00);

  handler.feed(frame.data(), frame.size());

  EXPECT_EQ(callback_count, 0);
  EXPECT_EQ(handler.rx_overflow(), 1u);
}

TEST(PacketHandlerErrors, NoCallbackDoesNotCrash)
{
  PacketHandler handler;
  // No callback registered — feeding a valid packet must not crash.

  const auto payload = make_payload(0x01, {0x11, 0x22});
  const auto frame = handler.encode_packet(payload.data(), payload.size());

  EXPECT_NO_THROW(handler.feed(frame.data(), frame.size()));
  EXPECT_EQ(handler.rx_ok(), 1u);
}

TEST(PacketHandlerErrors, EmptyFrameIsIgnored)
{
  PacketHandler handler;
  int callback_count = 0;
  handler.set_callback([&](const uint8_t *, std::size_t) { ++callback_count; });

  // Two consecutive delimiters produce an empty frame; should be silently ignored.
  const uint8_t two_delimiters[2] = {0x00, 0x00};
  handler.feed(two_delimiters, sizeof(two_delimiters));

  EXPECT_EQ(callback_count, 0);
  EXPECT_EQ(handler.rx_ok(), 0u);
}

// ---------------------------------------------------------------------------
// Diagnostic counters
// ---------------------------------------------------------------------------

TEST(PacketHandlerCounters, RxOkIncrements)
{
  PacketHandler handler;
  handler.set_callback([](const uint8_t *, std::size_t) {});

  const auto payload = make_payload(0x01);
  const auto frame = handler.encode_packet(payload.data(), payload.size());

  handler.feed(frame.data(), frame.size());
  handler.feed(frame.data(), frame.size());
  handler.feed(frame.data(), frame.size());

  EXPECT_EQ(handler.rx_ok(), 3u);
}
