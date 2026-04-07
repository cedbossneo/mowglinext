// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_cobs.cpp
 * @brief Unit tests for the COBS encoder / decoder.
 *
 * Tests cover:
 *  - Empty payload
 *  - Single-byte payloads (zero and non-zero)
 *  - Payloads with no zero bytes
 *  - Payloads consisting entirely of zero bytes
 *  - Mixed payloads with zeros at various positions
 *  - Payloads that cross the 254-byte run-length boundary
 *  - Payloads larger than 254 bytes (multiple overhead bytes)
 *  - Roundtrip property: decode(encode(x)) == x
 *  - cobs_max_encoded_size() upper-bound guarantee
 *  - Decoder rejects streams containing 0x00
 */

#include <algorithm>
#include <cstring>
#include <numeric>
#include <vector>

#include "mowgli_hardware/cobs.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::cobs_decode;
using mowgli_hardware::cobs_encode;
using mowgli_hardware::cobs_max_encoded_size;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::vector<uint8_t> roundtrip(const std::vector<uint8_t>& input)
{
  const std::size_t max_enc = cobs_max_encoded_size(input.size());
  std::vector<uint8_t> encoded(max_enc);
  const std::size_t enc_len = cobs_encode(input.data(), input.size(), encoded.data());
  encoded.resize(enc_len);

  // The encoded stream must never contain 0x00.
  for (std::size_t i = 0; i < enc_len; ++i)
  {
    EXPECT_NE(encoded[i], 0x00u) << "0x00 found in encoded output at index " << i;
  }

  // Encoded length must fit within the declared maximum.
  EXPECT_LE(enc_len, max_enc);

  std::vector<uint8_t> decoded(enc_len);
  const std::size_t dec_len = cobs_decode(encoded.data(), enc_len, decoded.data());
  decoded.resize(dec_len);

  return decoded;
}

// ---------------------------------------------------------------------------
// Test cases
// ---------------------------------------------------------------------------

TEST(CobsTest, EmptyPayload)
{
  // An empty input should encode to a single overhead byte (0x01) and decode
  // back to an empty buffer.
  std::vector<uint8_t> encoded(cobs_max_encoded_size(0));
  const std::size_t enc_len = cobs_encode(nullptr, 0, encoded.data());
  EXPECT_EQ(enc_len, 1u);
  EXPECT_EQ(encoded[0], 0x01u);  // Code byte for a run of zero data bytes.

  std::vector<uint8_t> decoded(1);
  const std::size_t dec_len = cobs_decode(encoded.data(), enc_len, decoded.data());
  EXPECT_EQ(dec_len, 0u);
}

TEST(CobsTest, SingleNonZeroByte)
{
  const std::vector<uint8_t> input = {0x42};
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, SingleZeroByte)
{
  const std::vector<uint8_t> input = {0x00};
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, AllZeros_FourBytes)
{
  const std::vector<uint8_t> input(4, 0x00);
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, NoZeroBytes_ShortPayload)
{
  const std::vector<uint8_t> input = {0x01, 0x02, 0x03, 0x04, 0x05};
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, ZeroAtStart)
{
  const std::vector<uint8_t> input = {0x00, 0x01, 0x02, 0x03};
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, ZeroAtEnd)
{
  const std::vector<uint8_t> input = {0x01, 0x02, 0x03, 0x00};
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, ZeroInMiddle)
{
  const std::vector<uint8_t> input = {0x01, 0x00, 0x02, 0x00, 0x03};
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, ConsecutiveZeros)
{
  const std::vector<uint8_t> input = {0x01, 0x00, 0x00, 0x00, 0x02};
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, AllNonZero_253Bytes)
{
  // 253 non-zero bytes: fits in a single code-byte run (code = 254).
  std::vector<uint8_t> input(253);
  std::iota(input.begin(), input.end(), 1u);  // 1 … 253
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, AllNonZero_254Bytes_MaxRun)
{
  // Exactly 254 non-zero bytes: the encoder uses code = 0xFF and starts a new
  // run but does NOT inject a 0x00 for the max-run code byte.
  std::vector<uint8_t> input(254);
  std::iota(input.begin(), input.end(), 1u);
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, AllNonZero_255Bytes_SplitRun)
{
  // 255 bytes forces a run split: first run 254 bytes, second run 1 byte.
  std::vector<uint8_t> input(255);
  std::iota(input.begin(), input.end(), 1u);
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, LargePayload_512Bytes)
{
  std::vector<uint8_t> input(512);
  for (std::size_t i = 0; i < input.size(); ++i)
  {
    // Alternating 0x00 and non-zero to stress the encoder.
    input[i] = (i % 3u == 0u) ? 0x00u : static_cast<uint8_t>(i & 0xFFu);
  }
  EXPECT_EQ(roundtrip(input), input);
}

TEST(CobsTest, KnownVector)
{
  // Verified against the reference implementation in the COBS paper.
  // Input: {0x00}
  // Expected encoded: {0x01, 0x01}
  const std::vector<uint8_t> input = {0x00};
  std::vector<uint8_t> encoded(cobs_max_encoded_size(input.size()));
  const std::size_t enc_len = cobs_encode(input.data(), input.size(), encoded.data());
  ASSERT_EQ(enc_len, 2u);
  EXPECT_EQ(encoded[0], 0x01u);
  EXPECT_EQ(encoded[1], 0x01u);
}

TEST(CobsTest, KnownVector_NonZeros)
{
  // Input: {0x11, 0x22, 0x33}
  // Expected: {0x04, 0x11, 0x22, 0x33}
  const std::vector<uint8_t> input = {0x11, 0x22, 0x33};
  std::vector<uint8_t> encoded(cobs_max_encoded_size(input.size()));
  const std::size_t enc_len = cobs_encode(input.data(), input.size(), encoded.data());
  ASSERT_EQ(enc_len, 4u);
  EXPECT_EQ(encoded[0], 0x04u);
  EXPECT_EQ(encoded[1], 0x11u);
  EXPECT_EQ(encoded[2], 0x22u);
  EXPECT_EQ(encoded[3], 0x33u);
}

TEST(CobsTest, DecodeRejectsZeroCodeByte)
{
  // A 0x00 at a code-byte position is invalid; decoder must return 0.
  // Position 0 is a code byte: a zero there is invalid.
  const std::vector<uint8_t> bad_input = {0x00, 0x01, 0x01};
  std::vector<uint8_t> out(bad_input.size());
  const std::size_t result = cobs_decode(bad_input.data(), bad_input.size(), out.data());
  EXPECT_EQ(result, 0u);
}

TEST(CobsTest, DecodeRejectsOverrun)
{
  // Code byte claims more bytes than remain in the buffer.
  const std::vector<uint8_t> bad_input = {0x05, 0x01, 0x02};  // Claims 4 data bytes.
  std::vector<uint8_t> out(bad_input.size());
  const std::size_t result = cobs_decode(bad_input.data(), bad_input.size(), out.data());
  EXPECT_EQ(result, 0u);
}

TEST(CobsTest, MaxEncodedSizeIsRespected)
{
  // For various lengths, the actual encoded size must never exceed the declared max.
  for (std::size_t n = 0; n <= 512u; ++n)
  {
    std::vector<uint8_t> input(n, static_cast<uint8_t>(n & 0xFFu));
    const std::size_t max_sz = cobs_max_encoded_size(n);
    std::vector<uint8_t> encoded(max_sz);
    const std::size_t actual = cobs_encode(input.data(), n, encoded.data());
    EXPECT_LE(actual, max_sz) << "For n=" << n;
  }
}
