// SPDX-License-Identifier: GPL-3.0
/**
 * @file cobs.hpp
 * @brief Consistent Overhead Byte Stuffing (COBS) encoder/decoder.
 *
 * COBS guarantees that 0x00 never appears in encoded output, making it safe
 * to use 0x00 as a packet delimiter over serial.
 *
 * Encoding overhead: at most 1 byte per 254 input bytes plus a 1-byte final
 * overhead byte, so cobs_max_encoded_size() gives the safe output buffer size.
 *
 * References:
 *   S. Cheshire and M. Baker, "Consistent Overhead Byte Stuffing", IEEE/ACM
 *   Transactions on Networking, 1999.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace mowgli_hardware
{

/**
 * @brief Return the maximum number of bytes a COBS-encoded buffer may occupy.
 *
 * @param raw_size Number of raw (pre-encoding) bytes.
 * @return Maximum encoded size, safe to allocate as output buffer.
 */
[[nodiscard]] constexpr std::size_t cobs_max_encoded_size(std::size_t raw_size) noexcept
{
  // Every 254 data bytes require 1 overhead byte; plus 1 trailing overhead.
  return raw_size + (raw_size / 254u) + 1u;
}

/**
 * @brief COBS-encode @p len bytes from @p input into @p output.
 *
 * @p output must be at least cobs_max_encoded_size(len) bytes.
 * The delimiter byte 0x00 is NOT written; the caller is responsible for
 * framing (prepend/append 0x00 as required by the protocol).
 *
 * @param input  Pointer to raw data buffer.
 * @param len    Number of bytes to encode.
 * @param output Destination buffer (must not overlap @p input).
 * @return Number of bytes written to @p output.
 */
[[nodiscard]] std::size_t cobs_encode(
  const uint8_t * input, std::size_t len, uint8_t * output) noexcept;

/**
 * @brief COBS-decode @p len bytes from @p input into @p output.
 *
 * @p input must NOT contain any 0x00 bytes (i.e. the caller has already
 * stripped the framing delimiters before calling this function).
 *
 * @param input  COBS-encoded data (no delimiters).
 * @param len    Number of encoded bytes.
 * @param output Destination buffer; must be at least @p len bytes.
 * @return Number of decoded bytes written, or 0 on encoding error.
 */
[[nodiscard]] std::size_t cobs_decode(
  const uint8_t * input, std::size_t len, uint8_t * output) noexcept;

}  // namespace mowgli_hardware
