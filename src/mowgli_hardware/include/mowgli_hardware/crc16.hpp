// SPDX-License-Identifier: GPL-3.0
/**
 * @file crc16.hpp
 * @brief CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF, no reflection)
 *        checksum used by the STM32 firmware packet protocol.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace mowgli_hardware
{

/**
 * @brief Compute CRC-16 CCITT-FALSE over @p len bytes of @p data.
 *
 * Compatible with the crcCalc() implementation in the OpenMower STM32
 * firmware (poly 0x1021, init 0xFFFF, input/output NOT reflected).
 *
 * @param data Pointer to the byte buffer to checksum.
 * @param len  Number of bytes to include in the CRC.
 * @return Computed 16-bit CRC value.
 */
[[nodiscard]] uint16_t crc16_ccitt(const uint8_t * data, std::size_t len) noexcept;

}  // namespace mowgli_hardware
