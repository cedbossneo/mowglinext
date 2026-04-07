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
[[nodiscard]] uint16_t crc16_ccitt(const uint8_t* data, std::size_t len) noexcept;

}  // namespace mowgli_hardware
