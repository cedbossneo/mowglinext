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
#include "mowgli_hardware/crc16.hpp"

namespace mowgli_hardware
{

uint16_t crc16_ccitt(const uint8_t* data, std::size_t len) noexcept
{
  // CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refIn=false, refOut=false.
  // This matches the crcCalc() implementation in the OpenMower STM32 firmware.
  uint16_t crc = 0xFFFFu;

  for (std::size_t i = 0; i < len; ++i)
  {
    crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8u);
    for (int bit = 0; bit < 8; ++bit)
    {
      if ((crc & 0x8000u) != 0u)
      {
        crc = static_cast<uint16_t>((crc << 1u) ^ 0x1021u);
      }
      else
      {
        crc = static_cast<uint16_t>(crc << 1u);
      }
    }
  }

  return crc;
}

}  // namespace mowgli_hardware
