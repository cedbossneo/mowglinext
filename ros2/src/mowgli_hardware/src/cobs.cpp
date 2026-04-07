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
#include "mowgli_hardware/cobs.hpp"

#include <cassert>

namespace mowgli_hardware
{

std::size_t cobs_encode(const uint8_t* input, std::size_t len, uint8_t* output) noexcept
{
  assert(input != nullptr || len == 0);
  assert(output != nullptr);

  // Index of the current overhead byte (code byte) in the output.
  std::size_t code_idx = 0;
  // Next output write position.
  std::size_t out_idx = 1;
  // Current run length: counts bytes since the last 0x00 (or start).
  uint8_t code = 1;

  for (std::size_t i = 0; i < len; ++i)
  {
    if (input[i] == 0x00)
    {
      // Flush current run: write the code byte, start a new run.
      output[code_idx] = code;
      code_idx = out_idx++;
      code = 1;
    }
    else
    {
      output[out_idx++] = input[i];
      ++code;
      if (code == 0xFF)
      {
        // Run of 254 non-zero bytes: write the max-run code and restart.
        output[code_idx] = code;
        code_idx = out_idx++;
        code = 1;
      }
    }
  }

  // Write final code byte.
  output[code_idx] = code;
  return out_idx;
}

std::size_t cobs_decode(const uint8_t* input, std::size_t len, uint8_t* output) noexcept
{
  assert(input != nullptr || len == 0);
  assert(output != nullptr);

  if (len == 0)
  {
    return 0;
  }

  std::size_t out_idx = 0;
  std::size_t in_idx = 0;

  while (in_idx < len)
  {
    const uint8_t code = input[in_idx];
    if (code == 0x00)
    {
      // 0x00 must not appear in a COBS-encoded stream; signal error.
      return 0;
    }

    // Bounds check: the run claims code-1 more bytes exist.
    if (in_idx + code > len)
    {
      return 0;
    }

    ++in_idx;

    // Copy (code - 1) literal bytes.
    for (uint8_t k = 1; k < code; ++k)
    {
      output[out_idx++] = input[in_idx++];
    }

    // After every run that was not the maximum length (0xFF), and if we have
    // not yet consumed all input, output a 0x00 to restore the original byte.
    if (code < 0xFF && in_idx < len)
    {
      output[out_idx++] = 0x00;
    }
  }

  return out_idx;
}

}  // namespace mowgli_hardware
