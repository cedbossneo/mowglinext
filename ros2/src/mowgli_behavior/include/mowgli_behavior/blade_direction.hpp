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

#pragma once

#include <cstdint>
#include <optional>
#include <random>

namespace mowgli_behavior
{

/// One direction per BT mowing session, shared by coverage and manual mowing.
/// Selection describes a REQUEST, not measured rotation. Firmware must enforce
/// a stopped reversal: a ROS2 restart can select a different direction.
class BladeDirection
{
public:
  BladeDirection() : random_(std::random_device{}())
  {
  }
  explicit BladeDirection(uint32_t seed) : random_(seed)
  {
  }

  uint8_t forCommand(bool enabled, bool auto_reverse)
  {
    // OFF must neither select nor change direction. Repeated ONs and temporary
    // OFFs (transits, guards, recharge) retain the session's first selection.
    if (enabled && !direction_)
    {
      direction_ = auto_reverse
                       ? static_cast<uint8_t>(std::uniform_int_distribution<int>(0, 1)(random_))
                       : 0u;
    }
    return direction_.value_or(0u);
  }

  void endSession()
  {
    direction_.reset();
  }

private:
  std::mt19937 random_;
  std::optional<uint8_t> direction_;
};

}  // namespace mowgli_behavior
