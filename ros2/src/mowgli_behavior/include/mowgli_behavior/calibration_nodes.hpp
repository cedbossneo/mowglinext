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

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// RecordUndockStart — snapshot GPS position before undocking
// ---------------------------------------------------------------------------

class RecordUndockStart : public BT::SyncActionNode
{
public:
  RecordUndockStart(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock — compute heading from GPS displacement
// ---------------------------------------------------------------------------

/// After undocking (backward motion), computes heading from the difference
/// between the pre-undock and post-undock GPS positions. Since the robot
/// moved straight backward, the heading is opposite to the displacement vector.
class CalibrateHeadingFromUndock : public BT::SyncActionNode
{
public:
  CalibrateHeadingFromUndock(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override;
};

}  // namespace mowgli_behavior
