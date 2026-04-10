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

// Umbrella header — includes all action node groups for backward compatibility.
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/calibration_nodes.hpp"
#include "mowgli_behavior/docking_nodes.hpp"
#include "mowgli_behavior/navigation_nodes.hpp"
#include "mowgli_behavior/recording_nodes.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include "mowgli_behavior/utility_nodes.hpp"

namespace mowgli_behavior
{

/// Registers all mowgli_behavior BT nodes with the given factory.
void registerAllNodes(BT::BehaviorTreeFactory& factory);

}  // namespace mowgli_behavior
