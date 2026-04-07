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

#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

// Register the plugin so pluginlib can load it by its XML-declared class name.
PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::FTCController, nav2_core::Controller)
