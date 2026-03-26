// Copyright (c) 2024, Cedric - Mowgli ROS2 port.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <pluginlib/class_list_macros.hpp>

#include "mowgli_nav2_plugins/ftc_controller.hpp"

// Register the plugin so pluginlib can load it by its XML-declared class name.
PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::FTCController, nav2_core::Controller)
