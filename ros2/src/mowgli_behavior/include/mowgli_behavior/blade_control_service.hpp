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

#include <memory>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"

namespace mowgli_behavior
{

/// Operator and tick callbacks MUST share the owner's default mutually-exclusive
/// group: both update the session policy, and ordering matters for blade OFF.
class BladeControlService
{
public:
  BladeControlService(rclcpp::Node& owner, const std::shared_ptr<BTContext>& context)
  {
    using Control = mowgli_interfaces::srv::MowerControl;
    auto client = owner.create_client<Control>("/hardware_bridge/mower_control");
    service_ = owner.create_service<Control>(
        "~/mower_control",
        [weak = std::weak_ptr<BTContext>(context), client](const Control::Request::SharedPtr req,
                                                           Control::Response::SharedPtr resp)
        {
          auto ctx = weak.lock();
          if (!ctx || req->mow_enabled > 1u || req->mow_direction > 1u)
            return;
          // Always latch OFF, including during a hardware-bridge outage. Do not
          // accept a new ON/direction choice if it cannot be forwarded.
          const bool ready = client->service_is_ready();
          if (req->mow_enabled && !ready)
            return;
          const auto command =
              ctx->blade_direction.forOperatorCommand(req->mow_enabled != 0u, req->mow_direction);
          if (!ready)
            return;
          auto forward = std::make_shared<Control::Request>();
          forward->mow_enabled = command.enabled;
          forward->mow_direction = command.direction;
          client->async_send_request(forward);
          // Acknowledges the accepted request, not measured blade rotation.
          // Firmware owns emergency checks and stopped reversal sequencing.
          resp->success = true;
        });
  }

private:
  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr service_;
};

}  // namespace mowgli_behavior
