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
#include "mowgli_behavior/coverage_persistence.hpp"
#include "mowgli_interfaces/srv/coverage_orientation.hpp"

namespace mowgli_behavior
{
// Same default mutually-exclusive callback group as BT ticks: the operator's
// next choice and EndSession must be serialized with plan/phase updates.
class CoverageOrientationService
{
public:
  CoverageOrientationService(rclcpp::Node& owner, const std::shared_ptr<BTContext>& context)
  {
    using Service = mowgli_interfaces::srv::CoverageOrientation;
    service_ = owner.create_service<Service>(
        "~/coverage_orientation",
        [weak = std::weak_ptr<BTContext>(context)](const Service::Request::SharedPtr req,
                                                   Service::Response::SharedPtr resp)
        {
          auto ctx = weak.lock();
          if (!ctx)
            return;
          if (req->set_next)
          {
            const auto previous = ctx->cross_hatch;
            ctx->cross_hatch[req->area_index].next_override = req->perpendicular;
            if (!saveCoverageResumeState(*ctx))
            {
              ctx->cross_hatch = previous;
              resp->message = "Could not persist the next coverage orientation";
              return;
            }
          }
          const auto it = ctx->cross_hatch.find(req->area_index);
          const auto state = it == ctx->cross_hatch.end() ? CrossHatch{} : it->second;
          resp->success = true;
          resp->enabled = ctx->mow_cross_hatch;
          ctx->node->get_parameter_or("mow_angle_deg", resp->base_angle_deg, -1.0);
          resp->current_active = state.session_perpendicular.has_value();
          resp->current_perpendicular = state.session_perpendicular.value_or(false);
          resp->next_perpendicular = state.next();
        });
  }

private:
  rclcpp::Service<mowgli_interfaces::srv::CoverageOrientation>::SharedPtr service_;
};
}  // namespace mowgli_behavior
