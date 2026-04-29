// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/factors.hpp"

#include <cmath>

#include <gtsam/base/Matrix.h>

namespace fusion_graph
{

// ---------------------------------------------------------------------------
// GnssLeverArmFactor
// ---------------------------------------------------------------------------

GnssLeverArmFactor::GnssLeverArmFactor(gtsam::Key key,
                                       const gtsam::Vector2& gps_xy,
                                       const gtsam::Vector2& lever_arm_xy,
                                       const gtsam::SharedNoiseModel& model)
    : Base(model, key), gps_xy_(gps_xy), lever_arm_xy_(lever_arm_xy)
{
}

gtsam::Vector GnssLeverArmFactor::evaluateError(const gtsam::Pose2& X, OptMat H) const
{
  const double c = std::cos(X.theta());
  const double s = std::sin(X.theta());

  // Predicted antenna position in map: X.t() + R(theta) * lever
  const double lx = lever_arm_xy_.x();
  const double ly = lever_arm_xy_.y();
  const double pred_x = X.x() + c * lx - s * ly;
  const double pred_y = X.y() + s * lx + c * ly;

  gtsam::Vector2 e;
  e << (pred_x - gps_xy_.x()), (pred_y - gps_xy_.y());

  if (H)
  {
    // Pose2's local tangent ordering used by GTSAM is (dx, dy, dtheta) in
    // the body frame. The Jacobian of the predicted antenna position
    // wrt that tangent is:
    //
    //   d(pred_xy)/d(dx_body) = [ c, -s ]
    //   d(pred_xy)/d(dy_body) = [ s,  c ]
    //   d(pred_xy)/d(dtheta)  = R'(theta) * lever
    //                         = [ -s*lx - c*ly,  c*lx - s*ly ]
    H->resize(2, 3);
    (*H)(0, 0) = c;
    (*H)(0, 1) = -s;
    (*H)(0, 2) = -s * lx - c * ly;
    (*H)(1, 0) = s;
    (*H)(1, 1) = c;
    (*H)(1, 2) = c * lx - s * ly;
  }

  return e;
}

gtsam::NonlinearFactor::shared_ptr GnssLeverArmFactor::clone() const
{
  return gtsam::NonlinearFactor::shared_ptr(new GnssLeverArmFactor(*this));
}

// ---------------------------------------------------------------------------
// YawUnaryFactor
// ---------------------------------------------------------------------------

YawUnaryFactor::YawUnaryFactor(gtsam::Key key,
                               double meas_yaw,
                               const gtsam::SharedNoiseModel& model)
    : Base(model, key), meas_yaw_(meas_yaw)
{
}

gtsam::Vector YawUnaryFactor::evaluateError(const gtsam::Pose2& X, OptMat H) const
{
  gtsam::Vector1 e;
  e << WrapAngle(X.theta() - meas_yaw_);

  if (H)
  {
    H->resize(1, 3);
    (*H)(0, 0) = 0.0;
    (*H)(0, 1) = 0.0;
    (*H)(0, 2) = 1.0;
  }

  return e;
}

gtsam::NonlinearFactor::shared_ptr YawUnaryFactor::clone() const
{
  return gtsam::NonlinearFactor::shared_ptr(new YawUnaryFactor(*this));
}

}  // namespace fusion_graph
