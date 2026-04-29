// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Custom GTSAM factors for the Mowgli factor-graph localizer.
//
// All factors are SE(2) (Pose2). Roll/pitch are clamped to zero — the
// terrain is flat to <5° and there is no 3D reasoning anywhere else in
// the stack. Choosing Pose2 over Pose3 buys ~4x in iSAM2 update cost.

#pragma once

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace fusion_graph
{

// GTSAM 4.3+ uses a raw pointer type alias `gtsam::OptionalMatrixType`
// (= Matrix*) for the optional Jacobian argument of custom factors.
// We build GTSAM 4.3a1 from source in the Dockerfile (the apt version
// on Ubuntu Noble is 4.2 with a broken CMake config).
using OptMat = gtsam::OptionalMatrixType;

// ---------------------------------------------------------------------------
// GnssLeverArmFactor
// ---------------------------------------------------------------------------
//
// Unary factor on Pose2 that measures the GPS antenna position in the map
// frame, accounting for the lever arm from base_footprint to the antenna.
//
// Error: e = gps_meas - (X.t() + R(X.theta()) * lever_arm_xy)
//
// The yaw is a graph variable (optimized jointly), so the lever-arm
// rotation is part of the residual — that's the whole reason a custom
// factor exists rather than just feeding "pre-corrected" GPS poses.
//
// Jacobian is analytic (computed in evaluateError when H is non-null).
// Autodiff would work but doubles the per-factor cost on ARM.
class GnssLeverArmFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2>
{
public:
  using This = GnssLeverArmFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose2>;

  GnssLeverArmFactor(gtsam::Key key,
                     const gtsam::Vector2& gps_xy,
                     const gtsam::Vector2& lever_arm_xy,
                     const gtsam::SharedNoiseModel& model);

  // GTSAM API
  gtsam::Vector evaluateError(const gtsam::Pose2& X, OptMat H = nullptr) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  gtsam::Vector2 gps_xy_;
  gtsam::Vector2 lever_arm_xy_;
};

// ---------------------------------------------------------------------------
// YawUnaryFactor
// ---------------------------------------------------------------------------
//
// Unary factor on Pose2 that measures only the yaw component. Used for
// COG-derived heading and tilt-compensated magnetometer yaw.
//
// Error: e = wrap(meas_yaw - X.theta())
//
// The wrap is essential: a 359° measurement vs a 1° estimate is +2° of
// real error, not -358°.
class YawUnaryFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2>
{
public:
  using This = YawUnaryFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose2>;

  YawUnaryFactor(gtsam::Key key, double meas_yaw, const gtsam::SharedNoiseModel& model);

  gtsam::Vector evaluateError(const gtsam::Pose2& X, OptMat H = nullptr) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

private:
  double meas_yaw_;
};

// ---------------------------------------------------------------------------
// Helper: yaw wrap to (-pi, pi]
// ---------------------------------------------------------------------------
inline double WrapAngle(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a <= -M_PI)
    a += 2.0 * M_PI;
  return a;
}

}  // namespace fusion_graph
