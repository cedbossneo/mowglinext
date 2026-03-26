// Copyright (c) 2016, TU Dortmund - Institute of Control Theory and Systems Engineering.
// Copyright (c) 2024, Cedric - Mowgli ROS2 port.
//
// Originally part of teb_local_planner (BSD License).
// Ported from ftc_local_planner (ROS1) to mowgli_nav2_plugins (ROS2).
// Replaces boost::circular_buffer with std::deque.
//
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MOWGLI_NAV2_PLUGINS__OSCILLATION_DETECTOR_HPP_
#define MOWGLI_NAV2_PLUGINS__OSCILLATION_DETECTOR_HPP_

#include <deque>
#include <cmath>
#include <cstddef>

namespace mowgli_nav2_plugins
{

/**
 * @class FailureDetector
 * @brief Detects if the robot is oscillating by analysing a rolling window of
 *        normalised velocity measurements.
 *
 * The detector keeps the last N (v, omega) pairs (set via setBufferLength()).
 * After the buffer is at least half full it computes:
 *   - mean normalised linear velocity
 *   - mean normalised angular velocity
 *   - number of angular-velocity zero-crossings
 *
 * If |v_mean| < v_eps AND |omega_mean| < omega_eps AND zero_crossings > 1
 * the robot is considered oscillating.
 */
class FailureDetector
{
public:
  FailureDetector() = default;
  ~FailureDetector() = default;

  /**
   * @brief Set the rolling-window length (number of measurements retained).
   * @param length  Number of samples. A value <= 0 disables detection.
   */
  void setBufferLength(int length);

  /**
   * @brief Push a new velocity measurement and immediately recompute state.
   *
   * @param v             Commanded linear velocity in x (robot frame).
   * @param omega         Commanded angular velocity around z.
   * @param v_max         Maximum forward translational velocity (normalisation).
   * @param v_backwards_max  Maximum backward translational velocity.
   * @param omega_max     Maximum angular velocity (normalisation).
   * @param v_eps         Threshold for mean normalised linear velocity in (0,1).
   * @param omega_eps     Threshold for mean normalised angular velocity in (0,1).
   */
  void update(
    double v, double omega,
    double v_max, double v_backwards_max, double omega_max,
    double v_eps, double omega_eps);

  /**
   * @brief Return the last computed oscillation decision.
   *
   * The decision is recomputed inside every update() call; this accessor is
   * non-computing and therefore cheap.
   */
  [[nodiscard]] bool isOscillating() const noexcept;

  /**
   * @brief Clear the buffer and reset the oscillating flag.
   */
  void clear() noexcept;

  /**
   * @brief Return the configured capacity (buffer length).
   */
  [[nodiscard]] std::size_t capacity() const noexcept;

protected:
  /// Single normalised velocity sample stored in the ring buffer.
  struct VelMeasurement
  {
    double v = 0.0;
    double omega = 0.0;
  };

  /**
   * @brief Run the oscillation algorithm over the current buffer contents.
   * @return true if oscillation is detected.
   */
  bool detect(double v_eps, double omega_eps);

private:
  static int sign(double x) noexcept;

  std::deque<VelMeasurement> buffer_;  ///< Rolling window of measurements.
  std::size_t capacity_{0};            ///< Maximum window size.
  bool oscillating_{false};            ///< Current oscillation state.
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__OSCILLATION_DETECTOR_HPP_
