// Copyright (c) 2016, TU Dortmund - Institute of Control Theory and Systems Engineering.
// Copyright (c) 2024, Cedric - Mowgli ROS2 port.
//
// Originally part of teb_local_planner (BSD License).
// Ported from ftc_local_planner (ROS1) to mowgli_nav2_plugins (ROS2).
// boost::circular_buffer replaced with std::deque + explicit capacity.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "mowgli_nav2_plugins/oscillation_detector.hpp"

#include <cmath>
#include <cstddef>

namespace mowgli_nav2_plugins
{

// ── Public API ────────────────────────────────────────────────────────────────

void FailureDetector::setBufferLength(int length)
{
  capacity_ = (length > 0) ? static_cast<std::size_t>(length) : 0u;
  // Trim the existing buffer if it now exceeds the new capacity.
  while (buffer_.size() > capacity_) {
    buffer_.pop_front();
  }
  if (capacity_ == 0u) {
    oscillating_ = false;
  }
}

void FailureDetector::update(
  double v, double omega,
  double v_max, double v_backwards_max, double omega_max,
  double v_eps, double omega_eps)
{
  if (capacity_ == 0u) {
    return;
  }

  VelMeasurement measurement;
  measurement.v = v;
  measurement.omega = omega;

  // Normalise linear velocity.
  if (measurement.v > 0.0 && v_max > 0.0) {
    measurement.v /= v_max;
  } else if (measurement.v < 0.0 && v_backwards_max > 0.0) {
    measurement.v /= v_backwards_max;
  }

  // Normalise angular velocity.
  if (omega_max > 0.0) {
    measurement.omega /= omega_max;
  }

  // Maintain fixed capacity (oldest entry drops off the front).
  buffer_.push_back(measurement);
  if (buffer_.size() > capacity_) {
    buffer_.pop_front();
  }

  detect(v_eps, omega_eps);
}

bool FailureDetector::isOscillating() const noexcept
{
  return oscillating_;
}

void FailureDetector::clear() noexcept
{
  buffer_.clear();
  oscillating_ = false;
}

std::size_t FailureDetector::capacity() const noexcept
{
  return capacity_;
}

// ── Protected helpers ─────────────────────────────────────────────────────────

bool FailureDetector::detect(double v_eps, double omega_eps)
{
  oscillating_ = false;

  // Wait until the buffer is at least half-full before making a decision.
  if (buffer_.size() < capacity_ / 2u || buffer_.empty()) {
    return false;
  }

  const double n = static_cast<double>(buffer_.size());

  double v_mean = 0.0;
  double omega_mean = 0.0;
  int omega_zero_crossings = 0;

  for (std::size_t i = 0; i < buffer_.size(); ++i) {
    v_mean += buffer_[i].v;
    omega_mean += buffer_[i].omega;
    if (i > 0 && sign(buffer_[i].omega) != sign(buffer_[i - 1].omega)) {
      ++omega_zero_crossings;
    }
  }

  v_mean /= n;
  omega_mean /= n;

  if (std::abs(v_mean) < v_eps &&
    std::abs(omega_mean) < omega_eps &&
    omega_zero_crossings > 1)
  {
    oscillating_ = true;
  }

  return oscillating_;
}

// ── Private helpers ───────────────────────────────────────────────────────────

int FailureDetector::sign(double x) noexcept
{
  if (x > 0.0) {return 1;}
  if (x < 0.0) {return -1;}
  return 0;
}

}  // namespace mowgli_nav2_plugins
