// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mowgli_hardware/timer_period.hpp"

namespace mowgli_hardware
{

TEST(TimerPeriod, NormalHardwareBridgeDefaultsKeepTheirPeriods)
{
  EXPECT_EQ(timer_period_from_rate_hz(4.0), std::chrono::milliseconds(250));
  EXPECT_EQ(timer_period_from_rate_hz(100.0), std::chrono::milliseconds(10));
  EXPECT_EQ(timer_period_from_rate_hz(2.0), std::chrono::milliseconds(500));
  EXPECT_EQ(timer_period_from_rate_hz(10.0), std::chrono::milliseconds(100));
}

TEST(TimerPeriod, RejectsZeroNegativeAndNonFiniteRates)
{
  EXPECT_THROW(timer_period_from_rate_hz(0.0), std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(-1.0), std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(timer_period_from_rate_hz(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(TimerPeriod, VeryHighFiniteRateCannotCreateZeroDuration)
{
  EXPECT_EQ(timer_period_from_rate_hz(std::numeric_limits<double>::max()).count(), 1);
}

TEST(TimerPeriod, RejectsRateWithUnrepresentablyLongPeriod)
{
  EXPECT_THROW(timer_period_from_rate_hz(minimum_timer_rate_hz() / 2.0),
               std::invalid_argument);
}

TEST(TimerPeriod, RejectsInvalidDivisorsAndTimeouts)
{
  EXPECT_THROW(require_finite_positive(0.0, "wheel_track"), std::invalid_argument);
  EXPECT_THROW(require_finite_positive(std::numeric_limits<double>::quiet_NaN(), "wheel_track"),
               std::invalid_argument);
  EXPECT_THROW(require_finite_nonnegative_timeout(-1.0, "serial_rx_timeout_s"),
               std::invalid_argument);
  EXPECT_THROW(require_finite_nonnegative_timeout(std::numeric_limits<double>::quiet_NaN(),
                                                  "serial_rx_timeout_s"),
               std::invalid_argument);
}

}  // namespace mowgli_hardware
