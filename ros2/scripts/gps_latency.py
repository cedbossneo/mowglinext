#!/usr/bin/env python3
"""Measure end-to-end GPS pipeline latency.

Subscribes to:
  /ubx_nav_pvt    — has both UTC time-of-fix AND header.stamp set by driver
  /wheel_odom     — body-frame velocity from encoders (immediate, ground truth)
  /gps/fix        — what subscribers actually consume

For each NAV-PVT message:
  • UTC time of the fix         := datetime(year, month, day, hour, min, sec) + nano
    (this is when the F9P thinks the world was at the state it just measured)
  • header.stamp                := ROS time when the driver received the USB packet
  • now()                       := wall clock when our callback ran
  • Pipeline latency            := header.stamp - utc_of_fix
  • Driver→subscriber latency   := now - header.stamp

Also tracks motion onset:
  • wheel_motion_onset_t  := first time |wheel.vx| > 0.05 m/s
  • gps_motion_onset_t    := first time NAV-PVT gSpeed > 0.05 m/s
  • Detection delay       := gps_onset - wheel_onset

Writes a 30-second window of data and prints a summary. No motor commands —
just observes whatever motion happens externally (you drive manually or run
in parallel with another test).
"""
import math
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def utc_from_pvt(msg):
    """NAV-PVT exposes year/month/day/hour/min/sec + nano (ns offset). Build a
    UNIX-epoch float (seconds since 1970-01-01 UTC)."""
    if not getattr(msg, "valid_date", False) or not getattr(msg, "valid_time", False):
        return None
    try:
        dt = datetime(
            msg.year, msg.month, msg.day,
            msg.hour, msg.min, msg.sec,
            tzinfo=timezone.utc,
        )
    except ValueError:
        return None
    nano = float(getattr(msg, "nano", 0))  # int32, can be negative
    return dt.timestamp() + nano * 1e-9


GPS_LEAP_SECONDS = 18  # GPS time is 18s ahead of UTC (as of 2026)


def utc_from_itow(msg, ref_dt):
    """iTOW is GPS time-of-week in ms. Reconstruct UTC unix timestamp using a
    reference UTC datetime to identify which GPS week we're in."""
    itow_ms = getattr(msg, "itow", None)
    if itow_ms is None:
        return None
    # ref_dt is a datetime in UTC giving us approximately the right week
    # GPS week starts Sunday 00:00:00 GPS (= Saturday 23:59:42 UTC).
    days_since_sunday = (ref_dt.weekday() + 1) % 7  # Monday=0 → 1, Sunday=6 → 0
    sunday_utc = datetime(ref_dt.year, ref_dt.month, ref_dt.day,
                          tzinfo=timezone.utc)
    sunday_utc = sunday_utc.fromtimestamp(
        sunday_utc.timestamp() - days_since_sunday * 86400, tz=timezone.utc)
    # Sunday 00:00:00 UTC + (iTOW_ms - 18*1000) ms = UTC of measurement
    return sunday_utc.timestamp() + (itow_ms - GPS_LEAP_SECONDS * 1000) / 1000.0


class LatencyProbe(Node):
    def __init__(self, duration_s=30.0):
        super().__init__("gps_latency")
        self.duration = duration_s
        self.start_t = None
        self.records = []      # list of (kind, ...)
        self.wheel_onset_t = None
        self.gps_onset_t = None

        # NAV-PVT type is published by ublox_dgnss only inside the gps
        # container — we run there. Try both common message names.
        try:
            from ublox_ubx_msgs.msg import UBXNavPVT
            self.create_subscription(UBXNavPVT, "/ubx_nav_pvt", self.cb_pvt, 10)
        except ImportError:
            self.get_logger().error("ublox_ubx_msgs not available — must run inside mowgli-gps")
            raise

        self.create_subscription(Odometry, "/wheel_odom", self.cb_wheel, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self.cb_fix, 10)

    def cb_pvt(self, msg):
        now = time.time()
        if self.start_t is None:
            self.start_t = now
        utc_fix_pvt = utc_from_pvt(msg)
        utc_fix_itow = utc_from_itow(msg, datetime.now(timezone.utc))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gspeed = getattr(msg, "g_speed", None)
        if gspeed is None:
            gspeed = 0
        gspeed_mps = gspeed * 1e-3
        self.records.append((
            "pvt", now, utc_fix_pvt, utc_fix_itow, stamp, gspeed_mps,
        ))
        if abs(gspeed_mps) > 0.05 and self.gps_onset_t is None:
            self.gps_onset_t = (now, utc_fix, stamp, gspeed_mps)
        if now - self.start_t > self.duration:
            self.report()
            rclpy.shutdown()

    def cb_wheel(self, msg):
        now = time.time()
        if self.start_t is None:
            self.start_t = now
        vx = msg.twist.twist.linear.x
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.records.append(("wheel", now, stamp, vx))
        if abs(vx) > 0.05 and self.wheel_onset_t is None:
            self.wheel_onset_t = (now, stamp, vx)

    def cb_fix(self, msg):
        now = time.time()
        if self.start_t is None:
            self.start_t = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.records.append(("fix", now, stamp, msg.status.status))

    def report(self):
        print(f"\n=== GPS LATENCY REPORT — {self.duration:.0f}s window ===\n")

        pvt = [r for r in self.records if r[0] == "pvt" and r[2] is not None]
        if pvt:
            print(f"NAV-PVT samples: {len(pvt)}")
            def stats(values, label):
                v = sorted(values)
                print(f"  {label}")
                print(f"    min={v[0]:8.1f} ms  p50={v[len(v)//2]:8.1f} ms  "
                      f"p95={v[int(len(v)*0.95)]:8.1f} ms  max={v[-1]:8.1f} ms")

            # using iTOW (true GPS measurement instant)
            pipeline_itow = [(r[4] - r[3]) * 1000 for r in pvt if r[3] is not None]
            stats(pipeline_itow,
                  "Pipeline latency = stamp - utc(iTOW)  — F9P measurement → ROS publish")

            # using PVT date fields (less reliable — lags iTOW per F9P firmware)
            pipeline_pvt = [(r[4] - r[2]) * 1000 for r in pvt if r[2] is not None]
            stats(pipeline_pvt,
                  "Pipeline latency = stamp - utc(date_fields) — for reference, NOT trustworthy")

            driver_to_us = [(r[1] - r[4]) * 1000 for r in pvt]
            stats(driver_to_us, "Driver → subscriber  = now() - stamp")

            total_itow = [(r[1] - r[3]) * 1000 for r in pvt if r[3] is not None]
            stats(total_itow,
                  "Total wall-clock = now() - utc(iTOW)  — what the EKF sees as 'old'")
        else:
            print("NAV-PVT: no samples with valid time")

        # ----- Motion onset comparison -----
        if self.wheel_onset_t and self.gps_onset_t:
            wt = self.wheel_onset_t[0]
            gt = self.gps_onset_t[0]
            print(f"\nMotion onset (|v| > 0.05 m/s):")
            print(f"  wheel  detected at t_now = {wt - self.start_t:6.3f}s (vx={self.wheel_onset_t[2]:.2f})")
            print(f"  gps    detected at t_now = {gt - self.start_t:6.3f}s (gSpeed={self.gps_onset_t[3]:.2f})")
            print(f"  GPS detection lag from wheel motion: {(gt - wt) * 1000:.0f} ms")
        elif not self.wheel_onset_t:
            print("\nNo wheel motion observed in window (drive the robot during the test).")
        elif not self.gps_onset_t:
            print(f"\nWheel saw motion at t={self.wheel_onset_t[0]-self.start_t:.2f}s "
                  f"but GPS gSpeed never exceeded 0.05 m/s.")


def main():
    rclpy.init()
    node = LatencyProbe(duration_s=30.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
