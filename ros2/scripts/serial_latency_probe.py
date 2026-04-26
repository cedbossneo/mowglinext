#!/usr/bin/env python3
"""Read UBX NAV-PVT directly from /dev/ttyACM0 (kernel CDC ACM, no libusb)
and measure stamp - utc(iTOW) latency.

This bypasses the aussierobots libusb driver entirely — instead of going
through the libusb URB pipeline, we let the kernel CDC ACM driver deliver
each UBX byte as it arrives (the path the "official" ROS drivers use).

If latency drops below the 3.5 s we measured with libusb, the bug is in
the aussierobots driver. If latency stays at 3.5 s, it's the F9P firmware.

Stop mowgli-gps before running. Restart after.
"""
import struct
import sys
import time
from datetime import datetime, timezone

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 9600
DURATION_S = 25

UBX_NAV_PVT = (0x01, 0x07)
LEAP = 18
SECONDS_PER_WEEK = 604800


def parse_ubx_nav_pvt(payload):
    """Return (itow_ms, year, month, day, hour, minute, sec, nano) or None."""
    if len(payload) < 28:
        return None
    itow_ms, year, month, day, hour, minute, sec = struct.unpack_from(
        "<IHBBBBB", payload, 0)
    # Skip valid (1) + t_acc (4)
    nano = struct.unpack_from("<i", payload, 16)[0]
    return itow_ms, year, month, day, hour, minute, sec, nano


def itow_to_unix(itow_ms, ref_unix_s):
    now_gps_tow_s = (ref_unix_s + LEAP) % SECONDS_PER_WEEK
    delta_s = itow_ms / 1000.0 - now_gps_tow_s
    if delta_s > SECONDS_PER_WEEK / 2:
        delta_s -= SECONDS_PER_WEEK
    elif delta_s < -SECONDS_PER_WEEK / 2:
        delta_s += SECONDS_PER_WEEK
    return ref_unix_s + delta_s


def ubx_checksum(cls, mid, payload):
    a = b = 0
    data = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def main():
    print(f"Opening {PORT} @ {BAUD} baud …")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.2)
    ser.reset_input_buffer()

    buf = bytearray()
    end = time.time() + DURATION_S
    lags = []
    pvt_count = 0

    while time.time() < end:
        chunk = ser.read(2048)
        recv_t = time.time()
        if not chunk:
            continue
        buf.extend(chunk)

        # Scan for UBX frames
        while True:
            i = buf.find(b"\xb5\x62")
            if i < 0:
                # No preamble in buf — drop any garbage
                if len(buf) > 4096:
                    buf = buf[-4096:]
                break
            if len(buf) < i + 6:
                # Not enough for header
                if i > 0:
                    del buf[:i]
                break
            cls = buf[i + 2]
            mid = buf[i + 3]
            plen = struct.unpack_from("<H", buf, i + 4)[0]
            if len(buf) < i + 6 + plen + 2:
                # Not enough for payload + checksum
                if i > 0:
                    del buf[:i]
                break

            # Verify checksum
            payload = bytes(buf[i + 6:i + 6 + plen])
            a_exp, b_exp = ubx_checksum(cls, mid, payload)
            a_got = buf[i + 6 + plen]
            b_got = buf[i + 6 + plen + 1]

            if a_exp == a_got and b_exp == b_got:
                if (cls, mid) == UBX_NAV_PVT:
                    parsed = parse_ubx_nav_pvt(payload)
                    if parsed:
                        itow_ms = parsed[0]
                        utc_meas = itow_to_unix(itow_ms, int(recv_t))
                        lag_ms = (recv_t - utc_meas) * 1000
                        lags.append(lag_ms)
                        pvt_count += 1
                        if pvt_count % 10 == 0:
                            print(f"  [{pvt_count}] itow={itow_ms} lag={lag_ms:.0f}ms")
                # remove processed frame
                del buf[:i + 6 + plen + 2]
            else:
                # bad checksum, advance past preamble
                del buf[:i + 2]

    ser.close()

    print()
    if lags:
        lags_sorted = sorted(lags)
        print(f"=== /dev/ttyACM0 NAV-PVT latency (n={len(lags)}) ===")
        print(f"  min  = {lags_sorted[0]:8.1f} ms")
        print(f"  p50  = {lags_sorted[len(lags) // 2]:8.1f} ms")
        print(f"  p95  = {lags_sorted[int(len(lags) * 0.95)]:8.1f} ms")
        print(f"  max  = {lags_sorted[-1]:8.1f} ms")
        print(f"  mean = {sum(lags) / len(lags):8.1f} ms")
    else:
        print("No NAV-PVT received. Check the port and that mowgli-gps is stopped.")


if __name__ == "__main__":
    main()
