#!/usr/bin/env python3
"""Send UBX-CFG-VALSET CFG_RATE_NAV_PRIO=7 (RAM+BBR+Flash) directly over libusb.

Bypasses the kernel CDC driver. Stop mowgli-gps before running, restart
after. Endpoints (per ublox_dgnss source): 0x01 OUT (host→device), 0x82 IN.
"""
import struct
import sys
import time

import usb.core
import usb.util

VID, PID = 0x1546, 0x01a9
EP_OUT, EP_IN = 0x01, 0x82
INTERFACE = 1  # CDC Data interface

KEY = 0x20210004      # CFG_RATE_NAV_PRIO
VALUE = 7
LAYERS = 0x07         # RAM | BBR | Flash


def ubx_checksum(cls, mid, payload):
    a = b = 0
    data = bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def build_ubx(cls, mid, payload):
    a, b = ubx_checksum(cls, mid, payload)
    return b'\xb5\x62' + bytes([cls, mid]) + struct.pack('<H', len(payload)) + payload + bytes([a, b])


def main():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("F9P not found"); sys.exit(1)
    print(f"Found F9P bus={dev.bus} addr={dev.address}")

    # Reset to release any stale state
    try:
        dev.reset()
        time.sleep(1.5)
    except Exception as e:
        print(f"reset: {e}")
    dev = usb.core.find(idVendor=VID, idProduct=PID)

    # Detach all interfaces
    for i in range(2):
        try:
            if dev.is_kernel_driver_active(i):
                dev.detach_kernel_driver(i)
                print(f"detached kernel driver from interface {i}")
        except usb.core.USBError as e:
            print(f"detach interface {i}: {e}")

    usb.util.claim_interface(dev, INTERFACE)
    print(f"claimed interface {INTERFACE}")

    # Drain whatever's sitting on EP_IN
    try:
        while True:
            dev.read(EP_IN, 64, timeout=200)
    except usb.core.USBError:
        pass

    # Build VALSET
    val = struct.pack('<B', VALUE)
    payload = struct.pack('<BBH', 0x01, LAYERS, 0x0000) + struct.pack('<I', KEY) + val
    msg = build_ubx(0x06, 0x8a, payload)

    print(f"Sending CFG-VALSET key=0x{KEY:08x} value={VALUE} layers=0x{LAYERS:02x}")
    dev.write(EP_OUT, msg, timeout=500)

    # Read response, look for ACK/NAK
    deadline = time.time() + 2.0
    found = None
    buf = b''
    while time.time() < deadline:
        try:
            chunk = bytes(dev.read(EP_IN, 256, timeout=300))
        except usb.core.USBError:
            continue
        buf += chunk
        i = 0
        while i < len(buf) - 1:
            if buf[i] == 0xb5 and buf[i+1] == 0x62 and len(buf) >= i + 8:
                cls = buf[i+2]; mid = buf[i+3]
                plen = struct.unpack_from('<H', buf, i+4)[0]
                if len(buf) >= i + 6 + plen + 2:
                    if cls == 0x05 and plen == 2:
                        ack_cls = buf[i+6]; ack_id = buf[i+7]
                        if ack_cls == 0x06 and ack_id == 0x8a:
                            found = 'ACK' if mid == 0x01 else 'NAK'
                            break
                    i = i + 6 + plen + 2
                    continue
            i += 1
        if found:
            break

    print(f"Result: {found or 'TIMEOUT'}")

    usb.util.release_interface(dev, INTERFACE)
    try:
        dev.attach_kernel_driver(INTERFACE)
    except Exception:
        pass

    sys.exit(0 if found == 'ACK' else 1)


if __name__ == '__main__':
    main()
