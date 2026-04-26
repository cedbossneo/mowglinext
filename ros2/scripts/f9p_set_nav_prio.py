#!/usr/bin/env python3
"""Send a single UBX-CFG-VALSET for CFG_RATE_NAV_PRIO=7 to layers RAM+BBR+Flash.

Used after stopping mowgli-gps to release the USB device. Modeled on
~/flash/apply_config.py.
"""
import serial
import struct
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 9600

KEY = 0x20210004      # CFG_RATE_NAV_PRIO (U1, Hz)
VALUE = 7             # Hz of priority navigation epochs
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


def wait_ack(ser, timeout=2.0):
    end = time.time() + timeout
    buf = b''
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while True:
            i = buf.find(b'\xb5\x62')
            if i < 0:
                buf = b''
                break
            if i > 0:
                buf = buf[i:]
            if len(buf) < 8:
                break
            cls = buf[2]
            mid = buf[3]
            plen = struct.unpack_from('<H', buf, 4)[0]
            if len(buf) < 6 + plen + 2:
                break
            if cls == 0x05 and plen == 2:
                ack_cls, ack_id = buf[6], buf[7]
                if ack_cls == 0x06 and ack_id == 0x8a:
                    return ('ACK' if mid == 0x01 else 'NAK')
            buf = buf[6 + plen + 2:]
    return 'TIMEOUT'


def main():
    print(f"Connecting to {PORT} @ {BAUD} baud …")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # CFG-VALSET payload: version(1) layers(1) reserved(2) key(4) value(N)
    val_bytes = struct.pack('<B', VALUE)  # U1 = 1 byte
    payload = struct.pack('<BBH', 0x01, LAYERS, 0x0000) + struct.pack('<I', KEY) + val_bytes
    msg = build_ubx(0x06, 0x8a, payload)

    print(f"Sending CFG-VALSET key=0x{KEY:08x} value={VALUE} layers=0x{LAYERS:02x} …")
    ser.write(msg)
    ack = wait_ack(ser)
    print(f"Result: {ack}")

    # Read it back to verify
    valget = struct.pack('<BBH', 0x01, 0x00, 0x0000) + struct.pack('<I', KEY)
    ser.write(build_ubx(0x06, 0x8b, valget))
    end = time.time() + 1.5
    buf = b''
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        i = buf.find(b'\xb5\x62\x06\x8b')
        if i >= 0 and len(buf) > i + 8:
            plen = struct.unpack_from('<H', buf, i + 4)[0]
            if len(buf) >= i + 6 + plen + 2:
                payload = buf[i+6:i+6+plen]
                # Skip header (4) + key (4) → value
                if len(payload) >= 9:
                    val = payload[8]
                    print(f"VALGET reports CFG_RATE_NAV_PRIO = {val}")
                break
    ser.close()
    print("Done.")
    sys.exit(0 if ack == 'ACK' else 1)


if __name__ == '__main__':
    main()
