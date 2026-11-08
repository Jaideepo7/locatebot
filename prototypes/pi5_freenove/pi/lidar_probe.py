#!/usr/bin/env python3
"""Identify a LiDAR on /dev/ttyUSB0 by trying known protocols. Read-only probes."""
import time, serial

PORT = "/dev/ttyUSB0"


def opened(baud):
    s = serial.Serial(PORT, baud, timeout=0.3)
    s.dtr = False          # RPLIDAR A1/A2: DTR low = motor ON (harmless otherwise)
    s.reset_input_buffer()
    return s


# 1) LDROBOT LD06/LD19/STL: streams 47-byte packets starting 0x54 0x2C at 230400, no command needed
for baud in (230400,):
    s = opened(baud); time.sleep(0.8); d = s.read(600); s.close()
    if b"\x54\x2c" in d:
        print(f"MATCH LDROBOT (LD06/LD19 style) streaming @ {baud}, {len(d)} bytes in 0.8s"); raise SystemExit
    print(f"no LD06 stream @ {baud} ({len(d)} bytes, head {d[:8].hex()})")

# 2) Slamtec RPLIDAR: GET_INFO = A5 50 -> descriptor A5 5A 14 00 00 00 04 + 20 bytes
MODELS = {0x18: "A1", 0x28: "A2", 0x38: "A3", 0x41: "C1", 0x61: "S1", 0x71: "S2"}
for baud in (115200, 460800, 256000, 1000000):
    s = opened(baud)
    s.write(b"\xa5\x25"); time.sleep(0.05); s.reset_input_buffer()     # STOP first
    s.write(b"\xa5\x50"); time.sleep(0.3); d = s.read(64)
    if d.startswith(b"\xa5\x5a") and len(d) >= 27:
        p = d[7:27]
        model, fw_min, fw_maj, hw = p[0], p[1], p[2], p[3]
        serial_no = p[4:20].hex().upper()
        s.write(b"\xa5\x52"); time.sleep(0.2); h = s.read(10)
        health = {0: "Good", 1: "Warning", 2: "Error"}.get(h[7], "?") if len(h) >= 10 else "?"
        s.close()
        print(f"MATCH Slamtec RPLIDAR @ {baud}: model byte 0x{model:02x} (~{MODELS.get(model & 0xF0 | 0x01 if False else model, MODELS.get(model & 0xF8, 'unknown'))}), "
              f"firmware {fw_maj}.{fw_min:02d}, hardware {hw}, health {health}, serial {serial_no}")
        raise SystemExit
    s.close()
    print(f"no RPLIDAR reply @ {baud} (got {d[:12].hex()})")

# 3) YDLIDAR: device info = A5 90 -> A5 5A 14 00 00 00 04 + 20 bytes (model, fw, hw, serial)
for baud in (128000, 115200, 230400, 512000):
    s = opened(baud)
    s.write(b"\xa5\x65"); time.sleep(0.05); s.reset_input_buffer()     # stop scan
    s.write(b"\xa5\x90"); time.sleep(0.3); d = s.read(64); s.close()
    if d.startswith(b"\xa5\x5a") and len(d) >= 27:
        print(f"MATCH YDLIDAR @ {baud}: model {d[7]}, fw {d[9]}.{d[8]}, hw {d[10]}"); raise SystemExit
    print(f"no YDLIDAR reply @ {baud} (got {d[:12].hex()})")

print("NO MATCH - tell me the model printed on the LiDAR")
