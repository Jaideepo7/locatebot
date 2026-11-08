#!/usr/bin/env python3
"""Pulse PCA9685 channel(s):  python3 pulse.py CH[,CH...] [DUTY] [SECONDS]"""
import sys, time
from smbus2 import SMBus
from hw import PCA9685, OLED

chs = [int(c) for c in sys.argv[1].split(",")]
duty = int(sys.argv[2]) if len(sys.argv) > 2 else 2000
secs = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
bus = SMBus(1)
pwm = PCA9685(bus)
try:
    try:
        OLED(bus).text("CH " + ",".join(map(str, chs)), "pulsing...")
    except OSError:
        pass
    for ch in chs:
        pwm.set_duty(ch, duty)
    time.sleep(secs)
finally:
    pwm.all_off()
print(f"channels {chs} pulsed {secs}s at duty {duty}, all off now")
