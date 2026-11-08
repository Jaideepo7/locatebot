#!/usr/bin/env python3
"""Gentle servo sweep:  python3 servo.py CH   -> center, left, right, center, then release."""
import sys, time
from smbus2 import SMBus
from hw import PCA9685, OLED

ch = int(sys.argv[1])
bus = SMBus(1)
pwm = PCA9685(bus, freq=50)
try:
    try:
        OLED(bus).text(f"SERVO {ch}", "sweeping...")
    except OSError:
        pass
    for us in (1500, 1100, 1900, 1500):
        pwm.set_pulse_us(ch, us)
        time.sleep(0.7)
finally:
    pwm.all_off()
print(f"servo channel {ch}: swept 1500->1100->1900->1500 us, released")
