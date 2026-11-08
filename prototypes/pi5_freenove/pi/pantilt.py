#!/usr/bin/env python3
"""Pan/tilt test. Pan = channel 8 (Servo 0), Tilt = channel 9 (Servo 1).
python3 pantilt.py            -> full test sweep
python3 pantilt.py PAN TILT   -> move to pulse widths (us) and hold, e.g. 1500 1500
"""
import sys, time
from smbus2 import SMBus
from hw import PCA9685, OLED

PAN, TILT = 8, 9
LO, HI = 1000, 2000          # safe range; widen later once limits are known

bus = SMBus(1)
pwm = PCA9685(bus, freq=50)
try:
    oled = OLED(bus)
except OSError:
    oled = None

def say(a, b=""):
    print(a, b, flush=True)
    if oled:
        try: oled.text(a, b)
        except OSError: pass

def ease(ch, a, b, secs=1.2, steps=30):
    for i in range(steps + 1):
        pwm.set_pulse_us(ch, a + (b - a) * i / steps)
        time.sleep(secs / steps)

if len(sys.argv) == 3:
    pwm.set_pulse_us(PAN, int(sys.argv[1])); pwm.set_pulse_us(TILT, int(sys.argv[2]))
    say("PAN/TILT", f"{sys.argv[1]} / {sys.argv[2]}")
    sys.exit(0)

try:
    say("CENTER", "pan+tilt 1500")
    pwm.set_pulse_us(PAN, 1500); pwm.set_pulse_us(TILT, 1500); time.sleep(1)
    say("PAN", "left -> right")
    ease(PAN, 1500, LO); ease(PAN, LO, HI, 2.0); ease(PAN, HI, 1500)
    say("TILT", "down -> up")
    ease(TILT, 1500, LO); ease(TILT, LO, HI, 2.0); ease(TILT, HI, 1500)
    say("CIRCLE", "pan+tilt")
    import math
    for i in range(61):
        a = 2 * math.pi * i / 60
        pwm.set_pulse_us(PAN, 1500 + 400 * math.cos(a))
        pwm.set_pulse_us(TILT, 1500 + 400 * math.sin(a))
        time.sleep(0.05)
    pwm.set_pulse_us(PAN, 1500); pwm.set_pulse_us(TILT, 1500); time.sleep(0.6)
    say("PAN/TILT", "test done")
finally:
    pwm.all_off()
