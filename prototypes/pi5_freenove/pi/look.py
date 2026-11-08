#!/usr/bin/env python3
"""look.py - point the camera, take a photo, read sensors. Prints JSON.
   python3 look.py [pan_us=1500] [tilt_us=1500] [out=~/car/look.jpg]
"""
import json, os, sys, time, warnings
warnings.filterwarnings("ignore")
from smbus2 import SMBus
from hw import PCA9685, ADS7830

pan = int(sys.argv[1]) if len(sys.argv) > 1 else 1500
tilt = int(sys.argv[2]) if len(sys.argv) > 2 else 1500
out = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser("~/car/look.jpg")

bus = SMBus(1)
pwm = PCA9685(bus, freq=50)
pwm.set_pulse_us(8, pan)
pwm.set_pulse_us(9, tilt)

from picamera2 import Picamera2
from libcamera import Transform
cam = Picamera2()
cam.configure(cam.create_still_configuration(main={"size": (960, 540)}, transform=Transform()))
cam.start()
time.sleep(1.0)                       # servo settle + auto-exposure
cam.capture_file(out)
cam.close()

from gpiozero import DistanceSensor, DigitalInputDevice
s = DistanceSensor(echo=22, trigger=27, max_distance=3, queue_len=5)
time.sleep(0.3)
dist = round(s.distance * 100, 1)
s.close()
ir = [DigitalInputDevice(p, pull_up=None, active_state=True) for p in (14, 15, 23)]
line = [int(d.value) for d in ir]
for d in ir: d.close()
batt = ADS7830(bus).raw(2)
pwm.all_off()                         # release servos (camera stays where it is)
print(json.dumps({"image": out, "pan_us": pan, "tilt_us": tilt, "distance_cm": dist,
                  "line_LMR": line, "battery_adc": batt}))
