#!/usr/bin/env python3
"""Pan the camera left<->right while capturing.
  python3 pan_capture.py photos   -> 5 photos at pan positions, ~/car/pan_*.jpg
  python3 pan_capture.py video    -> 12 s smooth-sweep video, ~/car/sweep.mp4
"""
import math, os, sys, time
from smbus2 import SMBus
from picamera2 import Picamera2
from libcamera import Transform
from hw import PCA9685, OLED

PAN, TILT = 8, 9
LEFT, RIGHT, CENTER = 1000, 2000, 1500
TILT_LEVEL = 1500

mode = sys.argv[1] if len(sys.argv) > 1 else "photos"
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

def ease_to(start, end, secs, steps=60):
    for i in range(steps + 1):
        t = i / steps
        k = (1 - math.cos(math.pi * t)) / 2          # smooth start/stop
        pwm.set_pulse_us(PAN, start + (end - start) * k)
        time.sleep(secs / steps)

cam = Picamera2()
try:
    pwm.set_pulse_us(TILT, TILT_LEVEL)
    pwm.set_pulse_us(PAN, CENTER)
    time.sleep(0.8)
    if mode == "photos":
        cam.configure(cam.create_still_configuration(main={"size": (1280, 720)},
                                                     transform=Transform()))
        cam.start(); time.sleep(1.5)
        pos = CENTER
        for n, target in enumerate((LEFT, 1250, CENTER, 1750, RIGHT)):
            say("PHOTO %d/5" % (n + 1), "pan %d us" % target)
            ease_to(pos, target, 0.8); pos = target
            time.sleep(0.5)                            # let it settle so the shot is sharp
            cam.capture_file(os.path.expanduser(f"~/car/pan_{n+1}_{target}.jpg"))
        ease_to(pos, CENTER, 0.8)
    else:
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FfmpegOutput
        cam.configure(cam.create_video_configuration(main={"size": (1280, 720)},
                                                     transform=Transform()))
        say("RECORDING", "smooth sweep")
        cam.start_recording(H264Encoder(bitrate=6_000_000), FfmpegOutput(os.path.expanduser("~/car/sweep.mp4")))
        time.sleep(1.0)
        ease_to(CENTER, LEFT, 2.0)
        ease_to(LEFT, RIGHT, 4.0)
        ease_to(RIGHT, LEFT, 4.0)
        ease_to(LEFT, CENTER, 2.0)
        time.sleep(0.5)
        cam.stop_recording()
    say("DONE", mode)
finally:
    try: cam.close()
    except Exception: pass
    pwm.all_off()
