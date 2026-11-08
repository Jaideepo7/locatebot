#!/usr/bin/env python3
"""
server.py - HTTP control layer for the car (what an LLM / laptop talks to).
Keeps the camera running so snapshots are fast. All moves are time-capped and auto-stop.

GET /snap[?pan=1500&tilt=1500]      -> JPEG (moves camera first if pan/tilt given)
GET /sense                          -> {"distance_cm", "line_LMR", "battery_adc"}
GET /drive?l=50&r=50&t=0.5          -> differential drive (sides -100..100)
GET /forward?speed=50&t=1   /backward?speed=50&t=1
GET /turn?kind=spin_left&speed=70&t=0.5
GET /pantilt?pan=1500&tilt=1500
GET /stop
"""
import io, json, threading, time, warnings
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
warnings.filterwarnings("ignore")

from car import Car
from hw import ADS7830
from picamera2 import Picamera2
from libcamera import Transform
from gpiozero import DigitalInputDevice

PORT = 8000
car = Car()
move_lock = threading.Lock()
cam_lock = threading.Lock()
servo = {"pan": 1500, "tilt": 1500}
car.pwm.set_pulse_us(8, servo["pan"]); car.pwm.set_pulse_us(9, servo["tilt"])

cam = Picamera2()
cam.configure(cam.create_still_configuration(main={"size": (960, 540)}, transform=Transform()))
cam.start()
time.sleep(1.0)

ir = [DigitalInputDevice(p, pull_up=None, active_state=True) for p in (14, 15, 23)]
adc = ADS7830(car.bus)


def set_pantilt(pan=None, tilt=None):
    moved = False
    if pan is not None:
        pan = max(800, min(2200, int(pan))); moved |= pan != servo["pan"]
        servo["pan"] = pan; car.pwm.set_pulse_us(8, pan)
    if tilt is not None:
        tilt = max(800, min(2200, int(tilt))); moved |= tilt != servo["tilt"]
        servo["tilt"] = tilt; car.pwm.set_pulse_us(9, tilt)
    return moved


def sense():
    return {"distance_cm": car.distance_cm(), "line_LMR": [int(d.value) for d in ir],
            "battery_adc": adc.raw(2), "pan": servo["pan"], "tilt": servo["tilt"]}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code); self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path); q = {k: v[0] for k, v in parse_qs(u.query).items()}
        f = lambda k, d: float(q.get(k, d))
        try:
            if u.path == "/snap":
                if set_pantilt(q.get("pan"), q.get("tilt")):
                    time.sleep(0.5)
                buf = io.BytesIO()
                with cam_lock:
                    cam.capture_file(buf, format="jpeg")
                b = buf.getvalue()
                self.send_response(200); self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
                return
            if u.path == "/sense":
                return self._json(sense())
            if u.path == "/pantilt":
                set_pantilt(q.get("pan"), q.get("tilt")); return self._json({"ok": True, **servo})
            if u.path == "/stop":
                return self._json(car.stop())
            with move_lock:
                if u.path == "/drive":
                    return self._json(car.drive(f("l", 50), f("r", 50), f("t", 0.5)))
                if u.path == "/forward":
                    return self._json(car.forward(f("speed", 50), f("t", 1.0)))
                if u.path == "/backward":
                    return self._json(car.backward(f("speed", 50), f("t", 1.0)))
                if u.path == "/turn":
                    return self._json(car.turn(q.get("kind", "spin_left"), f("speed", 70), f("t", 0.5)))
            self._json({"ok": False, "error": "unknown path", "doc": __doc__}, 404)
        except Exception as e:
            car.stop()
            self._json({"ok": False, "error": repr(e)}, 500)


if __name__ == "__main__":
    print(f"car server on :{PORT}", flush=True)
    try:
        ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
    finally:
        car.stop(); car.pwm.all_off(); cam.close()
