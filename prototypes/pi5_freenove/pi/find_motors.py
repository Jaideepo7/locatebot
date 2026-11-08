#!/usr/bin/env python3
"""Interactive motor finder. Pulses ONE PCA9685 channel at a time, then asks what moved.
Results are saved to ~/car/motor_map.json.
Run:  python3 ~/car/find_motors.py          (optional: --duty 2500 --time 1.0 --ch 0-7)
"""
import argparse, atexit, json, os, signal, sys, time
from smbus2 import SMBus
from hw import PCA9685, ADS7830, OLED

ap = argparse.ArgumentParser()
ap.add_argument("--duty", type=int, default=2000, help="0-4095, default 2000 (~50%%)")
ap.add_argument("--time", type=float, default=0.8, help="seconds per pulse")
ap.add_argument("--ch", default="0-15", help="channels, e.g. 0-7 or 0,1,4")
args = ap.parse_args()

def parse_ch(s):
    out = []
    for part in s.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out

bus = SMBus(1)
pwm = PCA9685(bus)            # all channels off, 50 Hz
adc = ADS7830(bus)
try:
    oled = OLED(bus)
except OSError:
    oled = None

def stop(*_):
    try:
        pwm.all_off()
    except Exception:
        pass
atexit.register(stop)
signal.signal(signal.SIGINT, lambda *a: (stop(), print("\nSTOPPED - all channels off"), sys.exit(0)))
signal.signal(signal.SIGTERM, lambda *a: (stop(), sys.exit(0)))

def show(big, small=""):
    if oled:
        try:
            oled.text(big, small)
        except OSError:
            pass

WHEELS = {"1": "front-left", "2": "front-right", "3": "back-left", "4": "back-right",
          "l": "left", "r": "right", "s": "servo", "n": "nothing"}

print(__doc__)
print("BEFORE YOU START:\n  * Car's wheels OFF THE GROUND (on a box/cup)\n  * Power switch ON and LOAD switch ON\n"
      "  * Ctrl+C at any time stops everything immediately\n")
print("ADC raw ch0..7 (battery is one of these):", adc.all_raw())
input("Press Enter when ready... ")

results = {}
mpath = os.path.expanduser("~/car/motor_map.json")
for ch in parse_ch(args.ch):
    while True:
        show(f"CH {ch}", "pulsing...")
        print(f"\n>>> Channel {ch}: pulsing for {args.time}s at duty {args.duty}")
        time.sleep(0.4)
        pwm.set_duty(ch, args.duty)
        time.sleep(args.time)
        pwm.set_duty(ch, 0)
        show(f"CH {ch}", "what moved?")
        a = input("  What moved?  1=front-left 2=front-right 3=back-left 4=back-right\n"
                  "               l=left r=right s=servo n=nothing  x=repeat  (or type anything): ").strip().lower()
        if a == "x":
            continue
        what = WHEELS.get(a, a or "nothing")
        entry = {"moved": what}
        if what not in ("nothing", "servo"):
            d = input("  Direction?  f=forward  b=backward  ?=not sure: ").strip().lower()
            entry["dir"] = {"f": "forward", "b": "backward"}.get(d, "unsure")
        results[ch] = entry
        with open(mpath, "w") as f:
            json.dump(results, f, indent=2)
        break

stop()
show("DONE", "map saved")
print("\nRESULTS:")
for ch, e in results.items():
    print(f"  ch {ch:2}: {e}")
print(f"\nSaved to {mpath}")
