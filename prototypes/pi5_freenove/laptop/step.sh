#!/bin/bash
# step.sh [L R T]  -> optional differential drive via server.py, then snapshot + sensors.
# Saves the image to $CAR_OUT/f_N.jpg and prints the path + sensor JSON.
# Env: CAR_URL (default http://raspberrypi.local:8000), CAR_OUT (default ./captures)
B=${CAR_URL:-http://raspberrypi.local:8000}
OUT=${CAR_OUT:-captures}
mkdir -p "$OUT"
if [ -n "$1" ]; then
  curl -s -m 15 "$B/drive?l=$1&r=$2&t=${3:-0.5}" | python3 -c "import json,sys; d=json.load(sys.stdin); print('drive', {k:d.get(k) for k in ('left','right','moved_s','stopped_because','distance_cm_end')})"
  sleep 0.25
fi
N=$(ls "$OUT"/f_*.jpg 2>/dev/null | wc -l)
curl -s -m 15 -o "$OUT/f_$N.jpg" "$B/snap"
echo "sense $(curl -s -m 10 "$B/sense")"
echo "IMG $OUT/f_$N.jpg"
