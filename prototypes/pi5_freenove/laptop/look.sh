#!/bin/bash
# look.sh [pan_us] [tilt_us] -> runs pi/look.py on the Pi (detached), copies the photo to $CAR_OUT/look_N.jpg
# Env: CAR_SSH (default "car"), CAR_OUT (default ./captures). Use this when server.py is NOT running.
HERE=$(cd "$(dirname "$0")" && pwd)
CAR_SSH=${CAR_SSH:-car}
OUT=${CAR_OUT:-captures}
mkdir -p "$OUT"
N=$(ls "$OUT"/look_*.jpg 2>/dev/null | wc -l)
"$HERE/carrun.sh" "python3 look.py ${1:-1500} ${2:-1500}" 2>&1 | grep '^{'
timeout 60 scp -q -o BatchMode=yes "$CAR_SSH:car/look.jpg" "$OUT/look_$N.jpg" && echo "IMG $OUT/look_$N.jpg"
