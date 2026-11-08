#!/bin/bash
# Deploy the phone app (botd.py, motion.py, restart/tunnel scripts, web/) to the car, restart botd, check the app.
#   ./deploy_app.sh          deploy now
#   ./deploy_app.sh --wait   first wait (up to 3 h) for the car to come online
# Env: CAR_SSH (ssh target, default "car" - an ~/.ssh/config alias), CAR_URL (default http://raspberrypi.local:8000)
PI="$(cd "$(dirname "$0")/../pi" && pwd)"
CAR_SSH="${CAR_SSH:-car}"; CAR="${CAR_URL:-http://raspberrypi.local:8000}"
if [ "$1" = "--wait" ]; then
  for i in $(seq 1 1080); do
    ssh -o ConnectTimeout=4 -o BatchMode=yes "$CAR_SSH" true 2>/dev/null && break
    sleep 10
  done
  sleep 20      # let the board, LiDAR and camera settle after boot
fi
ssh -o ConnectTimeout=6 "$CAR_SSH" true || { echo "car not reachable"; exit 1; }
set -e
ssh "$CAR_SSH" 'mkdir -p ~/car/web/fonts && [ -f ~/car/botd.py ] && cp ~/car/botd.py ~/car/botd.py.bak-$(date +%m%d-%H%M%S) || true'
scp -q "$PI/botd.py" "$PI/motion.py" "$PI/restart_botd.sh" "$PI/tunnel.sh" "$CAR_SSH":car/
scp -q -r "$PI/web/." "$CAR_SSH":car/web/
ssh "$CAR_SSH" 'cd ~/car && chmod +x restart_botd.sh tunnel.sh && python3 -m py_compile botd.py motion.py && ./restart_botd.sh'
set +e
curl -s --retry-connrefused --retry 10 --retry-delay 1 -m 5 -o /dev/null "$CAR/state"
echo "--- checks"
curl -s -m 5 "$CAR/state" | python3 -c "import json,sys; s=json.load(sys.stdin); print('state: mission', s.get('mission'), '| front', s.get('front_cm'), s.get('front_source'), '| control_age', s.get('control_age_s'), '| battery', s.get('battery_adc'))"
for p in / /manifest.webmanifest /icon-192.png /fonts/PixelifySans.woff2 /sw.js; do
  curl -s -m 5 -o /dev/null -w "$p %{http_code} %{content_type} %{size_download}B\n" "$CAR$p"
done
TMP=$(mktemp); curl -s -m 4 -o "$TMP" "$CAR/stream"; echo "stream: $(grep -c -a -- '--frame' "$TMP") frames in 4 s"; rm -f "$TMP"
echo "stop: $(curl -s -m 5 "$CAR/stop")"
echo "pending questions: $(curl -s -m 5 "$CAR/inbox?status=pending")"
