#!/bin/bash
# (re)start botd.py in the background; stops server.py first (both need the motors/camera)
cd "$(dirname "$0")"
for p in $(pgrep -f "python3 [s]erver.py"); do kill "$p"; done
for p in $(pgrep -f "python3 [b]otd.py"); do kill "$p"; done
for i in $(seq 1 16); do                       # wait for the old one to really exit (LiDAR/camera shutdown)
  pgrep -f "python3 [b]otd.py" >/dev/null || break
  sleep 0.5
done
for p in $(pgrep -f "python3 [b]otd.py"); do echo "old botd $p did not exit: kill -9"; kill -9 "$p"; done
while ss -ltn | grep -q ":8000 "; do sleep 0.3; done
nohup python3 botd.py > /tmp/botd.log 2>&1 < /dev/null &
NEW=$!
for i in $(seq 1 40); do
  kill -0 $NEW 2>/dev/null || break
  ss -ltn | grep -q ":8000 " && { echo "botd up (pid $NEW)"; exit 0; }
  sleep 0.5
done
echo "botd failed to start:"; tail -25 /tmp/botd.log; exit 1
