#!/bin/bash
# (re)start botd.py in the background; stops server.py first (both need the motors/camera)
cd "$(dirname "$0")"
for p in $(pgrep -f "python3 server.py"); do kill "$p"; done
for p in $(pgrep -f "python3 botd.py"); do kill "$p"; done
sleep 1.5
nohup python3 botd.py > /tmp/botd.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
  ss -ltn | grep -q ":8000 " && { echo "botd up (pid $(pgrep -f 'python3 botd.py'))"; exit 0; }
  sleep 0.5
done
echo "botd failed to start:"; tail -25 /tmp/botd.log; exit 1
