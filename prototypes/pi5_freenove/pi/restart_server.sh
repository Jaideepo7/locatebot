#!/bin/bash
# (re)start the car HTTP server in the background
for p in $(pgrep -f "python3 server.py"); do kill "$p"; done
sleep 0.7
cd "$(dirname "$0")"
nohup python3 server.py > /tmp/server.log 2>&1 < /dev/null &
for i in $(seq 1 20); do
  ss -ltn | grep -q ":8000 " && { echo "server up (pid $(pgrep -f 'python3 server.py'))"; exit 0; }
  sleep 0.5
done
echo "server failed to start:"; tail -20 /tmp/server.log; exit 1
