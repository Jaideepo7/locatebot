#!/bin/bash
# stop the HTTP car server (frees the motors / ultrasonic GPIO for other programs)
for p in $(pgrep -f "python3 server.py"); do kill "$p"; done
sleep 0.5
pgrep -f "python3 server.py" >/dev/null && echo "server still running" || echo "server stopped"
