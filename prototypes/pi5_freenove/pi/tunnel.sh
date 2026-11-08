#!/bin/bash
# Public, password-protected https link to the app (ngrok). One-time setup: ~/bin/ngrok config add-authtoken <token>
cd "$(dirname "$0")"
for p in $(pgrep -f "[n]grok http"); do kill "$p"; done
nohup ~/bin/ngrok http 8000 --traffic-policy-file tunnel_policy.yml --log stdout --log-format logfmt > /tmp/ngrok.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
  url=$(curl -s localhost:4040/api/tunnels | python3 -c "import json,sys; t=json.load(sys.stdin)[\"tunnels\"]; print(t[0][\"public_url\"] if t else \"\")" 2>/dev/null)
  [ -n "$url" ] && { echo "$url"; exit 0; }
  pgrep -f "[n]grok http" >/dev/null || break
  sleep 0.5
done
echo "ngrok did not start:"; grep -iE "err|fail" /tmp/ngrok.log | tail -5; exit 1
