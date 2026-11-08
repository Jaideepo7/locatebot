#!/bin/bash
# Run car commands DETACHED on the Pi so a Wi-Fi/SSH drop can't cut a sequence short.
#   carrun.sh 'python3 motion.py approach 50 6'
# Env: CAR_SSH (ssh target, default "car" - an ~/.ssh/config alias), CARRUN_WAIT (poll attempts, default 60)
# A retry never starts a second copy: the remote side only starts if this job's log doesn't exist yet.
CAR_SSH=${CAR_SSH:-car}
ID=$(date +%s%N)
LOG=/tmp/carrun_$ID.log
for i in 1 2 3; do
  timeout 60 ssh -o BatchMode=yes "$CAR_SSH" "[ -e $LOG ] || (cd ~/car && nohup bash -c '$1; echo __DONE__' > $LOG 2>&1 &)" && break
  sleep 2
done
for i in $(seq 1 "${CARRUN_WAIT:-60}"); do
  out=$(timeout 30 ssh -o BatchMode=yes "$CAR_SSH" "cat $LOG 2>/dev/null")
  if grep -q __DONE__ <<<"$out"; then grep -v __DONE__ <<<"$out"; exit 0; fi
  sleep 1
done
echo "TIMEOUT waiting; partial log:"; echo "$out"; exit 1
