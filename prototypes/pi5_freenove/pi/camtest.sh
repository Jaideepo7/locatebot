#!/bin/bash
# Camera check + one test photo at ~/car/test.jpg
rpicam-hello --list-cameras -t 1 2>&1 | head -8
if rpicam-hello --list-cameras -t 1 2>&1 | grep -q "Available cameras"; then
  rpicam-still -n -t 1500 --width 1280 --height 720 -o ~/car/test.jpg 2>&1 | tail -2
  ls -la ~/car/test.jpg
else
  echo "NO CAMERA DETECTED"
fi
