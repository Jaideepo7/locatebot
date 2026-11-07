#!/usr/bin/env bash
# Download the LocateAnything q4_k GGUF used by locatebot, pinned and checksum-verified.
#
#   scripts/get_model.sh                 # -> models/locate-anything-q4_k.gguf
#   MODELS_DIR=/data/models scripts/get_model.sh
#
# The weights are NVIDIA's (NVIDIA license, https://huggingface.co/nvidia/LocateAnything-3B),
# repackaged as GGUF by mudler. They are NOT stored in this repo; models/ is git-ignored.
# Resumable: re-run after an interrupted download and it continues where it stopped.
set -euo pipefail

REPO="mudler/locate-anything.cpp-gguf"
REVISION="4e50db5b836dd876f5e464734ef5484cae754a52"   # pinned Hugging Face commit
FILE="locate-anything-q4_k.gguf"
SHA256="894088a00a2cd2bbb7f34b12893988dd0376c8ed92213a9f2cf6420f1e3901da"
SIZE=4716320320                                       # bytes (~4.7 GB)

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODELS_DIR="${MODELS_DIR:-$ROOT/models}"
DEST="$MODELS_DIR/$FILE"
PART="$DEST.part"
URL="https://huggingface.co/$REPO/resolve/$REVISION/$FILE"

verify() { echo "$SHA256  $1" | sha256sum --check --status; }

mkdir -p "$MODELS_DIR"

if [[ -f "$DEST" ]]; then
  echo "Found $DEST, verifying checksum..."
  if verify "$DEST"; then echo "OK: $DEST is already the pinned q4_k model."; exit 0; fi
  echo "Checksum mismatch; moving it aside and downloading again." >&2
  mv "$DEST" "$DEST.bad.$(date +%s)"
fi

avail=$(df -B1 --output=avail "$MODELS_DIR" | tail -1)
have=0; [[ -f "$PART" ]] && have=$(stat -c %s "$PART")
if (( avail < SIZE - have + 200000000 )); then
  echo "Not enough disk space in $MODELS_DIR: need ~$(( (SIZE - have) / 1000000 )) MB free." >&2
  exit 1
fi

echo "Downloading $FILE (~4.7 GB) from $REPO@${REVISION:0:7}..."
curl -fL --retry 5 --retry-delay 3 -C - -o "$PART" "$URL"

got=$(stat -c %s "$PART")
if (( got != SIZE )); then echo "Size mismatch: got $got bytes, expected $SIZE." >&2; exit 1; fi
echo "Verifying checksum..."
if ! verify "$PART"; then echo "Checksum mismatch after download; deleting $PART." >&2; rm -f "$PART"; exit 1; fi
mv "$PART" "$DEST"
echo "OK: $DEST"
