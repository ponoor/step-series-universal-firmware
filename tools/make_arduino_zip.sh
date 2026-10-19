#!/bin/sh
# Builds step-series-universal-firmware.zip for the Arduino IDE (a release asset).
# The zip holds one flat sketch folder: every file of src/, with main.cpp as the .ino.
# Usage: tools/make_arduino_zip.sh [output-dir]   (default: ./release_dist)
set -e
cd "$(dirname "$0")/.."
out="${1:-release_dist}"
name=step-series-universal-firmware
tmp="$(mktemp -d)"
mkdir -p "$tmp/$name" "$out"
cp src/*.c src/*.cpp src/*.h "$tmp/$name/"
mv "$tmp/$name/main.cpp" "$tmp/$name/$name.ino"
rm -f "$out/$name.zip"
(cd "$tmp" && zip -qr "$OLDPWD/$out/$name.zip" "$name")
rm -rf "$tmp"
echo "$out/$name.zip"
