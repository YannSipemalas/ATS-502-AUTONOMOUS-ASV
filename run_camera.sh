#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash

echo "Kamera robot: membaca /camera/image_raw dan menyimpan kotak terdeteksi."
echo "Overlay deteksi bola/kotak: /camera/detections/image_raw"
echo "Hasil: ${DIR}/captured_images/{green_box,blue_box}/"
python3 "${DIR}/scripts/camera_capture.py" \
  --ros-args -p use_sim_time:=true &
CAMERA_PID=$!

cleanup() {
  kill "${CAMERA_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2
ros2 run rqt_image_view rqt_image_view /camera/detections/image_raw
