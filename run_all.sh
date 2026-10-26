#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "ERROR: gnome-terminal tidak ditemukan." >&2
  exit 1
fi

gnome-terminal --window --title="Gazebo" -- bash -lc "exec '${DIR}/run_sim.sh'"
sleep 5
gnome-terminal --window --title="RViz" -- bash -lc "exec '${DIR}/run_rviz.sh'"
sleep 5
gnome-terminal --window --title="Kamera" -- bash -lc "exec '${DIR}/run_camera.sh'"
sleep 2
gnome-terminal --window --title="Navigasi" -- bash -lc "exec '${DIR}/run_nav.sh'"
