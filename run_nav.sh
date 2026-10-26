#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/ats_sems5_nav_${UID}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "ERROR: run_nav.sh untuk workspace ini sudah berjalan." >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -Fxq "/auto_nav"; then
  echo "ERROR: node /auto_nav sudah aktif; hentikan navigasi lama sebelum menjalankan ulang." >&2
  exit 1
fi

echo "Navigasi waypoint: Pure Pursuit + safety controller."
echo "Pastikan run_sim.sh dan run_rviz.sh sudah berjalan satu kali."
exec python3 "${DIR}/scripts/auto_nav.py" \
  --ros-args -p use_sim_time:=true
