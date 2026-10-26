#!/usr/bin/env bash
# Jalankan arena + robot di Gazebo Sim
DIR="$(cd "$(dirname "$0")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$DIR/models:${GZ_SIM_RESOURCE_PATH}"
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/ats_sems5_sim_${UID}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "ERROR: run_sim.sh untuk workspace ini sudah berjalan." >&2
  exit 1
fi
python3 "$DIR/generate_arena.py"
exec gz sim -r "$DIR/worlds/arena.world" "$@"
