#!/usr/bin/env bash
# =============================================================
# run_rviz.sh  –  Buka RViz2 dan node pendukung visualisasi
# Jalankan SETELAH run_sim.sh; Gazebo berjalan otomatis dengan opsi -r
# =============================================================
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/ats_sems5_rviz_${UID}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "ERROR: run_rviz.sh untuk workspace ini sudah berjalan." >&2
  exit 1
fi

EXISTING_NODES="$(ros2 node list 2>/dev/null || true)"
STALE_NODES="$(grep -E '^/(ros_gz_bridge|static_transform_publisher.*|robot_state_publisher|arena_markers_pub|robot_description_pub|rviz)$' <<<"${EXISTING_NODES}" || true)"
if [[ -n "${STALE_NODES}" ]]; then
  echo "ERROR: node visualisasi lama masih aktif; hentikan sebelum run_rviz.sh:" >&2
  printf '%s\n' "${STALE_NODES}" >&2
  exit 1
fi

echo "======================================================"
echo " Arena Simulasi – RViz2"
echo "======================================================"

# ─── 0. Regenerasi robot_resolved.urdf (selalu fresh) ────────
echo "[0] Menyiapkan URDF ..."
sed "s|@ROOT@|${DIR}|g" "${DIR}/urdf/robot.urdf" > "${DIR}/urdf/robot_resolved.urdf"
URDF_BYTES=$(wc -c < "${DIR}/urdf/robot_resolved.urdf")
echo "    URDF siap: ${DIR}/urdf/robot_resolved.urdf (${URDF_BYTES} bytes)"

# ─── 1. ros_gz_bridge ────────────────────────────────────────
echo "[1/5] ros_gz_bridge ..."
setsid ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  --ros-args -p use_sim_time:=true &
BRIDGE_PID=$!
sleep 2

# ─── 2. Static TF: world → odom ─────────────────────────────
# DiffDrive mempublish odom→base_footprint. Static TF world→odom
# diset ke posisi spawn robot (dibaca dari arena_layout.json).
# Ini menyambungkan: world → odom → base_footprint → base_link → ...
read SX SY SYAW < <(python3 -c \
  "import json; s=json.load(open('${DIR}/config/arena_layout.json'))['spawn']; \
   print(s['x'], s['y'], s['yaw'])")
echo "[2/5] static TF world → odom  (spawn: x=${SX} y=${SY} yaw=${SYAW}) ..."
setsid ros2 run tf2_ros static_transform_publisher \
  --x "${SX}" --y "${SY}" --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw "${SYAW}" \
  --frame-id world --child-frame-id odom &
TF_PID=$!
sleep 1

# ─── 3. robot_state_publisher ────────────────────────────────
# Pakai FILE PATH langsung (bukan string parameter) – ini cara paling reliable
# RSP versi 3.3.4 mendukung: ros2 run robot_state_publisher robot_state_publisher <file>
echo "[3/5] robot_state_publisher (file URDF langsung) ..."
setsid ros2 run robot_state_publisher robot_state_publisher \
  "${DIR}/urdf/robot_resolved.urdf" \
  --ros-args -p use_sim_time:=true &
RSP_PID=$!
sleep 2

# ─── 4. Arena marker publisher ───────────────────────────────
echo "[4/5] arena_markers_pub ..."
setsid python3 "${DIR}/scripts/arena_markers_pub.py" --ros-args -p use_sim_time:=true &
MARKER_PID=$!
sleep 1

# ─── 5. RViz2 ─────────────────────────────────────────────────
echo "[5/5] RViz2 ..."
setsid rviz2 -d "${DIR}/config/arena_rviz.rviz" \
  --ros-args -p use_sim_time:=true &
RVIZ_PID=$!

echo ""
echo "======================================================"
echo " PID aktif:"
echo "   [1] ros_gz_bridge      $BRIDGE_PID"
echo "   [3] robot_state_pub    $RSP_PID"
echo "   [4] arena_markers      $MARKER_PID"
echo "   [5] rviz2              $RVIZ_PID"
echo "   Navigasi dijalankan terpisah: ${DIR}/run_nav.sh"
echo "   Jalankan kamera di terminal terpisah: ${DIR}/run_camera.sh"
echo "   Capture: ${DIR}/captured_images/{green_box,blue_box}/"
echo ""
echo " TF chain saat robot di START:"
echo "   odom ─(DiffDrive /tf)─► base_footprint"
echo "          └──(robot_state_pub)──► base_link, left_wheel, right_wheel, camera"
echo ""
echo " ✓ Robot seharusnya muncul di garis START (kanan bawah arena)"
echo " ⚠ Tekan Ctrl+C untuk menutup semua"
echo "======================================================"

cleanup() {
  trap - EXIT SIGINT SIGTERM
  for pid in "${BRIDGE_PID}" "${TF_PID}" "${RSP_PID}" "${MARKER_PID}" "${RVIZ_PID}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  wait "${BRIDGE_PID}" "${TF_PID}" "${RSP_PID}" "${MARKER_PID}" "${RVIZ_PID}" 2>/dev/null || true
}
trap cleanup EXIT SIGINT SIGTERM

wait "${RVIZ_PID}"
