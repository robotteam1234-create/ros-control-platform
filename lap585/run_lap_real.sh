#!/bin/bash
# Pinky REAL-WORLD mapping (no Gazebo, no use_sim_time).
# Order: robot driver -> SLAM -> Nav2 -> 4-stage mapping.
# Ctrl+C lowers everything that was raised here.
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
# Auto-source base + Pinky Pro workspace (fixes "Missing ROS package" when
# launched from clean shell / nohup).
# shellcheck disable=SC1091
set +u  # ROS setup.bash uses unbound vars (AMENT_TRACE_SETUP_FILES)
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
for ws in "$HOME/pinky_pro/install/setup.bash" "$HOME/ros2_ws/install/setup.bash" "$HOME/dev_ws/install/setup.bash"; do
  [ -f "$ws" ] && source "$ws" && echo "sourced $ws"
done
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"  # match running Pinky stack (1420 uses 12)
# REAL safety gaps calibrated from field logs (sim defaults are tighter).
# Median clearance on real runs ~3.7cm, so FREE 5cm is unreachable.
export PINKY_HARD_GAP="${PINKY_HARD_GAP:-0.012}"
export PINKY_WATCH_GAP="${PINKY_WATCH_GAP:-0.025}"
export PINKY_FREE_GAP="${PINKY_FREE_GAP:-0.035}"
export PINKY_APPROACH_MAX="${PINKY_APPROACH_MAX:-0.05}"
# Unknown-size rooms: disable absolute map-size diverge check.
# export PINKY_TRACK_W=0 PINKY_TRACK_H=0

for pkg in pinky_bringup pinky_navigation; do
  if ! ros2 pkg prefix "$pkg" > /dev/null 2>&1; then
    echo "Missing ROS package: $pkg. Source workspace first: source <ws>/install/setup.bash" >&2
    exit 1
  fi
done

trap 'kill 0' EXIT
L=/tmp/lap_real
mkdir -p "$L"

# Coexistence: Pinky features (lamp/BLE/bringup PID 1420) already own the
# Dynamixel serial port — a second bringup dies with SerialException
# "multiple access on port". Reuse the running driver when present.
if ros2 node list 2>/dev/null | grep -q bringup || pgrep -f "bringup_robot.launch.xml" > /dev/null; then
  echo "1/4 robot driver — already running (Pinky features active), reusing"
else
  echo "1/4 robot driver (real lidar/odom/TF, no sim)"
  ros2 launch pinky_bringup bringup_robot.launch.xml > "$L/robot.log" 2>&1 &
  sleep 10
fi

echo "2/4 SLAM (slam_toolbox, real time)"
ros2 launch pinky_navigation map_building.launch.xml use_sim_time:=false \
    slam_params_file:="$D/mapper_params_track.yaml" > "$L/slam.log" 2>&1 &
sleep 10
if ! ros2 topic info /map > /dev/null 2>&1; then
  echo "WARN: /map not visible yet, check $L/slam.log"
fi
# slam_toolbox boots lifecycle-inactive (no autostart) and stays there
# across reboots until explicitly activated — with 0 /map publishers and
# no map->odom TF (robot_2 failure, 2026-09-17). Activate before mapping.
if ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q inactive; then
  echo "SLAM inactive — activating lifecycle"
  ros2 lifecycle set /slam_toolbox activate
  sleep 3
fi

echo "3/4 Nav2 (real params: ${D}/nav2_params_real.yaml)"
ros2 launch pinky_navigation navigation_launch.xml use_sim_time:=false \
    params_file:="$D/nav2_params_real.yaml" > "$L/nav2.log" 2>&1 &
sleep 15
if ! ros2 lifecycle get /controller_server > /dev/null 2>&1; then
  echo "WARN: /controller_server not active, check $L/nav2.log"
fi
echo "HOLD joystick / e-stop in hand. Supervisor required (28% sim contact rate)."

echo "4/4 mapping — real speeds, no --sim"
cd "$D" && python3 -u pinky_hybrid.py --save "$D/map_real"
echo "Done. Map: $D/map_real.pgm"
