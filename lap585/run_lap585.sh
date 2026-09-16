#!/bin/bash
# Pinky 585회차 재현: 시뮬 -> SLAM -> Nav2 -> 4단계 매핑을 한 번에 띄운다.
# Pinky Pro 워크스페이스를 먼저 source 한 뒤 이 폴더에서 실행한다.
#   source <워크스페이스>/install/setup.bash && ./run_lap585.sh
# Ctrl+C 로 멈추면 띄운 것을 모두 내린다.
D="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"    # 다른 사람과 겹치지 않는 번호로
if ! ros2 pkg prefix pinky_gz_sim > /dev/null 2>&1; then
    echo "Pinky Pro 패키지(pinky_gz_sim)를 못 찾았습니다. 워크스페이스를 먼저 source 하세요."
    exit 1
fi
trap 'kill 0' EXIT
L=/tmp/lap585
mkdir -p "$L"

echo "1/4 시뮬 — 출발 월드 (1.090, 0.317), 135°"
ros2 launch pinky_gz_sim launch_sim.launch.xml world:="$D/map_260905.world" \
    spawn_x:=1.090 spawn_y:=0.317 spawn_yaw:=2.3562 > "$L/sim.log" 2>&1 &
sleep 30

echo "2/4 SLAM (slam_toolbox)"
ros2 launch pinky_navigation map_building.launch.xml use_sim_time:=true \
    slam_params_file:="$D/mapper_params_track.yaml" > "$L/slam.log" 2>&1 &
sleep 15

echo "3/4 Nav2"
ros2 launch pinky_navigation navigation_launch.xml use_sim_time:=True \
    params_file:="$D/nav2_params_sim.yaml" > "$L/nav2.log" 2>&1 &
sleep 25

echo "4/4 매핑 시작 — 약 6~9분 (띄운 것들의 로그: $L/)"
cd "$D" && python3 -u pinky_hybrid.py --sim --save "$D/map_lap585"
echo "끝. 맵: $D/map_lap585.pgm"
