# Pinky Control Center 운영 런북

이 런북은 현재 checkout에서 확인 가능한 mock 운영과 T12~T15 ROS/실물 인수 준비를 구분한다. T11 영상 녹화·재생은 범위에 없으며, 브라우저는 rosbridge에 직접 연결하지 않는다. 실물 주행 전에는 로봇 측 정지 래치·watchdog·Nav2 action server를 별도로 확인한다.

## 1. 설치와 빌드

Ubuntu 24.04와 Python 3.12를 기준으로 한다. 저장소를 `/opt/pinky-control-platform`에 배치하는 운영 설치는 다음을 따른다.

```bash
cd /opt/pinky-control-platform
python3.12 -m venv backend/.venv
backend/.venv/bin/pip install --upgrade pip
backend/.venv/bin/pip install -e 'backend[dev]'
cd frontend
npm ci
npm run build
cd ..
sudo install -d -o control-center -g control-center /var/lib/pinky-control-center /var/log/pinky-control-center
sudo install -d -m 0750 /etc/pinky-control-center
sudo install -m 0640 deployment/env.example /etc/pinky-control-center/control-center.env
sudo install -m 0640 deployment/robots.ros.example.yaml /etc/pinky-control-center/robots.ros.yaml
sudoedit /etc/pinky-control-center/control-center.env
deployment/scripts/validate-deployment.sh /etc/pinky-control-center/control-center.env
```

`CONTROL_PLATFORM_WORKERS=1`, `robot_1` domain `12`, `robot_2` domain `13`, 서로 다른 rosbridge URL을 바꾸지 않는다. 운영 origin이 HTTPS이면 `CONTROL_PLATFORM_SECURE_COOKIES=1`을 사용한다. 비밀 토큰은 env 또는 호스트 secret manager에서 주입하고 YAML·git·로그에 쓰지 않는다.

## 2. 관리자 계정과 mock 기동

계정 생성은 호스트 터미널에서 비밀번호를 직접 입력하는 일회성 명령으로 실행한다.

```bash
cd /opt/pinky-control-platform
backend/.venv/bin/python -m pinky_control_center.main \
  --database /var/lib/pinky-control-center/control.db \
  --reset-password operator --password '<터미널에서만 입력>' --role ADMIN
CONTROL_PLATFORM_DATABASE=/var/lib/pinky-control-center/control.db \
CONTROL_PLATFORM_MODE=mock CONTROL_PLATFORM_HOST=127.0.0.1 \
CONTROL_PLATFORM_PORT=8081 CONTROL_PLATFORM_ALLOWED_ORIGIN=http://localhost:5173 \
CONTROL_PLATFORM_SECURE_COOKIES=0 CONTROL_PLATFORM_WORKERS=1 \
deployment/scripts/start-backend.sh
```

개발 브라우저는 별도 터미널에서 `cd frontend && npm run dev`로 열고 `http://localhost:5173`을 사용한다. API의 `/health`가 `{"status":"ok","mode":"mock"}`를 반환하고 로그인 후 지도·두 카메라·MOCK 상태를 확인한다. 새 환경의 최소 검증은 다음 한 명령으로 실행한다.

```bash
deployment/scripts/acceptance.sh
```

## 3. TLS 동일 출처 배포

`deployment/nginx/control-platform.conf`를 nginx sites-enabled에 설치하고 인증서 경로와 `server_name`을 실제 값으로 바꾼다. 정적 UI, `/api/`, `/ws/`는 모두 같은 HTTPS origin에서 제공하며 backend 포트 8081은 loopback에만 바인딩한다.

```bash
sudo install -m 0644 deployment/nginx/control-platform.conf /etc/nginx/sites-available/pinky-control-center
sudo ln -sf /etc/nginx/sites-available/pinky-control-center /etc/nginx/sites-enabled/pinky-control-center
sudo nginx -t && sudo systemctl reload nginx
sudo install -m 0644 deployment/systemd/pinky-control-center.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pinky-control-center
systemctl status pinky-control-center --no-pager
```

실패 시 `journalctl -u pinky-control-center`와 `/var/log/pinky-control-center/server.log`에서 원인을 확인한다. 포트 충돌은 서버를 다른 로봇이나 mock endpoint로 자동 전환하지 않고 기동 실패로 남긴다.

## 4. 정지·재시작·복구

운영 중에는 UI의 전체 정지를 먼저 실행하고 두 로봇의 `CONFIRMED`를 확인한다. UI 정지는 소프트웨어 정지이며 물리 안전 장치가 아니므로 현장에서는 물리 정지 수단과 전원 차단을 함께 준비한다. 응답하지 않은 로봇은 `UNCONFIRMED`로 유지하고 현장 절차를 따른다. 서버 재시작은 진행 중 임무를 재개하지 않는다.

```bash
sudo systemctl stop pinky-control-center
sudo systemctl start pinky-control-center
sudo systemctl restart pinky-control-center
```

SQLite 백업은 서비스 정지 후 복사하고, 복원 전 기존 DB를 날짜가 붙은 파일로 보존한다.

```bash
sudo systemctl stop pinky-control-center
sudo cp --preserve=all /var/lib/pinky-control-center/control.db \
  /var/lib/pinky-control-center/control.db.$(date -u +%Y%m%dT%H%M%SZ)
sudo systemctl start pinky-control-center
```

복구는 서비스 정지 → 검증된 백업을 `control.db`로 복사 → 소유자/권한 확인 → 서비스 시작 → 로그인·이력 조회 순서다. 백업을 찾지 못하면 빈 DB를 만들어 운용 이력을 잃지 말고 관리자에게 복구 실패를 보고한다.

## 5. ROS/Gazebo 인수 준비

T12 ROS 어댑터가 설치된 별도 workspace에서만 수행한다. `/etc/pinky-control-center/robots.ros.yaml`의 실제 주소·매핑을 먼저 확인하고, bridge 두 개는 서로 다른 domain과 포트를 사용한다. `pinky-control-center.service`는 API만 관리하며 rosbridge launch의 lifecycle은 별도 ROS supervisor/operator가 관리한다.

관제 PC에 rosbridge가 없다면 먼저 ROS 2 Jazzy 패키지를 설치한다. 설치 후 `ros2 pkg prefix rosbridge_server`가 경로를 출력해야 한다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-rosbridge-server
ros2 pkg prefix rosbridge_server
```

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=12
ros2 launch pinky_gz_sim launch_sim.launch.xml namespace:=robot_1 world_name:=pinky_factory.world
```

다른 터미널에서 `ROS_DOMAIN_ID=13`으로 `namespace:=robot_2`를 실행한다. 현재 `pinky_gz_sim/launch/launch_sim.launch.xml`에는 두 인스턴스의 x/y spawn 인자가 없고 기본 spawn 위치가 겹칠 수 있으므로, 별도 world 또는 launch 수정으로 위치를 분리하기 전에는 dual-robot Gazebo 인수를 실행하지 않는다. 이 제한은 acceptance report에서 NOT_RUN으로 남긴다. 그 다음 준비된 rosbridge-only launch를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 deployment/launch/control_center.launch.py
```

이 launch는 API나 Gazebo를 시작하지 않고 `robot_1 → ws://127.0.0.1:9090`(domain 12), `robot_2 → ws://127.0.0.1:9091`(domain 13)의 rosbridge만 시작한다. `/robot_1`과 `/robot_2`의 `odom`, `scan`, `/tf`·`/tf_static`를 각각 확인한다. 현재 현장 공용 Wi-Fi 구성에서는 robot_1(`192.168.0.8`)에 rosbridge가 없어 PC의 domain 12 rosbridge `127.0.0.1:9090`을 사용하고, robot_2(`192.168.0.18`)는 로봇 내부 rosbridge `192.168.0.18:9091`을 사용한다. 실물 robot_2는 `ros/pinky_control_interfaces`와 `ros/pinky_control_watchdog`를 `/home/pinky/dev_ws/wj/src/`에 복사하고 `colcon build --symlink-install --packages-select pinky_control_interfaces pinky_control_watchdog`로 빌드한다. hardware bringup은 별도 터미널에서 실행하고, `deployment/scripts/start-pinky-robot2-session.sh`가 rosbridge·watchdog·카메라·압축 변환을 시작한다. `ros2 topic info /control/status -v`, `ros2 service type /control/command`, `/cmd_vel`의 유일한 publisher를 확인한다. `map → <robot>/odom → <robot>/base_footprint` TF가 유효할 때만 자동 주행 단계로 간다. control/follow 계약, 단일 cmd_vel 중재, stop 래치·watchdog 계약이 없으면 실물 인수는 중단하고 NOT_RUN으로 기록한다.

검증 순서는 무이동 상태의 상태 수신 → 카메라 → stop/reset service → MANUAL mode → 입력 중단 watchdog(선택 로봇 0속도, 정상 래치 없음) → Nav2 action/AMCL/TF 확인 → 저속 개별 주행 → 개별 정지 → 재연결이며, 무이동 검증을 통과하기 전에는 속도 제어를 열지 않는다. 웹소켓 단절·lease 만료·명시적 정지는 별도 보호 정지 래치로 확인한다. 결과와 명령·로그 증거는 [acceptance-report.md](acceptance-report.md)에 기록한다.

## 6. robot_2 단일 로봇 지도 주행·재현지화 시험

이 절차는 `robot_2`, ROS_DOMAIN_ID `13`, 현장 공용 Wi-Fi 주소 `192.168.0.18`인 현재 시험 구성을 기준으로 한다. `robot_1` domain 12와는 별도 rosbridge를 사용한다. 아래 절차를 수행해도 실제 이동 명령은 현장 담당자가 안전을 확인한 뒤 직접 실행해야 한다.

현장 클릭 목표의 Nav2 도달 허용오차는 평면 0.08m, 방향 0.17rad(약 10도)다. 변경 전 0.25m 설정에서는 목표 약 0.22m 전에 정상 성공 처리된 사례가 있으므로, 시험 기록에는 클릭 목표와 최종 `map→base_footprint` pose의 거리·방향 오차를 함께 남긴다.

주행 중 사용자 조작 없이 action이 `CANCELED`되고 history에 `SAFETY_STOP`이 남으면 같은 시각의 lease/session 만료를 확인한다. 현재 구현은 활성 제어 lease 자체가 만료되거나 그 lease를 소유한 로그인 세션이 만료될 때만 보호 정지하며, 제어권과 무관한 과거 로그인 세션 정리는 주행을 취소하지 않는다.

### 6.1 로봇 측 패키지 설치·빌드

로봇의 기존 bringup/session 프로세스를 확인한 뒤, 소스 패키지를 명시된 workspace에 복사한다. 기존 bringup은 유지할 수 있지만, 이전에 별도로 실행한 camera publisher·image republisher·rosbridge·watchdog는 session script와 중복되지 않게 종료한다.

```bash
ssh pinky@192.168.0.18 'mkdir -p /home/pinky/dev_ws/wj/src'
scp -r ros/pinky_control_interfaces ros/pinky_control_watchdog ros/pinky_control_navigation \
  pinky@192.168.0.18:/home/pinky/dev_ws/wj/src/
ssh pinky@192.168.0.18
```

로봇 shell에서 underlay를 먼저 source하고 의존성을 확인한 뒤 overlay를 빌드한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/pinky/dev_ws/wj
rosdep install --from-paths src/pinky_control_interfaces src/pinky_control_watchdog src/pinky_control_navigation \
  -y --ignore-src --skip-keys ament_python
colcon build --symlink-install --packages-select \
  pinky_control_interfaces pinky_control_watchdog pinky_control_navigation
source install/setup.bash
ros2 pkg prefix pinky_control_navigation
ros2 pkg prefix nav2_bt_navigator nav2_amcl nav2_map_server
```

`rosdep` 또는 `colcon`이 실패하면 주행 시험을 진행하지 않는다. 새 package가 install에 반영되기 전에는 session script의 `START_NAV2=1`이 실패하는 것이 정상이다.

### 6.2 무이동 기동과 graph 확인

workspace를 섞지 않는다. 터미널 A의 hardware bringup은 Pinky Pro 원본 workspace
(`~/pinky_pro`)에서 실행하고, 터미널 B의 control session/Nav2는 별도 overlay
workspace (`~/dev_ws/wj`)를 사용한다. 두 프로세스는 ROS_DOMAIN_ID 13과 ROS
토픽 graph로 연결되며, session script가 자체적으로 `wj/install/setup.bash`를
source한다.

일상적인 재부팅 후 기동은 systemd user unit(`pinky-bringup@<robot>`, `pinky-session@<robot>`)이 담당한다. 최초 1회는 로봇마다 설치 스크립트를 실행한다. 저장소의 `ros/pinky_control_bringup`을 로봇의 `/home/pinky/dev_ws/wj/src/` 아래에 복사해 `colcon build --symlink-install --packages-select pinky_control_bringup`으로 빌드한 뒤, 다음을 실행한다.

```bash
~/dev_ws/wj/install/pinky_control_bringup/share/pinky_control_bringup/scripts/install.sh robot_2
```

설치 스크립트는 workspace 빌드 여부를 확인하고, payload를 `~/pinky/startup/bin/`에, 환경 파일을 `~/.config/pinky-control/<robot>.env`에, unit을 `~/.config/systemd/user/`에 복사한 뒤 기존 `rosy-session-*` 부팅 서비스를 비활성화하고 linger를 확인한 뒤 두 unit을 `enable --now` 한다. sudo는 필요 없다. 이후 재부팅에서는 SSH 접속 없이 두 unit이 자동으로 시작된다. robot_1은 control workspace가 아직 없으므로 `docs/user-guide.md`의 절차로 workspace를 먼저 빌드해야 설치가 통과한다.

상태와 로그는 다음으로 확인한다.

```bash
systemctl --user status 'pinky-*'
journalctl --user -u 'pinky-*' -f
```

unit이 정상이면 `[pinky-bringup] ready`와 `[pinky-session] ready`가 로그에 남는다. 재부팅 뒤에는 AMCL 위치가 사라지므로 웹에서 실제 위치·방향을 지정하고 `위치 재설정(AMCL)`을 수행해야 한다. 세션 비정상 종료 시 systemd가 5초 뒤 재시작하며(120초당 최대 5회), 반복 실패는 `journalctl`로 원인을 확인한다.

아래의 터미널 A/B 절차는 통합 wrapper가 실패했을 때의 분리 진단 절차다.

터미널 A에서 hardware bringup을 유지한다.

```bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=13
source /opt/ros/jazzy/setup.bash
source /home/pinky/pinky_pro/install/setup.bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

터미널 B에서는 기존 user service나 수동으로 켜 둔 중복 프로세스를 정리한 뒤 session script를 실행한다. 스크립트는 bringup을 시작·종료하지 않지만, `START_NAV2=1`이면 bringup이 제공하는 `/start_motor`를 호출해 SLLidar를 시작한다.

```bash
systemctl --user stop rosy-session-control.service 2>/dev/null || true
START_NAV2=1 /home/pinky/start-pinky-robot2-session.sh
```

READY가 나오면 터미널 C에서 실제 이동 없이 다음을 확인한다.

```bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=13
source /opt/ros/jazzy/setup.bash
source /home/pinky/dev_ws/wj/install/setup.bash
ros2 action list -t | grep navigate_to_pose
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic info /control/status -v
ros2 topic info /control/nav_velocity -v
ros2 topic info /cmd_vel -v
ros2 run tf2_ros tf2_echo map base_footprint
```

AMCL은 초기 위치를 받기 전 `map→odom`을 만들지 않을 수 있다. 정지 해제는 TF 생성 동작이 아니다. 관제에서 시작점을 지정한 뒤 `위치 재설정(AMCL)`을 수행하면 navigation lifecycle gate가 `map→base_footprint` TF를 최대 0.5초 주기로 확인하고 Nav2 startup을 재시도한다. `map→odom→base_footprint`가 확인되고 `/planner_server`, `/bt_navigator`가 `active`가 되기 전에는 goal을 보내지 않는다. Nav2 controller/recovery가 `/control/nav_velocity`로 출력되고 `/cmd_vel` publisher가 watchdog 하나인지 확인한다. `ros2 topic info -v`에서 publisher/subscriber QoS가 맞지 않으면 먼저 QoS를 수정한다.

Nav2를 아직 준비하지 않고 영상·rosbridge·watchdog만 확인하려면 다음처럼 실행할 수 있다. 이 모드에서는 지도 자동 주행 버튼이 비활성화된다.

```bash
START_NAV2=0 /home/pinky/start-pinky-robot2-session.sh
```

### 6.3 로봇을 들어 옮긴 뒤 다시 시작하는 순서

1. 전체 또는 `robot_2` 정지를 요청하고 `CONFIRMED`/속도 0을 확인한다. 물리적으로도 로봇을 잡을 수 있는 상태인지 확인한다.
2. 로봇을 실제 현장의 새 위치에 놓고, 그 위치에 대응하는 지도 자유 셀을 `시작점 설정`으로 클릭·드래그한다. 우측 상단에서 시험하더라도 모서리 벽 셀 자체가 아니라 조금 안쪽의 바닥 셀을 선택한다.
3. `위치 재설정(AMCL)`을 누른다. 이것은 `/initialpose`를 전달해 AMCL 추정 위치만 바꾸며 `map_260905`를 삭제하거나 다시 그리지 않는다.
4. 지도에 `map→odom→base_footprint` TF가 나타나고 Nav2 planner/BT가 active가 될 때까지 잠시 기다린다. 정지 해제는 이 대기를 대신하지 않는다.
5. 정지 래치가 있으면 `robot_2 정지 해제`를 명시적으로 수행한다. 이전 목표는 자동 재개되지 않는다.
6. 새 `도착점 설정`을 지정하고 `시작점에서 도착점으로 이동`을 누른다.

 시작점·도착점이 점유/미상 셀이면 API가 `MAP_POINT_BLOCKED`로 거부한다. 시험 중 충돌·이상 상황이 다시 발생하면 같은 절차를 반복한다. `stop`은 활성 Nav2 goal을 취소하므로, 정지 해제만으로 로봇이 다시 움직이지 않아야 한다.

## 7. 라인 추종(robot_1) 현장 배선

라인 추종 MVP는 `robot_1`(ROS_DOMAIN_ID `12`)의 전방 바닥 카메라에만 의존한다. 아래는 배선 절차이며, 실제 이동 명령은 현장 담당자가 안전을 확인한 뒤 직접 실행한다. 로봇 측 빌드는 필요 없고 `ros/pinky_camera_pub.py` 단일 파일만 배포한다.

로봇에 camera publisher를 복사한다.

```bash
scp ros/pinky_camera_pub.py pinky@<robot1>:/home/pinky/
```

로봇 shell에서 domain 12로 publisher를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=12 python3 ~/pinky_camera_pub.py --topic /camera/image_raw/compressed --fps 10
```

관제 YAML에서는 `robot_1`에만 카메라 구독을 켠다. `robot_2`는 `camera.enabled: false`를 유지한다.

```yaml
# deployment/robots.ros.yaml (또는 robots.ros.local.yaml)
- robot_id: robot_1
  camera:
    enabled: true
- robot_id: robot_2
  camera:
    enabled: false
```

카메라 publisher를 먼저 켠 뒤 backend를 (재)시작하고, 웹에서 robot_1 카메라 `OK`와 JPEG 응답을 확인한다. 라인을 잃으면 추종 상태가 `LOST`로 바뀌고 소프트웨어 정지(물리 안전 장치가 아님)로 멈추며 자동 재개하지 않는다. 결과와 명령·로그 증거는 [acceptance-report.md](acceptance-report.md)에 기록하며, 로그 증거 없이 PASS로 바꾸지 않는다.
