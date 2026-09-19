# Pinky Pro 관제 플랫폼 사용자 가이드

이 문서는 현재 구현된 관제 플랫폼을 실행하고 사용하는 방법을 설명한다. 전체 기능을 확인하려면 먼저 mock 모드로 실행한다. ROS 모드는 두 rosbridge 연결과 상태 수신 adapter를 제공하며, robot_2에는 안전 수동 주행용 control watchdog를 별도로 설치할 수 있다. 목표 주행은 Nav2, 정적 점유 지도, AMCL/TF 검증이 끝난 뒤 진행한다.

## 1. 처음 실행하기

### 관리자 계정 만들기

처음 한 번 다음 명령으로 관리자 계정을 만든다. 비밀번호는 예시 문자열을 그대로 사용하지 말고 로컬에서 정한다.

```bash
cd /home/yoon/control-platform/backend
mkdir -p ~/.local/state/control-platform

.venv/bin/python -m pinky_control_center.main \
  --database ~/.local/state/control-platform/control.db \
  --reset-password operator \
  --password '<사용할 비밀번호>' \
  --role ADMIN
```

### 백엔드 실행

첫 번째 터미널에서 mock 백엔드를 실행한다.

```bash
cd /home/yoon/control-platform/backend

.venv/bin/python -m pinky_control_center.main \
  --mode mock \
  --host 127.0.0.1 \
  --port 8081 \
  --database ~/.local/state/control-platform/control.db
```

### 프런트엔드 실행

두 번째 터미널에서 웹 UI를 실행한다.

```bash
cd /home/yoon/control-platform/frontend
npm install
npm run dev
```

브라우저에서 `http://localhost:5173`을 열고 앞에서 만든 계정으로 로그인한다. 상단에 `실시간 연결`이 표시되면 상태 WebSocket이 정상적으로 연결된 것이다.

화면의 오류가 `관제 백엔드에 연결할 수 없습니다`이면 `127.0.0.1:8081` backend와 Vite proxy를 확인한다. `STOP_LATCHED`, TF, 지도 좌표, 명령 시간 초과처럼 구체적인 동작 오류가 표시되면 backend는 응답 중인 것이므로 그 오류 원인을 확인한다. 동작 오류에는 backend 실행 안내가 함께 표시되지 않는다.

## 2. 화면 구성

현재 UI는 한 화면에서 다음 영역을 위에서 아래로 배치한다.

1. 로그인 사용자, 권한, 실시간 연결 상태와 제어권
2. 지도와 두 로봇의 위치
3. 운용 설정과 선택 로봇의 초기 위치
4. 최신 경고와 운용 이력
5. 편대 상태와 편대 제어
6. 목표·순찰 임무 작성과 실행
7. 로봇별 연결·모드·배터리·속도 상태
8. 전체 또는 개별 긴급 정지
9. 두 로봇의 실시간 카메라 그리드
10. 선택 로봇 수동 조작

로봇 카드나 지도 위 로봇 마커를 누르면 `robot_1` 또는 `robot_2`가 선택된다. 선택은 지도 센서 레이어, 초기 위치, 수동 조작 대상에 함께 적용된다.

## 3. 권한과 제어권

| 권한 | 기능 |
|---|---|
| `VIEWER` | 지도, 영상, 상태, 경고, 이력 조회 |
| `OPERATOR` | 제어권 획득, 편대·임무 제어, 긴급 정지 |
| `ADMIN` | OPERATOR 기능과 지도·속도·초기 위치 설정 |

편대와 임무를 조작하려면 상단의 `제어권 획득`을 누른다. 한 번에 한 사용자만 제어권을 가질 수 있으며, 활성 상태에서는 UI가 주기적으로 갱신한다. 작업이 끝나면 `제어권 반납`을 누른다.

긴급 정지는 OPERATOR 또는 ADMIN이면 제어권 없이도 요청할 수 있다. 정지 해제에는 제어권이 필요하다.

## 4. 지도와 목표 설정

지도에는 로봇 위치와 방향, 이동 궤적, 계획 경로와 목표가 표시된다.

- `전체 보기`, `확대`, `축소`, 화살표 버튼으로 화면 범위를 조절한다.
- `선택 따라보기`를 켜면 선택한 로봇을 중심으로 지도가 이동한다.
- `지도`, `Scan`, `Local costmap`, `Global costmap`으로 선택 로봇의 레이어를 전환한다.
- 회색 로봇이나 `위치 지연` 표시는 위치 또는 TF가 유효하지 않다는 뜻이다.
- TF가 유효하지 않으면 편대 거리와 방위각을 표시하지 않는다.

지도 상단의 `시작점 설정` 또는 `도착점 설정` 버튼을 누른 뒤 **클릭한 상태로 드래그하고 놓아서** 지정한다.

- `시작점 설정`은 선택 로봇의 초기 위치 후보를 만든다.
- `도착점 설정`은 목표 위치와 도착 방향을 만든다.
- 처음 누른 지점이 위치이고, 드래그한 방향이 방향이다.
- `설정 초기화`는 시작점·도착점 후보를 모두 지운다.
- 점유 셀이나 알 수 없는 셀에는 목표를 지정할 수 없다.
- 지도 클릭만으로는 로봇을 움직이지 않는다. 단일 로봇 주행은 시작점·방향과 도착점·방향을 모두 지정한 뒤 `시작점에서 도착점으로 이동`을 눌러야 한다.
- `위치 재설정(AMCL)`은 로봇을 손으로 들어 다른 위치에 둔 뒤 새 시작점을 AMCL에 적용하는 버튼이다. 기존 위치나 map TF가 지연된 상태에서도 복구에 사용할 수 있으며, 로봇은 연결된 정지 상태여야 한다. encoder의 정지 잡음은 선속도 0.01 m/s 이하, 각속도 0.03 rad/s 이하까지 0으로 취급한다. 정적 `map_260905` 파일은 지우거나 다시 그리지 않는다.
- 위치 재설정이나 이동 버튼이 비활성화되면 버튼 아래의 `위치 재설정 불가` 또는 `이동 불가` 문구에서 제어권, 연결, 정지, TF 등 준비되지 않은 조건을 확인한다.
- 점유/미상 셀은 시작점·도착점·AMCL 위치로 사용할 수 없다. 지도 우측 상단 모서리 자체는 벽일 수 있으므로 실제 위치와 맞는 안쪽 자유 셀을 선택한다.

한 지점만 지정하면 단일 목표 임무가 되고, 여러 지점을 차례로 지정하면 순찰용 waypoint 목록이 된다.

## 5. 편대 구성

목표 임무를 시작하기 전에 다음 순서로 편대를 준비한다.

1. `제어권 획득`
2. 편대 상태가 `UNPAIRED`인지 확인
3. `편대 구성` 선택
4. 편대 상태가 `READY`로 바뀌는지 확인
5. 지도에서 목표 지정
6. 임무 생성·검증·시작

| 편대 상태 | 의미 |
|---|---|
| `UNPAIRED` | 편대가 구성되지 않음 |
| `READY` | 마스터와 슬레이브가 주행 준비됨 |
| `FOLLOWING` | 슬레이브가 마스터를 추종 중 |
| `PAUSED` | 편대 일시정지 |
| `LOST` | 슬레이브가 마스터를 놓침 |
| `STOPPED` | 보호 정지 또는 사용자 정지 |

`LOST` 상태에서는 자동으로 출발하지 않는다. 원인을 확인한 뒤 `재합류`를 눌러야 한다.

## 6. 목표 주행과 순찰 임무

지도에서 목표를 지정한 뒤 임무 영역에서 다음 순서로 실행한다.

1. 임무 이름과 반복 횟수 입력
2. waypoint 목록과 순서 확인
3. 필요하면 `위`, `아래`, `삭제`로 목록 편집
4. `임무 생성`
5. `검증`
6. 임무 상태가 `READY`가 되면 `시작`

일반적인 임무 상태 흐름은 다음과 같다.

```text
DRAFT → READY → RUNNING → SUCCEEDED
                   ├→ PAUSED → RUNNING
                   └→ CANCELED / FAILED
```

순찰은 여러 waypoint와 2 이상의 반복 횟수를 지정한다. 실행 중에는 현재 waypoint, lap과 진행 거리를 확인할 수 있다.

`시작` 버튼이 비활성화되었다면 제어권, `READY` 편대, 활성 지도, waypoint, 임무 검증 여부를 확인한다.

### 6.1 단일 로봇 지도 주행과 재설정

robot_2 한 대를 시험할 때는 편대를 구성하지 않고 다음 순서로 운용한다.

1. 선택 로봇을 확인하고 `제어권 획득`을 누른다.
2. 로봇이 `ONLINE/FRESH`, 정지 상태이고 `navigate` capability가 표시되는지 확인한다.
3. 로봇을 실제 시작 위치에 둔 뒤 지도에서 `시작점 설정`으로 위치와 전방 방향을 지정한다.
4. 처음 시작하거나 로봇을 들어 옮겼다면 `전체 정지` 또는 해당 로봇 정지를 먼저 확인하고 `위치 재설정(AMCL)`을 누른다. 이 동작은 지도 초기화나 주행을 수행하지 않는다.
5. 정지 래치가 남아 있으면 `robot_2 정지 해제`를 명시적으로 누른다. 이전 Nav2 목표는 자동으로 재개되지 않는다.
6. `도착점 설정`으로 자유 셀의 목표와 방향을 지정하고 `시작점에서 도착점으로 이동`을 누른다.

주행 중 벽에 부딪혔거나 시험을 다시 시작해야 하면 즉시 물리 정지 수단과 UI 정지를 사용하고, 속도가 0인 것을 확인한 뒤 로봇을 들어 안전한 임의 지점으로 옮긴다. 새 시작점 지정 → `위치 재설정(AMCL)` → 정지 해제 → 새 도착점 지정 순서를 다시 따른다. `stop`은 활성 Nav2 goal을 취소하며 통신 복구나 정지 해제만으로 자동 주행하지 않는다.

정확한 우측 상단 모서리는 지도 경계/벽 셀일 수 있다. 화면의 우측 상단에 실제 로봇을 놓더라도 모서리 픽셀에서 조금 안쪽인 자유 셀을 클릭해야 하며, 서버가 `MAP_POINT_BLOCKED`를 반환하면 더 안쪽의 실제 바닥 위치를 선택한다.

## 7. 긴급 정지와 재개

긴급 정지 영역에서 `전체 정지`, `robot_1 정지`, `robot_2 정지`를 선택할 수 있다.

- `CONFIRMED`: 로봇의 정지가 확인됨
- `UNCONFIRMED`: 정지 완료를 확인하지 못함

한 대라도 `UNCONFIRMED`이면 전체 정지가 성공한 것으로 판단하지 않는다. `정지 해제`는 정지 래치만 해제하며 자동 주행을 재개하지 않는다. 편대와 임무 상태를 다시 확인하고 사용자가 명시적으로 재개한다.

## 8. 카메라 그리드

지도 아래에서 MASTER와 SLAVE 카메라를 동시에 확인할 수 있다. 각 타일에는 로봇 이름, 역할, 연결 상태, FPS, 마지막 프레임 경과 시간, 화질과 WebSocket 상태가 표시된다.

- 화질은 `저화질`, `기본`, `고화질` 중 선택한다.
- 2초 이상 새 프레임이 없으면 `영상 지연`으로 표시한다.
- 5초 이상 프레임이 없으면 영상을 숨기고 `영상 없음`을 표시한다.
- 연결이 끊기면 최대 8초 간격으로 자동 재연결한다.

## 9. 운용 설정과 초기 위치

ADMIN은 다음 값을 변경할 수 있다.

- 활성 지도
- 추종 거리와 허용 오차
- 최대 선속도와 최대 각속도
- 기본 카메라 화질

관리자용 `초기 위치 적용`은 기존 설정 API로, 정지·편대 해제·유효한 지도 TF 조건을 확인한다. 현장 시험에서 로봇을 들어 옮긴 뒤 쓰는 `위치 재설정(AMCL)`은 별도 operator API이며 기존 위치/TF 신선도와 무관하게 연결·정지·속도 0 상태에서 stamp 0의 `/initialpose`만 발행한다. 두 동작 모두 자동 주행을 시작하지 않는다.

ROS 모드에서는 배포 설정에 지정된 초기 위치·수동 입력 토픽으로 전달된다. 실물 Pinky 매핑은 `/initialpose`와 `/control/manual_velocity`이며, 시뮬레이터처럼 namespace가 필요한 환경은 설정 파일에서 별도로 지정한다. robot_2 session script는 `pinky_control_watchdog`를 함께 시작해 `/control/manual_velocity`를 제한·감시한 뒤 `/cmd_vel`로 전달한다. 시작 직후에는 정지 래치가 걸리므로 제어권 획득 → 선택 로봇의 `robot_2 정지 해제` → `MANUAL 모드 전환` 순서로 준비한다. `정지 해제`는 자동 주행을 재개하지 않는다.

## 10. 경고와 운용 이력

최신 경고에는 통신 끊김, 위치 지연, TF 오류, 추종 상실, 배터리 위험, 센서 오류와 명령 거절 등이 표시된다. `확인`은 운영자가 경고를 읽었다는 기록이며 원인을 해결하거나 경고를 삭제하지 않는다.

운용 이력은 다음 조건으로 조회할 수 있다.

- 이력 유형
- 로봇
- 임무 ID
- 시작·종료 시간

기본 조회 범위는 최근 24시간이며 이력은 최대 30일 보관한다. 조회 결과는 `JSON 다운로드`로 내보낼 수 있다.

## 11. 수동 조작

수동 조작은 선택한 로봇에 적용된다. 제어권과 로봇의 `MANUAL` 모드가 모두 필요하다.

- `MANUAL 모드 전환`을 누른 뒤 `전진` 또는 `좌회전` 버튼을 누르는 동안 10 Hz로 명령을 보낸다.
- 버튼을 놓거나 포인터가 버튼 밖으로 나가면 0 속도를 보낸다.
- 탭 전환, 브라우저 비활성화, 연결 종료 시 정지한다.

robot_2의 watchdog는 입력이 0.35초 이상 끊기면 `/cmd_vel`에 0을 발행한다. 관제 백엔드도 정상적인 버튼 해제·입력 timeout에는 선택 로봇에 0속도만 전달하므로, 다음 수동 입력마다 정지 해제를 다시 누를 필요가 없다. 웹소켓 단절, lease 만료, 명시적 정지 또는 안전 경보가 발생한 경우에는 보호 정지 래치가 유지되며 다시 `정지 해제` 후 `MANUAL 모드 전환`을 해야 한다. 그래도 실제 시험에서는 긴급 정지 버튼과 로봇 전원 차단 수단을 준비한다. 실물 control interface가 설치되지 않은 환경에서는 모드 전환과 명령이 거절되거나 `UNSUPPORTED`로 표시된다.

## 12. 현재 가능한 시험과 ROS 제한

mock 모드에서는 다음 흐름을 확인할 수 있다.

- 로그인과 권한
- 지도와 로봇 선택
- 편대 구성
- 단일 목표와 순찰 임무
- 전체·개별 정지
- 카메라 그리드와 영상 지연 표시
- 경고 확인
- 설정, 초기 위치와 운용 이력

ROS 모드는 `robot_1=ROS_DOMAIN_ID 12`, `robot_2=ROS_DOMAIN_ID 13`에 각각 연결되는 rosbridge adapter와 상태·배터리·경로 수신을 제공한다. 현재 YYM 현장 프로필은 대역폭 확보를 위해 두 로봇의 카메라 발행과 관제 카메라 구독을 비활성화한다. Domain ID는 UI에서 변경하지 않는다.

YYM Wi-Fi에서 두 대를 동시에 연결할 때의 현재 주소는 `robot_1=172.20.10.9`, `robot_2=172.20.10.8`이다. robot_1은 PC에서 domain 12 rosbridge를 `127.0.0.1:9090`으로 실행하고, robot_2는 로봇 내부 domain 13 rosbridge의 `172.20.10.8:9091`을 사용한다. 두 로봇의 DHCP 주소가 바뀌지 않도록 공유기에서 각 MAC 주소에 대한 DHCP 예약을 설정한다.

Gazebo 또는 실물에서 목표 주행을 시험하려면 다음 외부 연결이 추가로 필요하다.

- 겹치지 않는 위치에 두 로봇을 생성하는 dual-robot launch
- domain 12/13 각각의 `ros_gz_bridge`와 rosbridge
- 로봇별 Nav2와 관제용 control mediator
- 슬레이브 follow controller와 정지 latch/watchdog
- `map → robot_N/odom → robot_N/base_footprint` TF 검증

robot_2 실물 시험의 권장 기동 방법은 systemd user unit 기반 부팅 자동 기동이다. `ros/pinky_control_bringup` 패키지가 하드웨어 bringup(`pinky-bringup@<robot>`)과 관제 세션(rosbridge·watchdog·Nav2, `pinky-session@<robot>`)을 로봇별로 관리하며, 두 unit은 부팅 때 자동으로 시작된다. 세션 unit은 bringup unit에 의존하므로 세션 재시작이 시리얼 장치를 다시 열지 않는다. `ROS_LOCALHOST_ONLY`는 해제되고 `ROS_DOMAIN_ID`는 환경 파일(`robot_1=12`, `robot_2=13`)에서 고정된다.

최초 설치는 로봇에서 한 번만 실행한다. 저장소의 `ros/pinky_control_bringup`을 `/home/pinky/dev_ws/wj/src/` 아래에 복사해 빌드한 뒤 설치 스크립트를 실행한다.

```bash
cd /home/pinky/dev_ws/wj
colcon build --symlink-install --packages-select pinky_control_bringup
./install/pinky_control_bringup/share/pinky_control_bringup/scripts/install.sh robot_2
```

설치가 끝나면 로봇을 재부팅해도 SSH로 무언가를 실행할 필요가 없다. 웹을 새로고침하고 `robot_2`를 선택한다. 재부팅하면 AMCL 추정 위치는 유지되지 않으므로 실제 위치와 방향을 지도에 지정하고 `위치 재설정(AMCL)`을 반드시 한 번 수행한다. 부팅 직후 watchdog는 정지 래치가 걸린 상태이므로 제어권 획득 → `robot_2 정지 해제` → `MANUAL 모드 전환` 순서로 준비한다.

control interface와 watchdog를 처음 설치할 때는 로봇에서 control workspace를 빌드한다. 저장소의 `ros/pinky_control_interfaces`, `ros/pinky_control_watchdog`, `ros/pinky_control_navigation` 디렉터리를 `/home/pinky/dev_ws/wj/src/` 아래에 복사한 뒤 다음을 실행한다.

```bash
cd /home/pinky/dev_ws/wj
colcon build --symlink-install --packages-select pinky_control_interfaces pinky_control_watchdog pinky_control_navigation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

설치 스크립트는 이 workspace가 빌드되어 있어야 통과한다. 하위 session script의 READY 로그에 `control: /control/manual_velocity -> /cmd_vel (watchdog)`가 있어야 한다.

로그와 상태 확인은 `systemctl --user status 'pinky-*'`와 `journalctl --user -u 'pinky-*' -f`로 한다. 수동 기동이 필요한 진단 상황에서는 기존 `deployment/scripts/start-pinky-robot2-*.sh`를 그대로 쓸 수 있지만, 이 스크립트들은 unit과 동시에 실행할 수 없다(중복 프로세스 감지로 실패한다). 실행 중인 로봇은 정지 상태에서 시험한다.

현재 단계의 실물 자동 주행은 robot_2 한 대의 Nav2 action server·AMCL·`map→odom→base_footprint` TF·정적 occupancy map을 확인한 뒤 저속으로 시작한다. 지도 클릭은 명시적 이동 버튼 전까지 주행을 시작하지 않으며, 로봇을 들어 옮긴 뒤에는 정적 지도 대신 AMCL 위치를 재설정한다. ROS 실행과 현장 인수 절차는 프로젝트 루트의 `runbook.md`와 `acceptance-report.md`를 따른다.
