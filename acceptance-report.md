# T14/T15 통합·인수 기록

작성일: 2026-09-12  ·  기준: `dev`  ·  영상 녹화/재생(T11): 제외

이 문서는 mock에서 재현 가능한 수용 증거와 ROS/Gazebo 현장 검증을 분리해 기록한다. `PASS`는 실행 증거가 저장된 항목, `PARTIAL`은 요구 증거 중 일부만 확인한 항목, `FAIL`은 assertion 또는 계약 위반, `NOT_RUN`은 ROS·하드웨어 전제가 없어 실행하지 않은 항목이다.

## Mock smoke

실행 명령:

```bash
deployment/scripts/acceptance.sh
```

| 시나리오 | 결과 | 증거 |
|---|---|---|
| A01 두 로봇·지도·두 JPEG 카메라 | PASS | `backend/tests/test_t14_acceptance.py::test_a01_mock_dashboard_has_two_robots_map_and_two_camera_streams` |
| A02 목표·편대·도착 | PASS | 기존 `backend/tests/test_t06_mission.py`, `test_t07_patrol.py` |
| A03 추종 준비 거절 | PASS | 기존 `backend/tests/test_t06_mission.py` |
| A04 전체 정지·슬레이브 무응답 | PASS | `test_a04_stop_preserves_offline_robot_as_unconfirmed_candidate` |
| A05 수동 watchdog/포커스·연결 종료 | PASS | 기존 `backend/tests/test_safety.py`, frontend `T05.test.tsx` |
| A06 추종 상실 | PASS | `test_a06_a07_failure_injection_is_scoped_to_formation_or_camera` |
| A07 카메라 한 대 중단 | PASS | 같은 test의 robot_2 `503`, robot_1 `200` assertion |
| A08 중복 request_id | PASS | `test_a08_duplicate_request_id_does_not_create_a_second_command` |
| N09 HTTPS Secure cookie | PASS | `test_n09_secure_cookie_switch_is_explicit` |
| A17 지도 클릭 주행·수동 재배치 후 AMCL 재설정 | PASS | `backend/tests/test_t15_map_navigation.py`, `frontend/src/MapPanel.test.tsx` |

## ROS/Gazebo gate

아래는 T12 어댑터와 현장 장비가 준비될 때 실행한다. 실행 전 `deployment/robots.ros.example.yaml`에 실제 bridge 주소·토픽·타입을 기록하고, 토큰은 환경에서 주입한다.

| 요구사항/시험 | 상태 | 필요한 증거 |
|---|---|---|
| robot_1 domain 12 / bridge 9090 | PASS | 2026-09-14 `192.168.0.8` hardware bringup, PC rosbridge `127.0.0.1:9090`, 플랫폼 `ONLINE` 및 odom·배터리 FRESH, scan/TF 구독 확인. 2026-09-15 PC rosbridge gateway를 `pinky_control_interfaces` 포함 재기동(`/control/status` 역직렬화 복구, Task 5 gap 해소), `192.168.1.201` DDS + PC rosbridge 9090으로 `ONLINE`/odom·배터리 FRESH·`stop_latched true` 확인 |
| robot_2 domain 13 / bridge 9091 | PASS | 2026-09-12 rosbridge client 연결, `/odom`·배터리·TF·카메라 구독 및 웹 실영상 확인. 2026-09-15 `192.168.1.202` DDS + PC rosbridge 9091로 `ONLINE`/odom·배터리 FRESH·`stop_latched true` 확인. 배터리 19% `BATTERY_WARNING` 활성 |
| 두 namespace의 TF 경로와 공통 map 좌표 | NOT_RUN | `tf2_tools view_frames`, 시간 동기 상태 |
| compressed camera topic 매핑·두 스트림·실제 FPS/p95 | PARTIAL | 2026-09-14 robot_2 `/camera/image_raw/compressed` publisher 1개와 플랫폼 JPEG HTTP 200 확인. robot_1 카메라/control workspace 미설치, 두 스트림 장기 FPS/p95 미측정 |
| control/follow 수락·결과·재연결 | NOT_RUN | command_id 로그와 adapter contract test. 2026-09-15 `follow_available false` 재확인(로봇에 `FollowCommand.srv` 없음) — formation은 미지원으로 기록 |
| stop latch·watchdog·최종 cmd_vel 단일 중재 | PARTIAL | 2026-09-15 양쪽 watchdog 부팅 래치(`STOPPED`·`STARTUP_STOP_LATCH`·0속도) 직접 확인, UI 전체/개별 정지·정지해제 `ACCEPTED`·`ACK`, lease 만료 시 양쪽 `SAFETY_STOP ACK` 확인. `/cmd_vel` 단독 publisher 장기 계측·로봇 측 출력 0 직접 샘플은 미실시 |
| robot_2 control/watchdog/navigation 패키지 build | PASS | 2026-09-12 `/home/pinky/dev_ws/wj`에서 3개 패키지 `colcon build` 통과 |
| `/navigate_to_pose` action·AMCL lifecycle·정적 map server | PASS | 2026-09-12 action server 1개, AMCL/map/planner/controller active 및 마지막 goal status 4 `SUCCEEDED` 확인. 2026-09-15 양쪽 로봇 action server 1개씩·AMCL/map `active` 확인, AMCL reset 후 navigation(planner/BT/controller) `active` 전환 및 `map→base_footprint` TF 확인, robot_1 goal status 4 `SUCCEEDED` |
| `map→odom→base_footprint` 및 라이다 costmap 반영 | PARTIAL | 갱신되는 TF와 `/scan` publisher 1개·약 10Hz 확인. 실제 장애물 costmap 반영은 미확인. 2026-09-15 robot_1 AMCL reset 후 `map→base_footprint` TF·map-frame pose FRESH 확인, 양쪽 `/scan` publisher 1개·라이브 확인 |
| 지도 시작점→AMCL→AUTO→목표 저속 주행 | PARTIAL | 실제 이동과 action 성공 확인. 0.25m 허용오차로 0.22m 조기 성공하여 0.08m/0.17rad로 조정. 두 번째 주행은 무관한 로그인 세션 만료가 잘못 발생시킨 안전정지로 취소되어 backend 회귀 테스트를 추가했으며 반복 정밀도 시험 필요. 2026-09-15 robot_1 단일 목표 `(0,0)→(0.4,0)`: backend chain·로봇 측 action status 4 `SUCCEEDED`, 약 9cm 이동 후 정지, 최종 오차 0.29m/0.019rad로 0.08m 위치 기준 미달(조기 성공). 0.15/0.5 clamp 내 저속·정지/복귀 래치 정상. 스캔 환경(개방 공간)과 2.7m 트랙 지도 불일치·초기 covariance 0이 원인 후보로 반복 정밀도 시험 필요 |
| Gazebo 무이동 상태→개별/전체 정지 | NOT_RUN | rosbag/로그, command 결과 |
| 두 Gazebo 인스턴스 spawn 위치 분리 | NOT_RUN | x/y spawn 인자를 지원하는 별도 world 또는 launch 수정. 현재 기본 위치 중첩 가능 |
| 1시간 지도+영상 RSS/큐/N01~N03 | NOT_RUN | 측정값, 호스트 사양, 시계 동기 상태 |

실물에서 제공되지 않은 follow/control 서비스는 `UNSUPPORTED`로 표시하며 PASS로 대체하지 않는다. Nav2 action server 또는 AMCL/TF가 준비되지 않으면 자동 주행 버튼을 사용하지 않는다. 서버 재시작 뒤 자동 주행 재개가 관찰되면 즉시 FAIL로 기록하고 임무를 재개하지 않은 상태에서 원인을 수정한다. 시험 중 로봇을 들어 옮길 때는 정지 확인 후 `localization-reset`을 사용하며, 정적 map 파일을 초기화하지 않는다.

rosbridge는 systemd API 서비스와 별도 lifecycle이다. ROS supervisor가 종료·재시작을 관리하며, API 서비스만 재시작해도 rosbridge가 자동으로 생긴다고 가정하지 않는다.

2026-09-14 두 로봇 동시 연결 smoke에서는 robot_1(`192.168.0.8`, domain 12)과 robot_2(`192.168.0.18`, domain 13)가 동시에 `ONLINE`이고 pose가 `FRESH`로 유지됐으며 두 sensor-layer API에서 scan 표본을 수신했다. 초기 API 표본에서는 양쪽 battery도 `FRESH`였지만 30초 표본 중 robot_2 battery가 `STALE`로 전환되어 갱신 안정성은 미통과다. robot_2 카메라는 `OK` 및 JPEG 11,679 bytes를 반환했고 stop latch는 true로 유지됐다. robot_1은 카메라/control workspace와 로봇 내부 rosbridge가 없어 camera `STALE`, mode `UNKNOWN`, capability 없음이 예상대로 표시됐다. Nav2를 비활성화한 무이동 연결 smoke이므로 양쪽 `MAP_TF_UNVERIFIED`는 미해결 gate로 남긴다. 같은 시점의 ICMP 지연은 robot_1 평균 350ms, robot_2 평균 543ms·최대 1.03초로 원격 주행 안정성 기준에는 부적합했다.

2026-09-15 Nav2 gate(Task 6)에서는 `~/pc_control_ws`에 `pinky_control_interfaces`를 빌드하고 Task 1 rosbridge gateway를 해당 workspace와 함께 재기동하여 Task 5의 `/control/status` 역직렬화 gap을 해소했다. `ros/pinky_control_navigation`은 robot_1에 이미 최신(md5 일치, tolerance 0.08/0.17)과 동일하여 재빌드만 확인하고 robot_2에만 신규 배포·빌드했으며, Nav2 패키지는 양쪽에 설치되어 있어 apt 설치가 불필요했다. 양쪽 watchdog 부팅 래치와 Nav2 action server·localization `active`를 무이동으로 확인한 뒤 backend를 ros 모드(`CONTROL_PLATFORM_WORKERS=1`)로 기동하여 `/api/v1/state`에서 양쪽 `ONLINE`·`stop_latched true`를 확인했다. 활성 지도는 ros adapter의 `apply_settings`가 항상 거부하므로 mock 모드에서 `map_260905`(version 2)로 전환한 뒤 ros 모드로 재기동했다(재시작 후에도 유지됨). robot_1 `localization-reset (0,0)` → TF 유효·`navigate` capability 출현·navigation lifecycle 활성화 → `reset_stop` → 단일 목표 `(0.4,0)` 순서로 저속 주행했으며 lease keepalive(3초 TTL)로 만료를 방지했다. robot_2는 AMCL reset까지만 수행(주행 없음, 배터리 19% `BATTERY_WARNING`)했다. 주행 종료 후 robot_1을 전체 정지로 재래치했고, 이후 양쪽 로봇이 동시에 재부팅되어(23:02 KST, 원인 미상) 테스트 프로세스는 소멸하고 부팅 bringup만 남았다. 재부팅 후 `/cmd_vel`은 무음이며 publisher는 `pinky_dual_bridge_12/_13`뿐으로 동작원이 없어 안전하다. 추가로 확인된 제품 이슈: (1) lease 만료 시 `protective_stop("robot_1")`이 `state_store.disconnected`에 영구 반영되어 backend 재시작 없이는 robot_1 표시가 복구되지 않는다. (2) ros 모드에서 설정 변경이 전면 불가(`apply_settings` 미구현)하여 지도 전환에 재시작이 필요하다. (3) backend가 보내는 `/initialpose` covariance가 36개 0이라 AMCL이 seed에 과확신할 수 있다. 카메라는 Task 4 ruling대로 `NOT_RUN`을 유지한다(`camera.enabled false`, 503 예상).
