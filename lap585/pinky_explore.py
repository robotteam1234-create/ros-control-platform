#!/usr/bin/env python3
"""
Pinky Pro 자율 탐사 (Frontier Exploration)
============================================
로봇이 스스로 미탐색 영역을 찾아다니며 맵을 완성하고 저장한다.

원리
----
  맵의 각 칸은 세 가지 상태다.
      0  = 비어 있음 (갈 수 있음)
    100  = 장애물
     -1  = 아직 모름

  "비어 있는 칸인데 바로 옆이 아직 모르는 칸" = 프론티어(경계).
  여기로 가면 새로운 정보를 얻는다. 프론티어가 없어지면 탐사 완료.

실행 순서
---------
  1) 로봇 + SLAM + Nav2(SLAM 모드) 를 띄운다
       ros2 launch pinky_bringup bringup_robot.launch.xml
       ros2 launch pinky_navigation map_building.launch.xml
       ros2 launch pinky_navigation bringup_launch.xml slam:=True
       ros2 launch pinky_navigation nav2_view.launch.xml     (선택, 보면서 하면 좋음)

  2) 탐사 시작
       python3 pinky_explore.py --save ~/warehouse_map

  3) 끝나면 맵이 ~/warehouse_map.yaml / .pgm 으로 저장된다

  ※ Gazebo 라면 launch_sim / gz_ 런치를 쓰고 --sim 을 붙일 것
"""

import argparse
import math
import os
import subprocess
import sys
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener

import pinky_mapcheck as mapcheck

# 차체 여유 임계값(m). 시뮬과 실물이 다르므로 환경변수로 받는다.
#
# 실물에서 정상 주행 중 여유를 재보니 중앙값 3.7cm, 하위 10% 가 1.6cm,
# 하위 1% 가 1.4cm 였다(표본 333). 그런데 시뮬 기준으로 잡은 값(닿기 직전
# 1.5cm / 접근 3.0cm / 탈출 목표 5.0cm)을 그대로 쓰니
#   - 정상 주행의 6.3% 가 '닿기 직전' 으로 잡히고
#   - 24.6% 가 '가까워지는 중' 으로 목표를 취소당하고
#   - 탈출 목표 5.0cm 는 90 백분위(4.1cm)보다 커서 영영 도달하지 못했다.
# 그 결과 멀쩡한 로봇이 "끼었다" 며 복귀를 포기했다(실물 394회차).
HARD_GAP = float(os.environ.get('PINKY_HARD_GAP', 0.015))
WATCH_GAP = float(os.environ.get('PINKY_WATCH_GAP', 0.030))
FREE_GAP = float(os.environ.get('PINKY_FREE_GAP', 0.050))


# /map 은 latched(transient_local) 토픽이라 QoS 를 맞춰야 수신된다
MAP_QOS = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_to_quaternion(yaw_rad):
    half = yaw_rad / 2.0
    return math.sin(half), math.cos(half)


class Explorer:

    def __init__(self, navigator, args):
        self.nav = navigator
        self.args = args

        self.grid = None            # numpy 2차원 배열
        self.resolution = None
        self.origin = None          # (x, y)
        self.map_updates = 0

        # 맵 좌표계가 무너졌는지. 무너진 맵 위에서 계속 주행하면
        # 맵만 더 망가진다(316회차: 벽에 붙어 제자리 회전하다 좌표계가
        # 끌려갔고, 그 뒤 목표마다 '남은 거리 0.00 m' 만 반복했다).
        self.diverged = False
        self._health_t = 0.0
        self.home_blocked = False   # 복귀 목표를 코스트맵에서 못 찾았는가
        self.nav2_broken = False    # Nav2 가 목표를 즉시 거부하는 상태인가
        self.last_goal = None       # 직전에 고른 프론티어(근처부터 훑기 위해)
        self._prev_long = None      # 직전에 잰 맵의 긴 변(갑자기 커짐 판정용)

        self.blacklist = []         # 실패한 목표 좌표들
        self.visited = []           # 다녀온 목표 좌표들
        self.fail_counts = {}       # 목표별 실패 횟수
        self.legs = []              # 구간별 결과
        self.home = None            # 출발 지점 (x, y, yaw) — 탐사 후 복귀용

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, navigator)
        navigator.create_subscription(
            OccupancyGrid, '/map', self._map_cb, MAP_QOS)
        self._cmd_vel_pub = navigator.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_front = None
        self.scan_msg = None      # 차체 기준 여유 계산용 원본
        self.scan_back = None
        navigator.create_subscription(LaserScan, 'scan', self._scan_cb, 10)

        self.costmap = None         # 글로벌 코스트맵 (플래너가 실제로 보는 것)
        navigator.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, MAP_QOS)

    def _costmap_cb(self, msg):
        self.costmap = msg

    def nearest_navigable(self, x, y, max_radius=0.6):
        """(x, y) 에 가장 가까우면서 플래너가 들어갈 수 있는 지점을 찾는다.

        좁은 통로에서는 목표 지점이 인플레이션 때문에 비용 99(inscribed)로
        둘러싸여 NavFn 이 "Failed to create plan" 을 내는 일이 잦다.
        그럴 때 근처의 통행 가능한 칸으로 대신 보내고, 마지막 몇 cm 는
        precise_approach 가 코스트맵과 무관하게 직접 좁힌다."""
        cm = self.costmap
        if cm is None:
            return x, y
        res = cm.info.resolution
        ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
        w, h = cm.info.width, cm.info.height
        data = cm.data

        def cost_at(px, py):
            c = int((px - ox) / res)
            r = int((py - oy) / res)
            if 0 <= r < h and 0 <= c < w:
                return data[r * w + c]
            return None

        # 주의: cost 0(완전한 빈칸)은 falsy 라서 `cost_at(...) or 100` 처럼
        # 쓰면 막힌 것으로 오판한다. None 인지 명시적으로 검사해야 한다.
        here = cost_at(x, y)
        if here is not None and here < 99:
            self.home_blocked = False      # 목표 자체가 갈 수 있는 자리다
            return x, y
        steps = int(max_radius / res)
        for ring in range(1, steps + 1):
            best = None
            for dr in range(-ring, ring + 1):
                for dc in range(-ring, ring + 1):
                    if max(abs(dr), abs(dc)) != ring:
                        continue
                    nx, ny = x + dc * res, y + dr * res
                    c = cost_at(nx, ny)
                    if c is not None and c < 99 and self._clear_line(nx, ny, x, y):
                        d = math.hypot(nx - x, ny - y)
                        if best is None or d < best[0]:
                            best = (d, nx, ny)
            if best:
                print(f'      출발지가 막혀 있어 {best[0]:.2f} m 옆의 '
                      f'진입 가능한 지점을 목표로 삼습니다')
                self.home_blocked = False
                return best[1], best[2]
        # 반경 안에서 못 찾았다. 부르는 쪽이 '목표가 원래 갈 수 있는 자리라
        # 그대로 돌려준 것' 과 구분할 수 있어야 한다. 예전에는 둘 다 (x,y)
        # 를 돌려줘서, 멀쩡한 목표에도 '갈 수 있는 자리가 없습니다' 를
        # 찍고 반경을 넓히느라 시간을 버렸다(340회차).
        self.home_blocked = True
        return x, y

    def _clear_line(self, x0, y0, x1, y1):
        """두 점 사이에 벽이 없는지 SLAM 맵으로 확인한다.
        이게 없으면 '3cm 옆'이 사실은 벽 반대편이라 엉뚱한 곳으로 간다."""
        if self.grid is None:
            return True
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / self.resolution) + 1)
        for i in range(n + 1):
            t = i / n
            if self.cell_value(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t) == 100:
                return False
        return True

    def _scan_cb(self, msg):
        """앞/뒤 최단 거리. 이 로봇의 rplidar_link 는 base_link 기준 yaw 180도로
        장착돼 있어(URDF/TF), 스캔각 0 이 로봇의 '뒤', ±180도가 '앞' 이다."""
        self.scan_msg = msg        # 차체 기준 여유 계산용 원본
        front = back = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min * 1.02:
                continue
            a = msg.angle_min + i * msg.angle_increment
            if abs(a) >= math.radians(160):
                front = min(front, r)
            elif abs(a) <= math.radians(20):
                back = min(back, r)
        self.scan_front = None if front == float('inf') else front
        self.scan_back = None if back == float('inf') else back

    def back_off(self, distance=0.15, speed=0.06, min_room=None):
        """벽에 바싹 붙어 멈추면 Nav2 는 제자리 회전조차 거부한다
        ("Collision Ahead - Exiting Spin"). 그러면 복구 동작이 전부 실패해
        영영 못 움직인다. 라이다로 뒤가 비었는지 직접 확인하고 후진시켜
        코스트맵의 치명 구역에서 빼낸다.

        주의: scan_back 은 '라이다에서 잰 거리' 이고, 정작 필요한 것은
        '차체 뒷면에서 벽까지' 다. 그 차이가 7.4cm 다(차체 뒤쪽 반경).
        예전에는 raw 거리 0.15 m 를 기준으로 삼아, 뒤에 3cm 나 여유가
        있는데도 "후진할 수 없습니다" 로 거부했다. 그래서 벽에 낀 로봇이
        빠져나올 유일한 수단을 스스로 막고 있었다(354회차 접촉)."""
        from pinky_wallfollow import body_reach
        min_room = HARD_GAP if min_room is None else min_room
        for _ in range(10):
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            if self.scan_back is not None:
                break
        if self.scan_back is None:
            return False
        rear = self.scan_back - body_reach(0.0)     # 뒤쪽 차체 여유
        if rear < min_room:
            print(f'      뒤쪽 차체 여유가 {rear*100:+.1f}cm 뿐이라 '
                  f'후진할 수 없습니다')
            return False
        distance = max(0.02, min(distance, rear - 0.005))

        twist = Twist()
        twist.linear.x = -abs(speed)
        t0 = time.time()
        moved = 0.0
        g0 = self.body_clearance()
        start_gap = g0 if g0 is not None else 0.0
        while rclpy.ok() and time.time() - t0 < distance / speed:
            if self.scan_back is not None and \
                    self.scan_back - body_reach(0.0) < 0.008:
                break
            # 뒤쪽 부채꼴(±20도)만 보면 대각선 벽을 못 본다. 물러나다
            # 옆구리가 벽에 닿는 일이 실제로 있었다(363회차: 16cm 물러난
            # 직후 다음 목표 시작 0.1초 만에 -0.8cm). 차체 전체 여유로 본다.
            # 절대 기준(2cm 미만이면 정지)으로 두면 안 된다. 이 함수는
            # 애초에 낀 상태에서 부르는 것이라 시작부터 여유가 1cm 인
            # 경우가 많고, 그러면 한 발도 못 움직이고 끝난다(실물 394회차:
            # '0cm 물러났습니다' 만 반복하다 복귀 포기). '지금보다
            # 나빠지면 멈춘다' 로 본다 — 빠져나오는 방향이면 여유가 는다.
            g = self.body_clearance()
            if g is not None and (g < start_gap - 0.004 or g < 0.002):
                break
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.nav, timeout_sec=0.05)
            moved = (time.time() - t0) * speed
        self._cmd_vel_pub.publish(Twist())
        print(f'      벽에서 {moved*100:.0f}cm 물러났습니다')
        return moved > 0.005

    def too_close(self, state, hard=None, watch=None, drop=0.004):
        """지금 목표를 접고 물러나야 하는가. (여유, 이유) 를 돌려준다.

        고정 기준 하나로는 안 된다. 1.5cm 는 이미 닿기 직전이라 감지하고
        멈추는 사이에 닿았고(350회차 접촉), 그렇다고 2.5cm 로 올리니 좁은
        통로를 정상 통과할 때도 취소가 쏟아졌다(351회차 5회, 재현율 하락).

        그래서 '가깝다' 와 '가까워지고 있다' 를 함께 본다. 3cm 라도 여유가
        줄고 있으면 접근 중이니 멈추고, 3cm 를 유지하며 지나가는 중이면
        그대로 둔다.

        state 는 부르는 쪽이 들고 있는 dict 다(빈 dict 로 시작)."""
        hard = HARD_GAP if hard is None else hard
        watch = WATCH_GAP if watch is None else watch
        # 속도에 비례해 기준을 키우는 방법을 시도했다가 되돌렸다.
        # body_clearance() 는 '차체 사방 중 최솟값' 이라, 폭 30cm 통로에서는
        # 늘 옆벽이 그 값을 지배한다(실측 중앙값 3.7cm). 거기에 속도분(10cm/s
        # 에서 6cm)을 더하면 정상 주행 내내 정지 판정이 난다. 방향을 가리지
        # 않는 이 값에는 속도 보정을 붙일 수 없다.
        # 돌진은 다른 방법으로 막는다: 통로에서는 애초에 느리게 가고
        # (desired_linear_vel 0.10), Nav2 자체 충돌 검사를 켜 둔다.
        gap = self.body_clearance()
        if gap is None:
            return None, None
        now = time.time()
        prev, t_prev = state.get('gap'), state.get('t', 0.0)
        state['gap'], state['t'] = gap, now
        if gap < hard:
            return gap, '벽에 닿기 직전'
        if (gap < watch and prev is not None and now - t_prev < 0.8
                and prev - gap >= drop):
            return gap, '벽으로 가까워지는 중'
        return gap, None

    def _open_side(self):
        """어느 쪽으로 돌아야 트이는가. +1 왼쪽, -1 오른쪽.

        스캔에서 좌우 절반의 '차체 여유' 합을 비교한다. 트여 있는 쪽이
        합이 크다. 각도 규약(스캔 0도가 로봇 뒤)에 의존하지 않도록,
        차체 여유를 기준으로 좌/우만 가른다."""
        m = self.scan_msg
        if m is None:
            return 1.0
        try:
            from pinky_wallfollow import WallFollower, body_reach
            import numpy as _np
            r = WallFollower._denoise(m, win_deg=5.0)
            ok = _np.isfinite(r)
            if not ok.any():
                return 1.0
            ang = _np.degrees(m.angle_min
                              + _np.arange(len(r)) * m.angle_increment)
            ang = (ang + 360.0) % 360.0 - 180.0
            gaps = r[ok] - _np.array([body_reach(a) for a in ang[ok]])
            a = ang[ok]
            left = gaps[(a > 20) & (a < 160)]
            right = gaps[(a < -20) & (a > -160)]
            if left.size and right.size:
                return 1.0 if left.mean() > right.mean() else -1.0
        except Exception:
            pass
        return 1.0

    def free_self(self, timeout=12.0, want=None):
        """몸이 벽에 낀 상태를 스스로 푼다. 풀렸으면 True.

        순서가 중요하다. **곧게 움직이는 것이 먼저이고 회전은 마지막**이다.
        낀 채로 제자리 회전하면 차체 대각 반경(8.5cm)이 반폭(6.35cm)보다
        크기 때문에 돌수록 더 파고든다. 실물 396회차 계측에 그대로 찍혔다:

            여유 0.1cm, 회전 0.6 -> 여유 -0.5cm -> 접촉
            그 뒤 후진(-0.06) 으로 겨우 1.8cm 확보

        곧은 이동은 휩쓰는 면적이 없어 안전하다. 어느 쪽으로 갈지는 기하로
        따지지 말고 '움직여 보고 여유가 늘어나는 쪽' 을 쓴다."""
        want = FREE_GAP if want is None else want
        gap0 = self.body_clearance()
        if gap0 is None or gap0 >= want:
            return True
        print(f'      몸이 벽에 끼었습니다 (여유 {gap0*100:.1f}cm). '
              f'곧게 빠져나옵니다.')
        twist = Twist()

        def creep(lin, name, seconds):
            """곧게 조금 움직이며 여유를 지켜본다. 나아지면 True."""
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < seconds:
                twist.linear.x = lin
                twist.angular.z = 0.0
                self._cmd_vel_pub.publish(twist)
                rclpy.spin_once(self.nav, timeout_sec=0.1)
                g = self.body_clearance()
                if g is None:
                    continue
                if g >= want or g >= gap0 + 0.015:
                    self._cmd_vel_pub.publish(Twist())
                    print(f'        {name} 움직여 여유 {g*100:.1f}cm 확보')
                    return True
                if g < gap0 - 0.005:      # 그 방향은 더 파고든다
                    break
            self._cmd_vel_pub.publish(Twist())
            return False

        # 1) 곧게 뒤로, 안 되면 곧게 앞으로 (휩쓰는 면적이 없다).
        #    2초(8cm)로는 코너에서 빠져나오기에 모자랐다. 3.5초면 14cm.
        for lin, name in ((-0.04, '뒤로'), (0.04, '앞으로')):
            if creep(lin, name, 3.5):
                return True

        # 2) 그래도 안 되면 회전. 단 이미 닿아 있으면(여유 0 이하) 돌지
        #    않는다 — 돌수록 더 파고든다.
        gap_now = self.body_clearance()
        if gap_now is not None and gap_now <= 0.0:
            print('        몸이 벽에 닿아 있어 회전은 쓰지 않습니다')
            return False
        # 회전 방향은 '트인 쪽' 을 재서 한 번만 정하고, 그쪽으로 끝까지
        # 돈다. 1.5초 돌려 보고 나아지지 않으면 반대로 트는 방식은,
        # 모서리를 쓸며 여유가 잠깐 줄어드는 정상 과정에서도 방향을 바꿔
        # 좌우로 오가다 끝난다(실물에서 '오른쪽으로 더 돌면 빠지는데
        # 왼쪽으로 되돌아간다' 로 관찰됨).
        sign = self._open_side()
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            twist.linear.x = 0.0
            twist.angular.z = sign * 0.4
            self._cmd_vel_pub.publish(twist)
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            g = self.body_clearance()
            if g is None:
                continue
            if g >= want or g >= gap0 + 0.015:
                self._cmd_vel_pub.publish(Twist())
                print(f'        {"왼" if sign > 0 else "오른"}쪽으로 돌려 '
                      f'여유 {g*100:.1f}cm 확보')
                return True
            if g <= 0.0:
                break            # 실제로 닿았을 때만 접는다
        self._cmd_vel_pub.publish(Twist())

        g = self.body_clearance()
        print(f'        못 빼냈습니다 (여유 '
              f'{"?" if g is None else f"{g*100:.1f}cm"})')
        return False

    # ------------------------------------------------------------------
    def nudge_forward(self, distance=0.3, speed=0.1, stop_gap=0.04):
        """맵 경계 바로 안쪽 벽에 붙어서 출발하면 SLAM 맵이 로봇이
        서 있는 지점을 아직 못 덮어서(planner 가 "Start Coordinates ...
        was outside bounds" 로 즉시 실패) 첫 목표가 실패할 수 있다.
        시작할 때 살짝 움직여 이미 알려진 영역 안쪽으로 옮겨둔다.

        예전에는 라이다를 한 번도 보지 않고 30cm 를 전진했다. 앞 단계가
        끝난 자리는 대개 프론티어(=아직 안 본 곳) 쪽을 보고 있어서, 그
        방향은 벽인 경우가 많다. '프론티어 탐사 2회차가 시작될 때 벽으로
        돌진' 하던 증상이 이것이다. 여유를 보며 움직이고, 앞이 막혔으면
        뒤로 물러난다(라이다는 360도를 보므로 뒤로 가도 된다)."""
        twist = Twist()
        for direction, name in ((1.0, '앞쪽'), (-1.0, '뒤쪽')):
            t0 = time.time()
            gap = None
            blocked = False
            while rclpy.ok() and time.time() - t0 < distance / speed:
                gap = self.body_clearance()
                if gap is not None and gap < stop_gap:
                    blocked = True
                    break
                twist.linear.x = direction * speed
                self._cmd_vel_pub.publish(twist)
                rclpy.spin_once(self.nav, timeout_sec=0.1)
            self._cmd_vel_pub.publish(Twist())
            if not blocked:
                return True
            print(f'      {name}이 막혀 있습니다 '
                  f'(차체 여유 {gap * 100:.1f} cm)')
        return False

    # ------------------------------------------------------------------
    # 수신
    # ------------------------------------------------------------------
    def map_broken(self, period=6.0):
        """맵 좌표계가 무너졌으면 True.

        1단계(벽 따라가기)는 8초마다 스스로 이 검사를 했지만 2·4단계에는
        아무 검사가 없었다. 그래서 탐사 중에 맵이 틀어져도 아무도 모른 채
        Nav2 목표를 계속 쏴 맵을 끝까지 망가뜨렸다."""
        if self.diverged:
            return True
        now = time.time()
        if now - self._health_t < period:
            return False
        gap_t = now - self._health_t if self._health_t else 0.0
        self._health_t = now
        if self.grid is None or not self.resolution:
            return False
        try:
            h = mapcheck.health(self.grid, self.resolution)
        except Exception:
            return False
        # 공간 크기를 모르면(실물) 절대 크기로는 판정할 수 없다. 대신 맵이
        # 갑자기 몇 배로 늘어나는지를 본다 — 스캔 매칭이 어긋날 때의 신호다.
        long_side = h['size_m'][0]
        if not h['diverged'] and mapcheck.jumped(self._prev_long, long_side,
                                                 gap_t):
            h = dict(h)
            h['diverged'] = True
            h['reasons'] = [f'맵이 {gap_t:.0f}초 만에 '
                            f'{self._prev_long:.2f} -> {long_side:.2f} m 로 '
                            f'갑자기 커짐 — 좌표계 틀어짐']
        self._prev_long = long_side
        if not h['diverged']:
            return False
        self.diverged = True
        print(f"\n맵 좌표계가 무너졌습니다 ({'; '.join(h['reasons'])}). "
              f"이 맵 위에서 더 주행해도 나빠지기만 하므로 중단합니다.")
        return True

    def _map_cb(self, msg):
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x,
                       msg.info.origin.position.y)
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.map_updates += 1

    def _robot_in_bounds(self):
        """로봇의 현재 위치가 지금 받은 맵 범위 안에 있는지 확인한다.
        SLAM 맵은 몇 초 간격으로만 갱신되므로, 막 시작했을 때는
        로봇이 서 있는 지점이 아직 맵 경계 밖일 수 있다 (planner_server 가
        "Start Coordinates ... was outside bounds" 로 즉시 실패하는 원인)."""
        pose = self.robot_xy()
        if pose is None or self.grid is None:
            return False
        rx, ry = pose
        h, w = self.grid.shape
        x0, y0 = self.origin
        x1 = x0 + w * self.resolution
        y1 = y0 + h * self.resolution
        return x0 <= rx <= x1 and y0 <= ry <= y1

    def wait_for_map(self, timeout=30.0):
        # 맵 서버(slam_toolbox)의 첫 발행분은 스캔 한 번 분량이라 아직
        # 미탐색 경계(-1)가 제대로 안 잡혀 있을 수 있다. 최소 2번은
        # 받아야 프론티어 계산이 의미가 있다. 또한 로봇이 서 있는 지점이
        # 맵 범위 안에 들어올 때까지 기다린다.
        deadline = time.time() + timeout
        while rclpy.ok() and (self.map_updates < 2 or not self._robot_in_bounds()):
            rclpy.spin_once(self.nav, timeout_sec=0.2)
            if time.time() > deadline:
                if self.grid is None:
                    raise RuntimeError(
                        '/map 을 받지 못했습니다. SLAM 이 실행 중인지 확인하세요.')
                print('  경고: 로봇 위치가 아직 맵 범위 밖일 수 있습니다. 계속 진행합니다.')
                break
        h, w = self.grid.shape
        print(f'맵 수신: {w} x {h} 칸, 해상도 {self.resolution:.3f} m/칸')

    def robot_xy(self):
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def robot_pose(self):
        """(x, y, yaw라디안). TF 를 아직 못 읽었으면 None"""
        try:
            tf = self.buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    # ------------------------------------------------------------------
    # 좌표 변환
    # ------------------------------------------------------------------
    def cell_to_world(self, row, col):
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (row + 0.5) * self.resolution
        return x, y

    def cell_value(self, x, y):
        """월드 좌표가 속한 맵 칸의 값. 맵 범위 밖이면 None"""
        if self.grid is None:
            return None
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        h, w = self.grid.shape
        if 0 <= row < h and 0 <= col < w:
            return int(self.grid[row, col])
        return None

    def retract_goal(self, x, y, rx, ry):
        """프론티어 칸은 정의상 미탐색 영역과 맞닿아 있어서, 그 자리를 그대로
        목표로 주면 코스트맵 가장자리라 planner 가 즉시 실패한다
        ("worldToMap failed", "Goal outside bounds"). 로봇 쪽으로 조금 당겨
        확실히 알려진 빈 칸을 목표로 삼는다. 로봇이 그만큼 가까이 가면
        라이다가 어차피 그 너머를 보게 되므로 탐사 효과는 같다."""
        d = math.hypot(x - rx, y - ry)
        if d < 1e-6:
            return x, y
        back = min(self.args.goal_retract, d - self.resolution)
        while back > 0:
            nx = x + (rx - x) / d * back
            ny = y + (ry - y) / d * back
            if self.cell_value(nx, ny) == 0:
                return nx, ny
            back -= self.resolution
        return x, y

    # ------------------------------------------------------------------
    # 프론티어 탐색
    # ------------------------------------------------------------------
    def find_frontiers(self):
        """프론티어 덩어리 목록을 돌려준다: [(x, y, 크기), ...]"""
        grid = self.grid
        free = (grid == 0)
        unknown = (grid == -1)

        # 상하좌우 중 하나라도 '모름' 과 맞닿은 '비어있음' 칸
        touching = np.zeros_like(unknown)
        touching[1:, :] |= unknown[:-1, :]
        touching[:-1, :] |= unknown[1:, :]
        touching[:, 1:] |= unknown[:, :-1]
        touching[:, :-1] |= unknown[:, 1:]
        frontier = free & touching

        rows, cols = np.nonzero(frontier)
        cells = set(zip(rows.tolist(), cols.tolist()))

        clusters = []
        while cells:
            seed = cells.pop()
            group = [seed]
            queue = deque([seed])
            while queue:
                r, c = queue.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in cells:
                            cells.discard(n)
                            group.append(n)
                            queue.append(n)
            if len(group) >= self.args.min_frontier:
                clusters.append(group)

        result = []
        for group in clusters:
            arr = np.array(group)
            cr, cc = arr[:, 0].mean(), arr[:, 1].mean()
            # 중심에 가장 가까운 실제 프론티어 칸을 목표로 삼는다
            d = (arr[:, 0] - cr) ** 2 + (arr[:, 1] - cc) ** 2
            r, c = arr[int(np.argmin(d))]
            x, y = self.cell_to_world(r, c)
            result.append((x, y, len(group)))
        return result

    def is_blacklisted(self, x, y):
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < self.args.blacklist_radius:
                return True
        return False

    def is_visited(self, x, y):
        """이미 다녀온 지점인지. 도착했는데도 프론티어가 그대로 남아 있으면
        (센서 사각 등으로) 다시 가봐야 소용없다. 안 걸러내면 같은 목표를
        무한히 다시 고른다."""
        for vx, vy in self.visited:
            if math.hypot(x - vx, y - vy) < self.args.blacklist_radius:
                return True
        return False

    def choose_goal(self, frontiers, rx, ry):
        """점수가 가장 높은 프론티어를 고른다. 크면 좋고, 가까우면 좋고,
        지금 향한 쪽이면 좋다.

        방향을 안 보면 '왔던 길을 또 가는' 일이 생긴다. 목표에 도착한 순간
        가장 가까운 프론티어가 방금 지나온 쪽에 있는 경우가 많아, 앞뒤로
        오가느라 시간을 버린다(넓은 공간일수록 심하다). 뒤로 돌아가는
        목표에 벌점을 주면 같은 조건일 때 앞쪽을 먼저 훑는다. 벌점은
        '금지' 가 아니라 우선순위다 — 뒤쪽에 큰 프론티어만 남으면 간다."""
        yaw = None
        pose = self.robot_pose()
        if pose is not None:
            yaw = pose[2]
        best = None
        best_score = -1.0
        for x, y, size in frontiers:
            if self.is_blacklisted(x, y) or self.is_visited(x, y):
                continue
            dist = math.hypot(x - rx, y - ry)
            if dist < self.args.min_distance:
                continue
            # 거리를 제곱으로 본다. 1차식은 '멀리 있는 큰 덩어리' 가 늘
            # 이겨서, 한쪽을 끝내지 않고 반대편으로 건너뛰었다가 되돌아온다
            # (실측 406회차: 목표 전환의 24% 가 1m 넘는 도약, 트랙 길이가
            # 2.71m 인데 목표 사이 총 이동이 12.4m).
            score = size / (1.0 + dist) ** 2
            # 직전에 간 자리 근처면 점수를 올린다. '있는 자리부터 훑고
            # 옮긴다' 는 규칙이다. 사람이 방을 치우는 순서와 같다.
            if self.last_goal is not None and self.args.cluster > 0:
                if math.dist((x, y), self.last_goal) < self.args.cluster:
                    score *= 1.0 + self.args.cluster_bonus
            penalty = getattr(self.args, 'back_penalty', 0.0)
            if yaw is not None and penalty and dist > 0.3:
                # 로봇이 보는 방향과 목표 방향의 각도차. 뒤(180도)면 최대 벌점.
                head = math.atan2(y - ry, x - rx)
                turn = abs((head - yaw + math.pi) % (2 * math.pi) - math.pi)
                score *= 1.0 - penalty * (turn / math.pi)
            if score > best_score:
                best_score = score
                best = (x, y, size, dist)
        return best

    # ------------------------------------------------------------------
    # 이동
    # ------------------------------------------------------------------
    def go_to(self, x, y, rx, ry):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        qz, qw = yaw_to_quaternion(math.atan2(y - ry, x - rx))
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        t0 = time.time()
        self.nav.goToPose(goal)

        last_print = 0.0
        stall = {}
        near = {}
        while not self.nav.isTaskComplete():
            if self.nav_stalled(stall):
                self.nav.cancelTask()
                print('      Nav2 가 12초째 로봇을 못 움직입니다 — '
                      '직접 빠져나옵니다')
                # Nav2 가 얼어붙은 자리는 대개 코너다. 우리 탈출 동작은
                # 코스트맵이 아니라 라이다를 보므로 여기서 통한다.
                if not self.back_off():
                    self.free_self()
                break
            if time.time() - last_print >= 3.0:
                last_print = time.time()
                fb = self.nav.getFeedback()
                if fb:
                    print(f'      남은 거리 {fb.distance_remaining:.2f} m')
            # 맵이 틀어지면 벽 너머에 빈 공간이 칠해지고, 프론티어 탐사가
            # 그리로 달려간다. Nav2 는 그 맵을 믿으므로 그대로 벽에 박는다
            # (201회차에서 왼쪽 바깥벽을 정면으로 밀었다).
            # 맵이 뭐라고 하든 라이다로 잰 차체 여유가 없으면 멈춘다.
            if self.map_broken():
                self.nav.cancelTask()
                break
            gap, why = self.too_close(near)
            if why:
                self.nav.cancelTask()
                print(f'      차체 여유 {gap*100:+.1f}cm ({why}) — '
                      f'목표를 취소하고 물러납니다')
                if not self.back_off():
                    self.free_self()
                break
            if time.time() - t0 > self.args.goal_timeout:
                self.nav.cancelTask()
                print(f'      구간 시간 초과({self.args.goal_timeout:.0f}초)')
                break

        elapsed = time.time() - t0
        ok = self.nav.getResult() == TaskResult.SUCCEEDED
        self.legs.append({'x': x, 'y': y, 'ok': ok, 'seconds': elapsed})
        return ok, elapsed

    # ------------------------------------------------------------------
    def nav_stalled(self, state, threshold=12.0):
        """Nav2 가 목표를 들고 있으면서 아무 일도 안 하는 상태인가.

        두 가지 모습이 있는데 겉보기 신호는 다르지만 결과는 같다.
          (1) 경로를 못 짬 -> 피드백이 '남은 거리 0.00 m' 로 고정
          (2) 경로는 있는데 컨트롤러가 0 속도 -> 남은 거리가 그대로 멈춤
              (실물 계측 22:33: 남은 거리 0.24m 인 채 24초간 정지.
               RPP 의 자체 충돌 검사가 좁은 통로에서 못 간다고 판단한다)

        그래서 '거리' 가 아니라 **로봇이 실제로 움직였는가** 로 본다.
        Nav2 가 제자리 회전으로 방향을 맞추는 동안에도 잠시 안 움직이므로
        12초는 기다린다.

        state 는 호출하는 쪽이 들고 있는 dict 다(빈 dict 로 시작)."""
        now = time.time()
        pose = self.robot_pose()
        fb = self.nav.getFeedback()
        left = fb.distance_remaining if fb is not None else None
        moved = 1.0
        if pose and state.get('ref'):
            moved = math.hypot(pose[0] - state['ref'][0],
                               pose[1] - state['ref'][1])
        # 판정 기준은 '움직였는가' 가 아니라 **목표에 가까워졌는가** 다.
        # 코너에서는 로봇이 2~20cm 씩 찔끔찔끔 움직이며 제자리를 맴돈다.
        # '움직임' 을 기준으로 삼으면 그때마다 타이머가 초기화돼, 실물에서
        # 60초를 헤맨 뒤에야 감지됐다(22:38:13~22:39:13).
        # 조금이라도 가까워지고 있으면 정상, 아니면 시간을 센다.
        closing = (left is not None and state.get('best') is not None
                   and left < state['best'] - 0.03)
        if left is not None:
            state['best'] = min(state.get('best', left), left)
        if closing or moved > 0.25:      # 크게 움직였으면 상황이 바뀐 것
            state.pop('since', None)
            if pose:
                state['ref'] = pose
            return False
        state.setdefault('since', now)
        if pose and 'ref' not in state:
            state['ref'] = pose
        return now - state['since'] > threshold

    def body_clearance(self):
        """차체 표면에서 가장 가까운 벽까지 남은 여유(m). 못 재면 None.

        미세 접근은 코스트맵을 무시하고 목표로 직진한다. 그런데 안전 검사가
        '정면 빔 10cm' 하나뿐이라 옆구리와 대각선은 보지 못했다. 목표가
        벽 뒤에 있으면(루프 클로저로 옛 좌표가 어긋나면 그렇게 된다)
        40초 동안 벽을 밀게 된다. 실제로 탐색이 끝난 뒤 모서리에서 벽으로
        돌진하는 증상이 이것이었다. 벽 따라가기와 같은 기준으로 본다."""
        # 안전 판정이라 좁은 창(1.7도)을 쓴다. 넓은 창은 벽 끝처럼 얇은
        # 것을 지워버려 모서리에 그대로 긁힌다(201·202회차에서 확인).
        try:
            from pinky_wallfollow import body_gap
        except Exception:
            return None
        return body_gap(self.scan_msg, win_deg=1.7)

    def precise_approach(self, hx, hy, hyaw, timeout=40.0):
        """Nav2 컨트롤러는 xy_goal_tolerance 안에 들어오면 그대로 멈춰서
        마지막 몇 cm 를 못 좁힌다. TF 로 현재 위치를 직접 보며 cmd_vel 로
        미세 접근하고, 마지막에 출발 당시 방향까지 맞춘다.
        REAL: PINKY_APPROACH_MAX로 상한 조절 (sim 0.08, real 0.05 권장)."""
        import os as _os
        vmax = float(_os.environ.get('PINKY_APPROACH_MAX', 0.08))
        twist = Twist()
        t0 = time.time()

        # 1) 위치 맞추기
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self.nav, timeout_sec=0.05)
            pose = self.robot_pose()
            if pose is None:
                continue
            x, y, yaw = pose
            dx, dy = hx - x, hy - y
            dist = math.hypot(dx, dy)
            if dist <= self.args.home_tolerance:
                break
            ang = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(ang) > math.radians(25):
                twist.linear.x = 0.0                      # 먼저 고개부터 돌린다
            elif self.scan_front is not None and self.scan_front < 0.10:
                # 회전 중에는 정면이 막혀도 상관없다. 전진하려는 순간에만 막는다.
                print('      정면이 막혀 미세 접근을 멈춥니다')
                break
            elif (gap := self.body_clearance()) is not None and gap < 0.02:
                # 정면만 보면 옆구리와 대각선을 놓친다. 차체 어느 쪽이든
                # 벽에 닿기 직전이면 더 밀지 않는다.
                print(f'      차체 여유 {gap*100:+.1f}cm — 미세 접근을 멈춥니다')
                break
            else:
                twist.linear.x = max(0.03, min(vmax, 1.0 * dist))
            twist.angular.z = max(-0.6, min(0.6, 1.5 * ang))
            self._cmd_vel_pub.publish(twist)
        self._cmd_vel_pub.publish(Twist())

        # 2) 방향 맞추기
        t1 = time.time()
        while rclpy.ok() and time.time() - t1 < 15.0:
            rclpy.spin_once(self.nav, timeout_sec=0.05)
            pose = self.robot_pose()
            if pose is None:
                continue
            ang = (hyaw - pose[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(ang) < math.radians(5):
                break
            twist.linear.x = 0.0
            twist.angular.z = max(-0.6, min(0.6, 1.5 * ang))
            self._cmd_vel_pub.publish(twist)
        self._cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------
    def breadcrumb_toward(self, hx, hy):
        """탐사 중 도달에 성공했던 지점들을 징검다리 삼아 출발지 쪽으로 간다.

        긴 경로를 한 번에 주행하면 좁은 통로에서 컨트롤러가 자주 포기하는데,
        이 지점들은 이미 주행이 검증된 곳이라 구간을 잘게 나눌 수 있다.
        현재 위치보다 출발지에 더 가까운 지점만 골라 단조롭게 접근한다."""
        pose = self.robot_pose()
        if pose is None:
            return
        here = math.hypot(pose[0] - hx, pose[1] - hy)
        steps = sorted(
            (p for p in self.visited if math.hypot(p[0] - hx, p[1] - hy) < here),
            key=lambda p: -math.hypot(p[0] - hx, p[1] - hy))
        if not steps:
            return

        print(f'  갔던 길 중 출발지에 가까워지는 {len(steps)}개 지점을 거쳐 갑니다')
        for i, (wx, wy) in enumerate(steps, 1):
            pose = self.robot_pose()
            if pose is None:
                break
            if math.hypot(pose[0] - hx, pose[1] - hy) < 0.5:
                print('  출발지에 충분히 가까워졌습니다')
                break
            if math.hypot(pose[0] - wx, pose[1] - wy) < 0.2:
                continue                          # 이미 그 근처면 건너뛴다
            gx, gy = self.nearest_navigable(wx, wy)
            print(f'  [{i}/{len(steps)}] 경유 ({wx:.2f}, {wy:.2f})')
            self.go_to(gx, gy, pose[0], pose[1])
            if self.legs:
                self.legs[-1]['via'] = True  # 경유 구간 표기 (pop 대신 유지)

    # ------------------------------------------------------------------
    def return_home(self):
        """탐사가 끝나면 출발 지점으로 돌아온다. 방향까지 원래대로 맞춘다."""
        if self.home is None:
            print('출발 지점을 기록하지 못해 복귀를 건너뜁니다.', file=sys.stderr)
            return False

        hx, hy, hyaw = self.home
        print(f'\n출발 지점으로 복귀합니다 -> ({hx:.2f}, {hy:.2f}, '
              f'{math.degrees(hyaw):.0f}도)')

        for attempt in range(1, self.args.max_retries + 1):
            # 직접 복귀가 실패하면, 탐사 중 '실제로 도달에 성공한' 지점들을
            # 징검다리 삼아 간다. 각 구간이 짧고 이미 검증된 길이다.
            # 단, 출발지에 가까워지는 지점만 골라야 맵을 헤매지 않는다.
            if attempt > 1 and self.visited and not self.args.no_breadcrumb:
                self.breadcrumb_toward(hx, hy)

            # 코스트맵은 계속 바뀌므로 시도할 때마다 진입 가능 지점을 다시 찾는다.
            for _ in range(10):
                rclpy.spin_once(self.nav, timeout_sec=0.1)
                if self.costmap is not None:
                    break
            # 출발지는 탐사 초반에 기록한 맵 좌표다. 그 사이 slam_toolbox 가
            # 루프 클로저로 포즈 그래프를 다시 풀면 옛 좌표가 지금 맵에서
            # 갈 수 없는 자리(또는 맵 밖)가 되기도 한다. 실제로 39회차에서
            # 복귀가 0.0초 만에 실패했다. 기본 반경으로 못 찾으면 넓혀서
            # 다시 찾고, 그래도 없으면 이유를 남긴다.
            self.home_blocked = False
            gx, gy = self.nearest_navigable(hx, hy)
            if self.home_blocked:
                # 못 갈 좌표를 그대로 쏘면 Nav2 가 '남은 거리 0.00 m' 만
                # 반복한다. 반경을 넓혀 대신 갈 자리를 찾는다.
                gx, gy = self.nearest_navigable(hx, hy, max_radius=1.5)
            if self.home_blocked:
                gx, gy = self.nearest_navigable(hx, hy, max_radius=2.5)
            if self.home_blocked:
                print(f'      출발지({hx:.2f},{hy:.2f}) 근처 2.5m 안에 '
                      f'갈 수 있는 자리가 없습니다 — 맵이 그 사이 '
                      f'재정렬된 것으로 보입니다')

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.nav.get_clock().now().to_msg()
            goal.pose.position.x = float(gx)
            goal.pose.position.y = float(gy)
            qz, qw = yaw_to_quaternion(hyaw)
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw

            t0 = time.time()
            self.nav.goToPose(goal)
            last_print = 0.0
            timed_out = False
            stall = {}
            near = {}
            while not self.nav.isTaskComplete():
                if self.nav_stalled(stall):
                    self.nav.cancelTask()
                    print('      Nav2 가 경로를 못 짭니다'
                          "(남은 거리 0.00 m 반복) — 다른 방법으로 갑니다")
                    break
                if time.time() - last_print >= 3.0:
                    last_print = time.time()
                    fb = self.nav.getFeedback()
                    if fb:
                        print(f'      남은 거리 {fb.distance_remaining:.2f} m')
                # 맵이 틀어지면 벽 너머에 빈 공간이 칠해지고, 프론티어 탐사가
                # 그리로 달려간다. Nav2 는 그 맵을 믿으므로 그대로 벽에 박는다
                # (201회차에서 왼쪽 바깥벽을 정면으로 밀었다).
                # 맵이 뭐라고 하든 라이다로 잰 차체 여유가 없으면 멈춘다.
                if self.map_broken():
                    self.nav.cancelTask()
                    break
                # 복귀 구간은 기준을 조금 낮춘다. 탐사와 달리 목적지가
                # 하나뿐이라, 좁은 곳에서 매번 취소하면 영영 못 돌아간다
                # (360회차: 세 번 다 같은 자리에서 취소, 1.07m 남기고 포기).
                # 대신 접촉 직전(1.2cm)에서는 여전히 멈춘다.
                gap, why = self.too_close(near, hard=HARD_GAP * 0.8,
                                          watch=WATCH_GAP * 0.75)
                if why:
                    self.nav.cancelTask()
                    print(f'      차체 여유 {gap*100:+.1f}cm ({why}) — '
                          f'목표를 취소하고 물러납니다')
                    if not self.back_off():
                        self.free_self()
                    break
                if time.time() - t0 > self.args.home_timeout:
                    self.nav.cancelTask()
                    timed_out = True
                    print(f'      시간 초과({self.args.home_timeout:.0f}초)')
                    break

            # Nav2 는 자체 허용오차 안에 들어오면 멈춘다. 마지막 몇 cm 는
            # TF 를 보며 직접 좁히고 방향까지 맞춘다.
            # 미세 접근은 벽을 고려하지 않고 직진하므로 가까울 때만 쓴다.
            # (전진 직전에 라이다로 정면을 확인하므로 벽에 박지는 않는다.)
            # 출발지가 인플레이션에 막혀 있으면 Nav2 는 그 앞에서 "도착" 처리하고
            # 멈춰버리므로, 남은 거리가 이 범위면 직접 좁혀야 한다.
            pose = self.robot_pose()
            if pose and math.hypot(pose[0] - hx, pose[1] - hy) < 0.6:
                print('      미세 접근으로 위치와 방향을 맞춥니다')
                self.precise_approach(hx, hy, hyaw)

            elapsed = time.time() - t0
            # 취소 직후 getResult() 는 SUCCEEDED 를 돌려줄 수 있으므로
            # 실제로 도착했는지는 현재 위치로 직접 확인한다.
            pose = self.robot_pose()
            err = math.hypot(pose[0] - hx, pose[1] - hy) if pose else float('inf')
            if err <= self.args.home_tolerance:
                yaw_err = abs((hyaw - pose[2] + math.pi) % (2 * math.pi) - math.pi)
                print(f'      복귀 완료 ({elapsed:.1f}초, 위치 오차 {err:.3f} m, '
                      f'방향 오차 {math.degrees(yaw_err):.1f}도)')
                self.legs.append({'x': hx, 'y': hy, 'ok': True,
                                  'seconds': elapsed, 'home': True})
                return True
            print(f'      복귀 실패 ({attempt}/{self.args.max_retries}회, '
                  f'출발지까지 {err:.3f} m 남음)')
            # 출발지가 벽에 바짝 붙어 있으면(맵이 재정렬돼 그렇게 되기도
            # 한다) 더 밀어붙일수록 벽을 민다. 328회차에서 그렇게 접촉했다.
            # 몸이 벽에 닿기 직전인데 이미 충분히 가까우면 거기서 멈춘다.
            gap_now = self.body_clearance()
            if attempt >= 2 and err <= self.args.home_near \
                    and gap_now is not None and gap_now < 0.03:
                print(f'      출발지 {err:.2f} m 앞이고 벽에 {gap_now*100:.1f}cm '
                      f'까지 붙었습니다. 더 밀지 않고 여기서 마칩니다.')
                self.legs.append({'x': hx, 'y': hy, 'ok': True,
                                  'seconds': 0.0, 'home': True, 'near': True})
                return True
            # 후진은 출발지에서 멀어지게 하므로, 정말 벽에 끼었을 때만 한다.
            # 정면만 보면 옆구리로 낀 경우를 놓친다. 차체 어느 쪽이든
            # 여유가 없으면 낀 것으로 본다.
            gap = self.body_clearance()
            if (self.scan_front is not None and self.scan_front < 0.15) \
                    or (gap is not None and gap < 0.02):
                # 후진이 안 되면(뒤도 막힘) 제자리 회전으로 빼낸다.
                if not self.back_off():
                    self.free_self()
            time.sleep(2.0)

        print(f'      복귀를 포기합니다. 출발지에서 {err:.3f} m 떨어져 있습니다')
        self.legs.append({'x': hx, 'y': hy, 'ok': False,
                          'seconds': 0.0, 'home': True})
        return False

    # ------------------------------------------------------------------
    # 본체
    # ------------------------------------------------------------------
    def explore(self):
        start = time.time()
        round_no = 0
        instant = 0          # 0.0초 만에 거부당한 목표가 연속 몇 번인가

        while rclpy.ok():
            if time.time() - start > self.args.total_timeout:
                print('\n전체 제한 시간에 도달했습니다. 탐사를 마칩니다.')
                return True
            if self.map_broken():
                return False

            # 최신 맵을 한 번 받아둔다
            for _ in range(10):
                rclpy.spin_once(self.nav, timeout_sec=0.1)

            pose = self.robot_xy()
            if pose is None:
                print('로봇 위치(TF)를 아직 못 읽었습니다. 잠시 대기...')
                time.sleep(1.0)
                continue
            rx, ry = pose

            frontiers = self.find_frontiers()
            if not frontiers:
                print('\n남은 프론티어가 없습니다. 탐사 완료.')
                return True

            target = self.choose_goal(frontiers, rx, ry)
            if target is None:
                print('\n갈 수 있는 프론티어가 없습니다. 탐사 완료.')
                return True

            x, y, size, dist = target
            round_no += 1
            gx, gy = self.retract_goal(x, y, rx, ry)
            moved = math.hypot(gx - x, gy - y)
            note = f'  (경계에서 {moved:.2f} m 당김)' if moved > 1e-3 else ''
            print(f'\n[{round_no}회차] 프론티어 {len(frontiers)}개 중 선택  '
                  f'-> ({x:.2f}, {y:.2f})  크기 {size}칸, 거리 {dist:.2f} m{note}')

            self.last_goal = (x, y)
            ok, elapsed = self.go_to(gx, gy, rx, ry)
            if ok:
                self.visited.append((x, y))
                instant = 0
                print(f'      도착 ({elapsed:.1f}초)')
            else:
                # 0.0초 만에 실패하면 로봇이 시도조차 못 한 것이다. 주행하다
                # 실패한 게 아니라 Nav2 가 목표를 '거부' 한 것이다. 이 상태로
                # 계속 쏘면 아무 일도 안 하면서 프론티어만 제외 목록에 쌓는다
                # (실물 384회차: 목표 28개가 전부 0.0초 실패, 총 19초).
                # 원인은 대개 controller_server 가 active 가 아닌 것이다.
                if elapsed < 0.5:
                    instant += 1
                    if instant >= 5:
                        print('\nNav2 가 목표를 즉시 거부합니다(5회 연속 '
                              '0.0초 실패). 주행을 시작조차 못 하는 상태라 '
                              '중단합니다.\n'
                              '  확인: ros2 lifecycle get /controller_server\n'
                              '  대개 Nav2 가 두 벌 떠 있거나 노드가 '
                              'active 가 아닙니다.\n'
                              '  조치: pkill -f nav2_  후 '
                              './pc_stack_start.sh 로 한 벌만 다시 띄우세요.',
                              file=sys.stderr)
                        self.nav2_broken = True
                        return False
                else:
                    instant = 0
                # Nav2 실패는 코스트맵이 아직 안 자랐거나 복구 동작이 필요한
                # 일시적 경우가 많다. 바로 제외하면 프론티어가 하나뿐일 때
                # 탐사가 통째로 끝나버리므로 몇 번은 다시 시도한다.
                key = (round(x, 2), round(y, 2))
                self.fail_counts[key] = self.fail_counts.get(key, 0) + 1
                tries = self.fail_counts[key]
                if tries >= self.args.max_retries:
                    self.blacklist.append((x, y))
                    print(f'      실패 {tries}회. 이 지점을 제외 목록에 넣습니다 '
                          f'(누적 {len(self.blacklist)}개)')
                else:
                    print(f'      실패 ({tries}/{self.args.max_retries}회). '
                          f'잠시 후 다시 시도합니다')
                    self.back_off()
                    time.sleep(2.0)

        return False

    # ------------------------------------------------------------------
    def save_map(self, path):
        path = os.path.expanduser(path)
        print(f'\n맵을 저장합니다: {path}.yaml / .pgm')
        try:
            # map_saver 기본 대기 시간은 5초인데, /map 이 크거나 시뮬레이션이
            # 느리면 그 안에 못 받아서 "Failed to spin map subscription" 으로
            # 실패한다. 넉넉히 준다.
            r = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path,
                 '--ros-args', '-p', 'save_map_timeout:=20.0'],
                capture_output=True, text=True, timeout=60)
            if r.returncode == 0:
                print('저장 완료')
                return True
            print(f'저장 실패:\n{r.stderr.strip()}', file=sys.stderr)
        except Exception as e:
            print(f'저장 중 오류: {e}', file=sys.stderr)
        print('수동으로 저장하세요: '
              f'ros2 run nav2_map_server map_saver_cli -f {path}')
        return False

    def print_report(self):
        line = '=' * 56
        print('\n' + line)
        print('탐사 결과')
        print(line)
        if not self.legs:
            print('이동한 구간이 없습니다.')
            print(line + '\n')
            return

        total = sum(l['seconds'] for l in self.legs)
        ok = sum(1 for l in self.legs if l['ok'])
        for i, l in enumerate(self.legs, 1):
            mark = '성공' if l['ok'] else '실패'
            tag = '  <- 출발지 복귀' if l.get('home') else ''
            if l.get('via'):
                tag += ' (경유)'
            print(f'  {i:2d}. ({l["x"]:6.2f}, {l["y"]:6.2f})  '
                  f'{mark}  {l["seconds"]:5.1f}s{tag}')
        print('-' * 56)
        print(f'구간 {len(self.legs)}개 중 {ok}개 성공,  총 {total:.1f}초')
        if self.grid is not None:
            known = int(np.count_nonzero(self.grid != -1))
            total_cells = self.grid.size
            print(f'탐색 완료 비율: {known / total_cells * 100:.1f}% '
                  f'({known:,} / {total_cells:,} 칸)')
        print(line + '\n')



def wait_nav2(navigator, timeout=120.0):
    """Nav2 가 목표를 받을 준비가 됐는지 시간을 정해두고 확인한다.

    nav2_simple_commander 의 waitUntilNav2Active() 는 끝을 정하지 않고
    기다린다. 320회차에서 Nav2 도 slam_toolbox 도 lifecycle 이 active 인데
    4단계가 그 안에서 9분 넘게 멈춰 있었다(같은 조건으로 새로 띄운
    프로세스는 4초 만에 통과했다). 한 회차가 통째로 날아간다.

    정작 필요한 것은 '목표를 받아줄 액션 서버가 있는가' 하나뿐이므로
    그것만 본다. 없으면 False 를 돌려주고, 부르는 쪽이 판단한다."""
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < timeout:
        if navigator.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            return True
        rclpy.spin_once(navigator, timeout_sec=0.1)
    return False

def main():
    p = argparse.ArgumentParser(description='Pinky Pro 자율 탐사')
    p.add_argument('--save', default='~/pinky_map',
                   help='맵 저장 경로(확장자 제외), 기본 ~/pinky_map')
    p.add_argument('--min-frontier', type=int, default=15,
                   help='이 칸수보다 작은 프론티어는 무시, 기본 15')
    p.add_argument('--cluster', type=float, default=0.7,
                   help='직전 목표에서 이 거리 안의 프론티어를 먼저 훑는다(m). '
                        '0 이면 끄기')
    p.add_argument('--cluster-bonus', type=float, default=2.0,
                   help='그때 점수에 더할 배수(2.0 이면 세 배). 한 구역을 '
                        '끝내고 옮기게 하는 것이 목적이다')
    p.add_argument('--back-penalty', type=float, default=0.35,
                   help='뒤쪽 목표에 주는 벌점(0~1). 0 이면 방향을 안 본다')
    p.add_argument('--min-distance', type=float, default=0.35,
                   help='이 거리보다 가까운 목표는 건너뜀(m), 기본 0.35')
    p.add_argument('--goal-timeout', type=float, default=90.0,
                   help='구간당 제한 시간(초), 기본 90')
    p.add_argument('--total-timeout', type=float, default=900.0,
                   help='전체 제한 시간(초), 기본 900')
    p.add_argument('--blacklist-radius', type=float, default=0.6,
                   help='실패 지점 재시도 금지 반경(m), 기본 0.6')
    p.add_argument('--max-retries', type=int, default=3,
                   help='한 지점을 이 횟수만큼 실패해야 제외한다, 기본 3')
    p.add_argument('--no-return', action='store_true',
                   help='탐사가 끝나도 출발 지점으로 복귀하지 않는다')
    p.add_argument('--no-breadcrumb', action='store_true',
                   help='복귀할 때 갔던 길을 되짚지 않고 곧바로 출발지를 목표로 삼는다')
    p.add_argument('--home', default=None,
                   help='복귀할 출발 지점을 "x,y,yaw도" 로 직접 지정한다. '
                        '생략하면 이 프로세스가 시작한 자리를 출발점으로 삼는다. '
                        '여러 단계로 나눠 실행할 때는 반드시 넘겨야 한다')
    p.add_argument('--home-near', type=float, default=0.40,
                   help='출발지가 벽에 막혀 있을 때 이 거리 안이면 복귀로 인정')
    p.add_argument('--home-tolerance', type=float, default=0.05,
                   help='이 거리 안에 들어와야 복귀 성공으로 인정(m), 기본 0.05')
    p.add_argument('--home-timeout', type=float, default=180.0,
                   help='복귀 한 번에 허용할 시간(초), 기본 180. '
                        '복귀는 맵을 가로질러야 해서 탐사 구간보다 오래 걸린다')
    p.add_argument('--goal-retract', type=float, default=0.12,
                   help='프론티어에서 로봇 쪽으로 당길 거리(m), 기본 0.12. '
                        '0 이면 프론티어 칸을 그대로 목표로 삼는다')
    p.add_argument('--no-save', action='store_true',
                   help='끝나도 맵을 저장하지 않음')
    p.add_argument('--sim', action='store_true',
                   help='Gazebo 시뮬레이션에서 실행')
    p.add_argument('--shutdown-nav2', action='store_true',
                   help='끝날 때 Nav2 라이프사이클을 종료한다. '
                        '기본은 살려둬서 바로 다시 탐사할 수 있게 한다')
    args = p.parse_args()

    if args.sim:
        rclpy.init(args=['pinky_explore',
                         '--ros-args', '-p', 'use_sim_time:=true'])
        print('시뮬레이션 모드 (use_sim_time=true)')
    else:
        rclpy.init()

    navigator = BasicNavigator()
    if args.sim:
        try:
            navigator.set_parameters(
                [Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception as e:
            print(f'use_sim_time 설정 경고: {e}', file=sys.stderr)

    explorer = Explorer(navigator, args)
    exit_code = 0
    started = False

    try:
        print('Nav2 가 준비될 때까지 기다립니다...')
        if not wait_nav2(navigator):
            print('Nav2 액션 서버가 응답하지 않습니다. 이 단계를 건너뜁니다.',
                  file=sys.stderr)
            raise RuntimeError('Nav2 준비 실패')
        print('Nav2 준비 완료')

        # 맵이 아직 로봇 위치를 못 덮은 경우에만 살짝 움직인다.
        # 이미 맵 안에 있으면 앞이 벽일 수 있으므로 건드리지 않는다.
        #
        # 맵·TF·라이다가 다 들어온 뒤에 판단해야 한다. 예전에는 맵만 보고
        # 넘어가서, TF 가 아직이면 robot_xy() 가 None 이라 _robot_in_bounds()
        # 가 무조건 False 였다. 즉 '맵 밖에 있다' 고 오해하고, 라이다도 아직
        # 없는 상태로 30cm 를 돌진했다.
        deadline = time.time() + 10.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.2)
            if (explorer.grid is not None and explorer.scan_msg is not None
                    and explorer.robot_xy() is not None):
                break
        if args.sim and explorer.grid is not None \
                and explorer.robot_xy() is not None \
                and not explorer._robot_in_bounds():
            print('출발 지점이 맵 경계 밖입니다. 살짝 움직입니다...')
            explorer.nudge_forward()

        explorer.wait_for_map()

        # 탐사가 끝나면 돌아올 지점. --home 이 있으면 그것을 쓴다
        # (단계를 나눠 실행할 때 이 프로세스의 시작 위치는 진짜 출발점이 아니다).
        if args.home:
            hx, hy, hdeg = (float(v) for v in args.home.split(','))
            explorer.home = (hx, hy, math.radians(hdeg))
            print(f'출발 지점을 인자로 받았습니다: ({hx:.2f}, {hy:.2f}, {hdeg:.0f}도)')
        else:
            explorer.home = explorer.robot_pose()
        if explorer.home and not args.home:
            hx, hy, hyaw = explorer.home
            print(f'출발 지점 기록: ({hx:.2f}, {hy:.2f}, '
                  f'{math.degrees(hyaw):.0f}도)')

        started = True
        print('\n탐사를 시작합니다. 중단하려면 Ctrl+C\n')
        explorer.explore()

        # 맵이 무너졌으면 복귀도 의미가 없다(목표 좌표 자체가 틀렸다).
        # 종료 코드 3 으로 알려, 상위(pinky_hybrid.py)가 다음 단계를
        # 돌리지 않고 멈추게 한다.
        if explorer.nav2_broken:
            exit_code = 4          # 상위(하이브리드)가 다음 단계를 멈추도록
        elif explorer.diverged:
            exit_code = 3
        elif not args.no_return:
            explorer.return_home()

    except KeyboardInterrupt:
        print('\n사용자 중단')
        try:
            navigator.cancelTask()
        except Exception:
            pass
        exit_code = 130
    except Exception as e:
        print(f'오류: {e}', file=sys.stderr)
        exit_code = 1
    finally:
        if started:
            try:
                explorer.print_report()
            except Exception as e:
                print(f'리포트 실패: {e}', file=sys.stderr)
            # 중단했더라도 그때까지 만든 맵은 저장한다
            if not args.no_save:
                explorer.save_map(args.save)
        # Nav2 를 내리면 라이프사이클이 finalized 가 되어, 다시 탐사하려면
        # navigation_launch 를 통째로 재시작해야 한다. 기본은 살려둔다.
        if args.shutdown_nav2:
            try:
                navigator.lifecycleShutdown()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
