#!/usr/bin/env python3
"""
Pinky Pro 좌수법 벽 따라가기 매핑 (Left-hand wall following)
==============================================================
왼쪽 벽에 일정 거리를 유지하며 계속 붙어 가면, 벽면 전체를 한 바퀴 훑게 된다.
이 트랙은 벽 16개가 모두 외곽선과 이어진 '하나의 덩어리'라(섬이 없다),
좌수법 한 바퀴로 모든 통로를 지나간다 = 빠지는 구역 없이 맵이 완성된다.

프론티어 방식과의 차이
----------------------
  프론티어 : 전역 플래너(Nav2)로 미탐색 경계를 골라 이동.
             효율적이지만 좁은 통로에서 코스트맵/컨트롤러 실패에 취약하고,
             작은 경계를 걸러내는 임계값 때문에 구석이 누락될 수 있다.
  좌수법   : Nav2 를 아예 쓰지 않는다. 라이다와 cmd_vel 만으로 도는
             순수 지역 제어라 플래너 문제에서 자유롭다.
             대신 벽면 전체를 훑으므로 이동 거리는 더 길 수 있다.

사용법
------
  python3 pinky_wallfollow.py --sim --save ~/pinky/map_wallfollow

종료 조건
---------
  기본은 '맵이 더 이상 안 늘어남'(--idle-time) 이다. 출발점 복귀를 종료
  조건으로 쓰면 훑기가 끝날 때마다 로봇이 굳이 집으로 돌아왔다가 다음
  단계에 다시 나가게 되므로, 한 바퀴 완주는 알리기만 하고 계속 훑는다.
  (--stop-on-loop 을 주면 예전처럼 한 바퀴에서 멈춘다)
"""

import argparse
import math
import subprocess
import sys
import time
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSHistoryPolicy, QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

import pinky_mapcheck as mapcheck


class _WatchdogPublisher:
    """Safety shim for supervised real-world runs (added 2026-09-16).

    Converts Twist to TwistStamped on /control/manual_velocity so the
    pinky_control_watchdog enforces deadman (0.35 s), clamps
    (0.15 m/s, 0.50 rad/s) and the stop latch. All WallFollower logic
    is untouched; only the transport changes. Raw /cmd_vel is never
    published by this process.
    """

    def __init__(self, node):
        from geometry_msgs.msg import TwistStamped
        self._node = node
        self._pub = node.create_publisher(TwistStamped, '/control/manual_velocity', 10)

    def publish(self, twist):
        from geometry_msgs.msg import TwistStamped
        stamped = TwistStamped()
        stamped.header.stamp = self._node.get_clock().now().to_msg()
        stamped.twist = twist
        self._pub.publish(stamped)

MAP_QOS = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


# 로봇 몸통 치수 (base_link 기준, 충돌 STL 실측)
HALF_W = 0.0635        # 좌우 반폭
NOSE = 0.0565          # 앞쪽 반길이
TAIL = 0.0565          # 뒤쪽 반길이
LIDAR_X = -0.017       # 라이다는 차체 중심보다 17mm 뒤에 달려 있다


def body_reach(robot_deg):
    """라이다에서 그 방향으로 잰 로봇 몸통 끝까지의 거리(m).

    라이다가 읽은 거리에서 이 값을 빼야 '벽까지 실제로 남은 여유' 가 된다.
    지금까지는 라이다 거리를 그대로 임계값과 비교했는데, 몸통 반폭이 6.35cm
    인데 임계값이 5.5cm 였다. 즉 벽이 이미 차체 안으로 1cm 파고든 뒤에야
    피하기 시작했다. 벽에 부딪치는 근본 원인이 여기에 있었다."""
    a = math.radians(robot_deg)
    ca, sa = math.cos(a), math.sin(a)
    cands = []
    if abs(ca) > 1e-6:
        cands.append(((NOSE - LIDAR_X) if ca > 0 else (-TAIL - LIDAR_X)) / ca)
    if abs(sa) > 1e-6:
        cands.append(HALF_W / abs(sa))
    return min(c for c in cands if c > 0)


def body_gap(msg, win_deg=1.7):
    """스캔 한 장에서 '차체 표면부터 벽까지' 남은 최소 여유(m). 못 재면 None.

    음수면 이미 몸이 벽 안으로 들어간 것이다. 주행 코드마다 따로 구현하면
    한쪽만 고치는 일이 생기므로(3단계는 아예 검사가 없어 365회차에서
    벽에 닿았다) 여기 한 곳에 둔다."""
    if msg is None:
        return None
    r = WallFollower._denoise(msg, win_deg=win_deg)
    ok = np.isfinite(r)
    if not ok.any():
        return None
    ang = np.degrees(msg.angle_min + np.arange(len(r)) * msg.angle_increment)
    ang = (ang + 360.0) % 360.0 - 180.0
    gaps = r[ok] - np.array([body_reach(a) for a in ang[ok]])
    return float(gaps.min())


def contact_direction(m, run_deg=4.5, margin=0.02, max_run_deg=60.0):
    """몸이 벽에 닿은 방향을 찾는다 (로봇 기준 각도). 없으면 None.

    너무 가까운 물체는 라이다가 무효값을 낸다. 그렇다고 무효값이 뭉쳐
    나온다고 곧바로 '닿았다' 로 보면 안 된다. 실물 라이다는 (1) 너무
    멀거나 (2) 반사가 약하거나 (3) 낮은 벽 위로 빔이 지나가도 무효값을
    낸다. 실제로 실물 시험에서 '왼쪽 12cm 여유' 인데 접촉으로 잘못
    판정해 계속 탈출 동작을 반복했다.

    그래서 무효 구간의 양옆에 있는 '유효한' 값을 본다. 양쪽 다 차체
    표면에 닿을 만큼 가까우면(여유 < margin) 그 사이도 벽이 이어진
    것이니 실제 접촉이다. 한쪽이라도 여유가 있으면 그냥 안 잡힌 것이다.
    기준을 고정 거리로 두면 안 된다. 차체 끝까지의 거리가 방향마다
    달라(뒤 4.0cm, 옆 6.4cm, 대각 9.0cm) 같은 16cm 도 뒤쪽이면 여유
    12cm 인데 접촉으로 오판한다. 구간이 지나치게 넓은 것도 접촉이
    아니다 (몸에 닿은 자국은 그렇게 넓게 퍼지지 않는다)."""
    if m is None:
        return None
    n = len(m.ranges)
    per = abs(math.degrees(m.angle_increment)) or 0.5
    min_run = max(3, int(round(run_deg / per)))
    max_run = max(min_run + 1, int(round(max_run_deg / per)))
    rs = m.ranges

    def val(i):
        v = rs[i % n]
        return v if (math.isfinite(v) and v > m.range_min * 1.02) else None

    start = next((i for i in range(n) if val(i) is not None), None)
    if start is None:
        return None                    # 전부 무효 = 라이다 이상이지 접촉이 아니다

    cnt = 0
    for k in range(1, n + 1):
        idx = (start + k) % n
        if val(idx) is None:
            cnt += 1
            continue
        if min_run <= cnt <= max_run:
            s0 = (idx - cnt) % n
            b_i, a_i = (s0 - 1) % n, idx
            before, after = val(b_i), val(a_i)

            def gap(i, v):
                d = math.degrees(m.angle_min + i * m.angle_increment)
                return v - body_reach((d + 180 + 180) % 360 - 180)

            if before is not None and after is not None \
                    and gap(b_i, before) < margin \
                    and gap(a_i, after) < margin:
                mid = (s0 + cnt // 2) % n
                a = math.degrees(m.angle_min + mid * m.angle_increment)
                return (a + 180 + 180) % 360 - 180
        cnt = 0
    return None


class WallFollower(Node):
    """이 로봇의 rplidar_link 는 base_link 기준 yaw 180도로 장착돼 있다.
    따라서 스캔각 0 이 로봇의 '뒤', ±180도가 '앞', -90도가 '왼쪽' 이다."""

    def __init__(self, args):
        super().__init__('pinky_wallfollow')
        self.args = args
        self.scan = None
        self.smooth = None    # 제어용(넓은 창, 부드러움)
        self.sharp = None     # 안전용(좁은 창, 얇은 물체 보존)
        self.last_lin = 0.0        # 직전에 낸 직진 속도
        self.stuck_in_pocket = False   # 막힌 주머니에 갇혔는가
        self.hand = -1 if getattr(args, 'right', False) else +1
        self.known = 0                # 맵에서 관측된 칸 수
        self.escape = None            # 진행 중인 탈출 동작 (아래 escape_step 참고)
        self.corner = None            # 앞이 막혔을 때의 동작
        self.turn_sign = None         # 계산으로 고른 회전 방향(없으면 따라가는 벽 반대)
        self.opening_since = None     # 개구부 상태가 언제부터 이어졌나
        self.pub = _WatchdogPublisher(self)  # safety: via watchdog, never raw /cmd_vel
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, MAP_QOS)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _map_cb(self, msg):
        self.known = sum(1 for v in msg.data if v != -1)
        self._map_msg = msg

    def map_diverged(self):
        """SLAM 좌표계가 틀어졌는지 검사.

        벽에 부딪치면 스캔 매칭이 어긋나 맵이 실제 트랙보다 커진다.
        로봇이 그걸 모른 채 계속 돌면 맵을 더 망치므로, 감지되면 즉시 멈춘다."""
        msg = getattr(self, '_map_msg', None)
        if msg is None:
            return False, None
        import numpy as np
        g = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        h = mapcheck.health(g, msg.info.resolution)
        # 공간 크기를 모르면(실물) 절대 크기 검사가 꺼진다. 그때는 맵이
        # 갑자기 몇 배로 커지는지로 붕괴를 잡는다.
        now = time.time()
        gap_t = now - getattr(self, '_health_prev_t', 0.0)
        long_side = h['size_m'][0]
        if not h['diverged'] and mapcheck.jumped(
                getattr(self, '_health_prev_long', None), long_side, gap_t):
            h = dict(h)
            h['diverged'] = True
            h['reasons'] = [f'맵이 {gap_t:.0f}초 만에 '
                            f'{self._health_prev_long:.2f} -> {long_side:.2f} m '
                            f'로 갑자기 커짐 — 좌표계 틀어짐']
        self._health_prev_long, self._health_prev_t = long_side, now
        return h['diverged'], h

    def _scan_cb(self, msg):
        self.scan = msg
        self.smooth = self._denoise(msg)
        self.sharp = self._denoise(msg, win_deg=1.7)

    @staticmethod
    def _denoise(msg, win_deg=5.0):
        """빔별 잡음을 걷어낸 거리 배열(무효값은 nan).

        시뮬 라이다에는 표준편차 2cm 의 가우시안 잡음이 걸려 있다
        (pinky_gz.urdf.xacro 의 <noise>). 빔 640개의 '최솟값' 으로 안전을
        판정하면 실제 거리가 아니라 잡음 분포의 꼬리를 재게 된다. 실제로
        15cm 떨어진 벽이 5cm 로 읽혀 '충돌 직전' 경보가 끝없이 울렸다.
        인접한 빔들은 같은 벽면을 보므로, 창 안의 중앙값을 쓰면 편향 없이
        잡음만 1/3 로 줄어든다. 실제 라이다에도 그대로 필요한 처리다."""
        r = np.asarray(msg.ranges, dtype=np.float64)
        r[~np.isfinite(r)] = np.nan
        r[r <= msg.range_min * 1.02] = np.nan
        # 창 크기는 '빔 몇 개' 가 아니라 '몇 도' 로 정한다. 시뮬 라이다는
        # 640빔/360도지만 실물 RPLidar C1 은 모드에 따라 빔 수가 다르다.
        # 개수로 고정하면 실물에서 창이 엉뚱하게 좁아지거나 넓어진다.
        per = abs(math.degrees(msg.angle_increment)) or 0.5
        win = max(3, int(round(win_deg / per)) | 1)      # 홀수로 맞춘다
        pad = win // 2
        ext = np.concatenate([r[-pad:], r, r[:pad]])
        stack = np.stack([ext[i:i + len(r)] for i in range(win)])
        # 창 전체가 무효인 방향이 있으면 nanmedian 이 경고를 낸다.
        # 그런 방향은 아래에서 무효로 처리하므로 경고만 조용히 덮는다.
        with np.errstate(all='ignore'), warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            out = np.nanmedian(stack, axis=0)
        # 창이 전부 무효면 그대로 무효
        out[np.all(np.isnan(stack), axis=0)] = np.nan
        return out

    def _angles(self):
        """빔별 로봇 기준 각도(도) 배열. rplidar 는 yaw 180도 장착이다."""
        m = self.scan
        a = np.degrees(m.angle_min + np.arange(len(m.ranges))
                       * m.angle_increment)
        return (a + 360.0) % 360.0 - 180.0

    def beam(self, robot_deg, half=4.0):
        """로봇 기준 각도(0=정면, +90=왼쪽)의 거리를 좁은 창의 중앙값으로 읽는다.

        넓은 부채꼴의 최솟값을 쓰면 폭 30cm 통로에서 반대편 벽이나
        개구부 모서리를 '가까운 왼쪽 벽'으로 착각한다. 좁게 보고
        중앙값을 써야 그 벽만 본다."""
        if self.scan is None or self.smooth is None:
            return None
        ang = self._angles()
        sel = (np.abs((ang - robot_deg + 180.0) % 360.0 - 180.0) <= half)
        vals = self.smooth[sel]
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def left_min(self):
        """왼쪽 반구(+20도~+160도)에서 가장 가까운 거리 (잡음 제거 후)."""
        if self.scan is None or self.smooth is None:
            return None
        ang = self._angles()
        vals = self.smooth[(ang >= 20.0) & (ang <= 160.0)]
        vals = vals[np.isfinite(vals)]
        return float(vals.min()) if vals.size else None

    def side_wall(self, sign):
        """옆 벽까지의 '수직' 거리와 벽 대비 기울기. sign=+1 왼쪽, -1 오른쪽.

        b = 정옆(90도), a = 비스듬히 앞(50도) 두 빔으로 벽의 직선을 추정한다.
        한 점만 보면 거리와 기울기를 구분할 수 없어 모서리에서 흔들린다."""
        theta = math.radians(40.0)
        b = self.beam(90.0 * sign)
        a = self.beam(50.0 * sign)
        if a is None or b is None:
            return None, None
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        return b * math.cos(alpha), alpha * sign

    def left_wall(self):
        """따라가는 쪽 벽까지의 수직 거리와 기울기.

        --right 를 주면 오른쪽 벽을 따라간다. 좌수법으로 한 바퀴를 이미 돈
        뒤에 같은 방향으로 또 돌면 같은 길을 되짚을 뿐이다(14회차에서 +3칸).
        반대 손으로 돌면 벽 네트워크의 반대면을 훑게 되어 안 가본 곳이 나온다."""
        return self.side_wall(self.hand)

    def pose(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_link',
                                              rclpy.time.Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def yaw(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_link',
                                              rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def contact_dir(self, run_deg=4.5, margin=0.02, max_run_deg=60.0):
        """몸이 벽에 닿은 방향(로봇 기준 각도). 없으면 None.
        판정 자체는 모듈 함수 contact_direction 에 있다 (LED 노드와 공용)."""
        return contact_direction(self.scan, run_deg, margin, max_run_deg)

    def all_min(self):
        """전방향 최단 거리와 그 방향(로봇 기준 각도) — 잡음 제거 후"""
        if self.scan is None or self.smooth is None:
            return None, None
        ang = self._angles()
        ok = np.isfinite(self.smooth)
        if not ok.any():
            return None, None
        i = int(np.nanargmin(np.where(ok, self.smooth, np.inf)))
        return float(self.smooth[i]), float(ang[i])

    def wall_to_seek(self, near=0.35, far=3.0):
        """따라갈 만한 벽의 방향과 거리를 찾는다. 없으면 (None, None).

        '가장 가까운 벽' 을 그냥 고르면 안 된다. 그건 이미 옆구리에 붙어
        있는 벽이라, 그쪽으로 가라고 하면 3cm 앞의 벽으로 돌진하거나
        (50회차에서 실제로 그랬다) 각도가 커서 제자리 회전만 한다
        (51회차). 지금 붙어 있는 것 말고, 가서 붙을 수 있는 거리(near~far)
        에 있는 벽 중 가장 가까운 것을 고른다."""
        if self.scan is None or self.smooth is None:
            return None, None
        ang = self._angles()
        ok = np.isfinite(self.smooth) & (self.smooth >= near) \
            & (self.smooth <= far)
        if not ok.any():
            return None, None
        idx = np.nonzero(ok)[0]
        k = int(np.argmin(self.smooth[idx]))
        return float(ang[idx[k]]), float(self.smooth[idx[k]])

    def enclosed(self, limit=0.80):
        """사방이 막힌 좁은 공간에 갇혀 있는가.

        트랙 안에는 X자 위쪽처럼 사방이 막힌 주머니가 있다(약 50x25cm).
        거기 놓이면 로봇은 회전만 할 수 있을 뿐 빠져나올 수 없어서, 겉보기엔
        '제자리에서 빙빙 도는' 것처럼 보인다. 실물 시험에서 실제로 그랬다.
        원인을 모르면 주행 로직을 계속 고치게 되므로 먼저 알려줘야 한다.

        어느 방향으로도 limit 보다 멀리 못 보면 갇힌 것으로 본다.
        정상 통로에서는 복도 끝까지 몇 미터가 보인다."""
        if self.scan is None or self.smooth is None:
            return None
        v = self.smooth[np.isfinite(self.smooth)]
        if v.size == 0:
            return None
        return float(v.max()) < limit

    # 안전 판정과 제어에 쓰는 거리를 나눈 이유
    # ----------------------------------------
    # 제어(벽까지 거리, 기울기)는 넓은 창의 중앙값이 좋다. 잡음이 줄고
    # 조향이 떨리지 않는다. 그런데 그 창은 얇은 물체를 통째로 지운다.
    # 실측: 9빔(5도) 중앙값은 4빔 이하 물체를 없앤다. 0.2m 거리에서 두께
    # 6mm 인 벽 끝은 1.7도(3빔)라 그대로 사라진다. 그래서 로봇이 벽
    # 모서리를 못 보고 그대로 긁었다(201·202회차).
    # 안전 판정은 3빔 창을 써서 얇은 것도 살린다. 단일 빔 잡음은 여전히
    # 걸러지고, 이웃 2개가 함께 가까우면 실제 물체로 본다.
    def clearance_min(self, lo=-180.0, hi=180.0):
        """몸통에서 벽까지 남은 여유 중 가장 작은 값과 그 방향(로봇 기준).

        음수면 이미 몸이 벽 안으로 들어간 것이다.
        lo~hi 로 볼 각도 범위를 좁힐 수 있다 (예: 왼쪽 옆구리만).
        거리는 잡음을 걷어낸 값을 쓴다. 날것의 최솟값을 쓰면 폭 30cm
        통로에서 상시 '충돌 직전' 이 되어 로봇이 아무 데도 못 간다."""
        rng = self.sharp if self.sharp is not None else self.smooth
        if self.scan is None or rng is None:
            return None, None
        ang = self._angles()
        sel = (ang >= lo) & (ang <= hi) & np.isfinite(rng)
        if not sel.any():
            return None, None
        idx = np.nonzero(sel)[0]
        gaps = rng[idx] - np.array([body_reach(a) for a in ang[idx]])
        k = int(np.argmin(gaps))
        return float(gaps[k]), float(ang[idx[k]])

    def room(self, robot_deg, half=40.0):
        """그 방향 부채꼴에 확보된 여유 거리(m).

        최솟값은 빔 하나의 잡음에 휘둘리고, 중앙값은 부채꼴 끝의 벽을 못
        본다. 하위 25% 값이 '이 방향으로 갈 수 있는가' 를 가장 잘 나타낸다.
        무효값(코앞이라 안 보이는 것)은 '막혔다'로 본다."""
        if self.scan is None or self.smooth is None:
            return None
        ang = self._angles()
        sel = np.abs((ang - robot_deg + 180.0) % 360.0 - 180.0) <= half
        vals = np.where(np.isfinite(self.smooth), self.smooth, 0.0)[sel]
        if vals.size == 0:
            return None
        return float(np.quantile(vals, 0.25))

    def points_base(self):
        """라이다 점들을 로봇(base_link) 좌표로. 잡음 제거된 값만 쓴다."""
        rng = self.sharp if self.sharp is not None else self.smooth
        if self.scan is None or rng is None:
            return None
        ang = np.radians(self._angles())
        r = rng
        ok = np.isfinite(r)
        if not ok.any():
            return None
        return np.stack([r[ok] * np.cos(ang[ok]) + LIDAR_X,
                         r[ok] * np.sin(ang[ok])], axis=1)

    def will_hit(self, lin, ang, horizon=0.8, margin=0.020):
        """지금 이 명령대로 horizon 초 동안 가면 몸이 벽에 닿는가.

        반경 거리로 판정하면 '옆으로 스쳐 지나갈 수 있는 벽' 까지 위험으로
        본다. 실제로 8회차에서 로봇 +40도 옆의 벽 때문에 모퉁이를 못 돌고
        전진과 후진을 무한히 반복했다. 위험한지 아닌지는 거리가 아니라
        '지금 가려는 궤적이 몸을 그 벽에 닿게 하는가' 로 판단해야 한다."""
        pts = self.points_base()
        if pts is None or (abs(lin) < 1e-4 and abs(ang) < 1e-4):
            return False
        for t in np.linspace(horizon / 8.0, horizon, 8):
            if abs(ang) < 1e-3:
                cx, cy, psi = lin * t, 0.0, 0.0
            else:
                psi = ang * t
                cx = lin / ang * math.sin(psi)
                cy = lin / ang * (1.0 - math.cos(psi))
            c, sn = math.cos(-psi), math.sin(-psi)
            dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
            qx = dx * c - dy * sn
            qy = dx * sn + dy * c
            inside = ((qx > -TAIL - margin) & (qx < NOSE + margin)
                      & (np.abs(qy) < HALF_W + margin))
            if inside.any():
                return True
        return False

    def start_corner(self):
        """앞이 막혔을 때 쓸 동작을 정한다.

        제자리 회전은 차체 대각 반경(8.5cm)이 그대로 휩쓸고 지나간다.
        폭 30cm 통로에서 그 반경이면 양옆 여유가 6.5cm 밖에 안 남아, 회전만
        해도 벽을 긁는다. 긁히면 스캔 매칭이 어긋나고 맵이 틀어진다.

        라이다가 360도를 보므로 뒤로 물러나는 데 아무 제약이 없다.
        먼저 뒤로 빠져 공간을 만든 다음 돌면 휩쓰는 반경이 벽에 닿지 않는다."""
        # 후보를 놓고 따져본다.
        #   (1) 그 동작을 지금 내면 몸이 벽에 닿는가  -> will_hit
        #   (2) 그 동작을 마치면 앞이 트이는가        -> 그 방향의 room
        # '뒤에 공간 있으면 후진' 같은 단순 기준으로는, 돌면 그만인 자리에서도
        # 굳이 후진하고 회전하면 긁히는 자리에서도 회전한다.
        w = self.args.turn_speed
        v = self.args.speed * 0.6
        turn_deg = 70.0                     # 회전 후 향하게 될 대략의 방향
        cands = []
        for sign, name in ((+1, '왼쪽'), (-1, '오른쪽')):
            opens = self.room(turn_deg * sign) or 0.0
            hit = self.will_hit(0.0, sign * w)
            cands.append(('turn', sign, opens, hit,
                          f'{name} 회전(그쪽 {opens:.2f}m)'))
        back = self.room(180.0) or 0.0
        cands.append(('back', 0, back, self.will_hit(-v, 0.0),
                      f'후진(뒤 {back:.2f}m)'))

        # 부딪치지 않으면서 가장 트인 쪽을 고른다.
        safe = [c for c in cands if not c[3] and c[2] > self.args.back_room]
        pick = max(safe, key=lambda c: c[2]) if safe else None
        if pick is None:
            # 어느 쪽도 마땅치 않다. 그래도 안 닿는 회전이 있으면 그걸 쓴다.
            rot = [c for c in cands if c[0] == 'turn' and not c[3]]
            pick = max(rot, key=lambda c: c[2]) if rot else cands[0]

        if pick[0] == 'back':
            # 후진 1초는 4cm 다. 막다른 통로에서는 4cm 물러나 봐야 여전히
            # 막다른 곳이라, 곧 회전으로 넘어가 '회전만 반복' 이 된다
            # (실물 402회차: `앞 0.22 앞막힘>회전` 이 30초).
            # 모퉁이 상태가 오래 이어지고 있으면 작정하고 길게 물러난다.
            long_stuck = (self.corner_since
                          and time.time() - self.corner_since > 6.0)
            self.corner = {'phase': 'back',
                           'until': time.time() + (3.5 if long_stuck else 1.0),
                           'why': pick[4] + (' (길게)' if long_stuck else '')}
        else:
            self.turn_sign = pick[1]
            self.corner = {'phase': 'turn', 'until': time.time() + 2.5,
                           'why': pick[4]}
        return self.corner

    def corner_step(self):
        """진행 중인 '후진 후 회전' 을 한 걸음 내보낸다. 끝났으면 None."""
        c = self.corner
        if c is None:
            return None
        if time.time() >= c['until']:
            if c['phase'] == 'back':
                # 물러났으니 이제 돈다. 이때는 여유가 생겨 안 긁힌다.
                c['phase'] = 'turn'
                c['until'] = time.time() + 2.0
                c['why'] = '물러난 자리에서 회전'
            else:
                self.corner = None
                return None
        if c['phase'] == 'back':
            # 뒤가 정말 막혔을 때만 회전으로 넘어간다. will_hit 은 여유
            # 2cm 를 기준으로 삼아 통로에서 늘 참이 되기 쉬운데, 그러면
            # 후진이 시작도 못 하고 회전만 남는다.
            g, _ = self.clearance_min()
            blocked = (g is not None and g <= 0.0) or \
                self.will_hit(-self.args.speed * 0.6, 0.0, margin=0.004)
            if blocked:
                c['phase'] = 'turn'
                c['until'] = time.time() + 2.0
            else:
                return (-self.args.speed * 0.6, 0.0)
        # 회전 중에도 앞이 충분히 트이면 그만 돈다
        front = self.beam(0.0) or 0.0
        if front > self.args.front_stop * 1.6:
            self.corner = None
            return None
        # 여유가 있으면 아주 느리게 전진을 섞어 '호'를 그리며 돈다.
        # 순수 제자리 회전만 하면 좌수법이 모퉁이를 돌아 나가지 못하고,
        # 개구부 좌회전과 서로 상쇄되어 제자리에서 맴돈다(32회차에서 1m).
        creep = self.args.speed * 0.25 if front > self.args.front_stop else 0.0
        sign = self.turn_sign if self.turn_sign is not None else -self.hand
        return (creep, sign * self.args.turn_speed)

    def start_escape(self, reason, seconds=1.8):
        """빠져나갈 방향을 '빈 공간' 을 보고 정한 뒤 그 동작을 붙잡아 둔다.

        예전에는 가장 가까운 장애물의 각도 부호만으로 매 주기 방향을 다시
        골랐다. 그러면 장애물이 정확히 앞뒤(+-180도 부근)에 있을 때 부호가
        계속 뒤집혀 좌회전과 우회전이 번갈아 나오고, 로봇은 제자리에서 떨기만
        한다. 실제로 5회차에서 30초 동안 그 상태로 멈춰 있었다.
        그래서 (1) 어디가 비었는지 보고 (2) 한 번 정하면 일정 시간 밀어붙인다."""
        fwd = self.room(0.0) or 0.0
        back = self.room(180.0) or 0.0
        left = self.room(90.0) or 0.0
        right = self.room(-90.0) or 0.0
        turn = 1.0 if left > right else -1.0
        # 후보 동작을 만들어 두고, 실제로 부딪치지 않는 것을 고른다.
        v, w = self.args.speed * 0.55, self.args.turn_speed
        cands = []
        if fwd >= back:
            cands += [(v, turn * w), (v, -turn * w)]
        # 똑바로 물러나기를 회전 섞인 후진보다 먼저 본다. 돌면서 물러나면
        # 차체 모서리가 벽을 쓸지만, 똑바로 물러나면 쓸지 않는다.
        # 320회차에서 왼쪽 1.5cm 에 끼었을 때 회전이 섞인 후보가 전부
        # '경로막힘' 으로 걸려 30초 동안 제자리 회전만 했다.
        cands += [(-v, 0.0), (-v, -turn * w), (-v, turn * w)]
        if fwd > back:
            cands += [(v, 0.0), (v, turn * w)]
        cands += [(0.0, turn * w), (0.0, -turn * w)]
        pick = next((c for c in cands if not self.will_hit(*c)), None)
        if pick is None:
            # 평소 여유(2cm)로는 어느 쪽도 안전하지 않다. 하지만 이미 벽에서
            # 1.5cm 인 상태에서 '전부 위험' 은 아무것도 안 하는 것과 같고,
            # 제자리 회전은 옆 여유를 1mm 도 늘려주지 못한다. 여유 기준을
            # 낮춰 '그나마 덜 위험한' 동작을 고른다.
            pick = next((c for c in cands
                         if not self.will_hit(*c, margin=0.004)), None)
        if pick is None:
            # 어느 쪽도 안전하지 않다. 제자리에서 넓은 쪽으로 돌아 길을 만든다.
            lin, turn, seconds = 0.0, turn, 2.5
        else:
            lin = pick[0] / v if v else 0.0
            turn = pick[1] / w
            if lin == 0.0:
                # 회전만으로 빠져나오려면 한 방향으로 충분히 돌아야 한다.
                # 2.5초(약 65도)로는 모자라 방향만 바꾸다 끝났다.
                # 4.5초면 약 115도까지 돈다.
                seconds = 4.5
        self.escape = {
            'until': time.time() + seconds,
            'began': time.time(),
            'lin': lin * self.args.speed * 0.55,
            'ang': turn * self.args.turn_speed,
            'why': f'{reason} 앞{fwd:.2f} 뒤{back:.2f} 좌{left:.2f} 우{right:.2f}'
                   f' -> {"회전만" if lin == 0 else ("전진" if lin > 0 else "후진")}',
        }
        return self.escape

    def escape_step(self):
        """진행 중인 탈출 동작을 한 걸음 내보낸다. 끝났으면 None."""
        e = self.escape
        if e is None:
            return None
        if time.time() >= e['until']:
            self.escape = None
            return None
        # 탈출 방향은 시작할 때 한 번 고르고 1.8~2.5초를 밀어붙인다. 그
        # 사이에 몸이 벽으로 파고들어도 멈추지 않는 것이 문제였다(실물
        # 396회차 계측: 후진 중 여유 0.8 -> -0.5cm 로 접촉). 매 걸음 실제
        # 여유를 보고, 닿았거나 뚜렷이 나빠지면 접고 다시 고른다.
        # 각도 규약을 따지지 않고 '재 본 값' 만 쓰므로 틀릴 여지가 없다.
        g, _ = self.clearance_min()
        if g is not None:
            if 'best' not in e:
                e['best'] = g
            # 판정을 동작 종류에 따라 다르게 한다.
            #  - 직진/후진: 여유가 줄면 그 방향이 벽 쪽이다. 바로 접는다.
            #  - 제자리 회전: 모서리를 쓸며 여유가 '잠깐' 줄었다가 트이는
            #    것이 정상이다. 줄었다고 접으면 오른쪽으로 돌다 취소하고
            #    왼쪽으로 돌기를 반복해 영영 못 빠져나온다(실물에서 관찰).
            #    그래서 회전은 실제로 닿았을 때만 접는다.
            turning = abs(e['lin']) < 1e-6
            if g <= 0.0 or (not turning and g < e['best'] - 0.008):
                self.escape = None
                return None
            e['best'] = max(e['best'], g)
        # 가려던 쪽이 도중에 막히면 접고 다시 고른다. 다만 시작하자마자
        # 접으면 매 주기 방향을 새로 고르는 예전 문제로 되돌아가므로,
        # 최소 0.6초는 밀어붙인 뒤에만 판단한다.
        if e['lin'] != 0.0 and time.time() - e['began'] > 0.6:
            ahead = self.room(0.0 if e['lin'] > 0 else 180.0) or 0.0
            if ahead < 0.10:
                self.escape = None
                return None
        return e

    def make_start_room(self, timeout=18.0):
        """출발 자리가 좁으면 앞이 트인 방향으로 돌아서고 시작한다.

        사람이 로봇을 모서리(예: 18번 구역)에 놓으면 첫 순간부터 '앞막힘'
        이다. 모퉁이 동작(start_corner)은 후보를 매 주기 다시 고르기 때문에,
        어느 쪽도 확실히 낫지 않은 모서리에서는 회전과 후진을 번갈아 내며
        제자리에 머문다(324회차: 30초 동안 0.53m 이동, 관측 +32칸).

        여기서는 (1) 뒤가 트였으면 조금 물러나 회전 반경을 만들고
        (2) 가장 트인 방향으로 앞이 실제로 트일 때까지 한 방향으로만 돈다.
        시작할 때 한 번만 쓴다."""
        front = self.beam(0.0)
        gap, gap_deg = self.clearance_min()
        # 기준을 0.02 로 두면 안 된다. 라이다로 잰 이 값은 차체를 감싸는
        # 직사각형으로 재기 때문에 실제보다 1~3cm 비관적이다(가제보 실제
        # 여유와 나란히 재서 확인). 그 기준으로는 여유 4~5cm 인 정상 통로에서도
        # '접촉' 으로 보고 아래의 직진 탈출이 발동한다. 실제로 350회차 이후
        # 충돌이 몰렸다. 진짜로 파고든 경우(음수)만 다룬다.
        pinched = gap is not None and gap < 0.0
        if (front is None or front >= self.args.front_stop) and not pinched:
            return False
        twist = Twist()
        if pinched:
            # 옆구리가 이미 닿아 있으면 도는 것은 최악이다(대각 반경 8.5cm 가
            # 반폭 6.35cm 보다 크다). 통로 방향으로 곧게 빠져나오는 게 먼저다.
            # 355회차 접촉 뒤 356회차가 그 자세로 시작해 즉시 또 닿았다.
            print(f'  몸이 벽에 닿아 있습니다 ({gap*100:+.1f}cm, '
                  f'{gap_deg:+.0f}도 방향). 곧게 빠져나옵니다.')
            for lin, name in ((-self.args.speed, '뒤로'),
                              (self.args.speed, '앞으로')):
                t0 = time.time()
                improved = False
                while rclpy.ok() and time.time() - t0 < 2.5:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    g, _ = self.clearance_min()
                    # 3.5cm 로는 부족했다. 그 여유로 벽추종을 시작하면 곧
                    # 다시 닿는다(366회차: +3.6cm 확보 직후 접촉).
                    if g is not None and g >= 0.055:
                        improved = True
                        break
                    # 눈 감고 17cm 를 직진하면 안 된다. 매 걸음 앞을 본다.
                    if self.will_hit(lin, 0.0, horizon=0.5, margin=0.004):
                        break
                    twist.linear.x = lin
                    twist.angular.z = 0.0
                    self.pub.publish(twist)
                self.stop()
                if improved:
                    print(f'    {name} 빠져나와 여유 {g*100:+.1f}cm 확보')
                    break
            front = self.beam(0.0)
            if front is not None and front >= self.args.front_stop:
                return True
        print(f'  출발 자리가 좁습니다 (앞 '
              f'{"?" if front is None else f"{front:.2f}"} m). '
              f'먼저 트인 쪽으로 돌아섭니다.')

        # (1) 뒤로 조금 물러난다. 회전은 차체 대각 반경이 그대로 휩쓸기
        #     때문에, 물러나 두면 도는 동안 벽을 긁지 않는다.
        back = self.room(180.0) or 0.0
        if back > self.args.back_room:
            t0 = time.time()
            while rclpy.ok() and time.time() - t0 < 2.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.will_hit(-self.args.speed * 0.6, 0.0, margin=0.010):
                    break
                twist.linear.x = -self.args.speed * 0.6
                twist.angular.z = 0.0
                self.pub.publish(twist)
            self.stop()

        # (2) 가장 트인 방향을 한 번 정하고, 그쪽으로만 돈다.
        best_deg, best_room = None, -1.0
        for d in range(-180, 180, 10):
            r = self.room(float(d)) or 0.0
            if r > best_room:
                best_room, best_deg = r, d
        if best_deg is None:
            return True
        sign = 1.0 if best_deg > 0 else -1.0
        print(f'    {best_deg:+d}도 쪽이 가장 트였습니다 ({best_room:.2f} m)')
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            front = self.beam(0.0)
            if front is not None and front > self.args.front_stop * 1.5:
                self.stop()
                print(f'    앞 {front:.2f} m 확보')
                return True
            twist.linear.x = 0.0
            twist.angular.z = sign * self.args.turn_speed
            self.pub.publish(twist)
        self.stop()
        print('    시간 내 못 트였습니다. 그대로 시작합니다.')
        return True

    def park_clear(self, want=0.085, timeout=25.0):
        """벽에서 떨어진 자리로 빠져나온다.

        벽 따라가기는 일부러 벽에 붙어 다니므로, 그 상태 그대로 Nav2 에
        넘기면 컨트롤러가 '충돌 예상' 으로 아예 움직이질 못한다.
        다음 단계로 넘기기 전에 사방이 트인 자리로 옮겨준다."""
        twist = Twist()
        t0 = time.time()
        print(f'\n다음 단계를 위해 벽에서 떨어진 자리로 이동합니다 '
              f'(목표 여유 {want:.2f} m)')
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            near, near_deg = self.all_min()
            if near is None:
                continue
            if near >= want:
                self.stop()
                print(f'  확보 완료: 최단 거리 {near:.3f} m')
                return True
            front = self.beam(0.0) or 9.9
            # 가장 가까운 벽의 반대쪽으로 돌면서, 앞이 트였으면 전진한다.
            twist.angular.z = math.copysign(self.args.turn_speed,
                                            -near_deg if near_deg else 1.0)
            twist.linear.x = self.args.speed * 0.5 if front > 0.25 else 0.0
            self.pub.publish(twist)
        self.stop()
        near, _ = self.all_min()
        print(f'  시간 내 확보 실패 (최단 거리 {near:.3f} m)' if near
              else '  거리 측정 실패')
        return False

    # ------------------------------------------------------------------
    def run(self):
        target = self.args.wall_distance
        twist = Twist()

        # 라이다와 TF 가 들어올 때까지 대기
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 15:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.pose() is not None:
                break
        start = self.pose()
        if start is None:
            raise RuntimeError('map -> base_link TF 를 못 받았습니다. SLAM 확인.')
        # 좁은 모서리에 놓고 시작하면 앞이 막힌 채로 시작한다. 반드시
        # HOME 을 찍기 '전에' 트인 자리로 나와야 한다. 벽에 붙은 자세를
        # 출발 지점으로 기록하면, 4단계가 그 자리로 돌아가려고 벽을 계속
        # 민다(328회차: 16번 구역에서 실제 접촉).
        if self.make_start_room():
            start = self.pose() or start
        print(f'출발 지점 ({start[0]:.2f}, {start[1]:.2f})')
        # 다른 스크립트(하이브리드)가 이 값을 받아 복귀 목표로 쓴다.
        yaw = self.yaw() or 0.0
        print(f'HOME {start[0]:.4f} {start[1]:.4f} {math.degrees(yaw):.2f}',
              flush=True)
        # 손 선택(auto_hand)이 아래에서 바뀔 수 있으므로 안내는 그 뒤에 찍는다

        # 어느 손으로 따라갈지 정한다.
        # 좌수법은 왼쪽에 벽이 있어야 성립한다. 벽이 오른쪽에만 있는 자리에서
        # 시작하면 왼쪽 벽을 찾겠다고 계속 왼쪽으로 돌며 나선을 그린다.
        # 실물에서 트랙 모서리에 놓고 "빙빙 돌았다"는 게 정확히 이 상황이다.
        # 시작할 때 벽이 있는 쪽을 보고 손을 고르면 바로 따라갈 수 있다.
        if self.args.auto_hand:
            ld, _ = self.side_wall(+1)
            rd, _ = self.side_wall(-1)
            lv = ld if ld is not None else 9.9
            rv = rd if rd is not None else 9.9
            if rv < self.args.lost_wall < lv:
                self.hand = -1
                print(f'  왼쪽에 벽이 없고(={lv:.2f}m) 오른쪽에 있어(={rv:.2f}m) '
                      f'우수법으로 시작합니다')
            else:
                print(f'  좌수법으로 시작합니다 (왼 {lv:.2f}m / 오른 {rv:.2f}m)')

        side_name = '오른' if self.hand < 0 else '왼'
        print(f'{side_name}쪽 벽과 {target:.2f} m 를 유지하며 한 바퀴 돕니다\n')

        if self.enclosed():
            far = float(self.smooth[np.isfinite(self.smooth)].max())
            print(f'\n[중단] 사방이 막힌 좁은 공간입니다 '
                  f'(가장 멀리 보이는 곳도 {far*100:.0f} cm).\n'
                  f'  로봇을 트인 통로로 옮긴 뒤 다시 시작하세요.\n'
                  f'  여기서 계속 돌려도 제자리에서 맴돌기만 합니다.\n'
                  f'  주의: SLAM 이 도는 중에 손으로 옮기면 맵이 망가집니다.\n'
                  f'        옮긴 뒤에는 [맵 초기화] 를 먼저 누르세요.')
            self.stuck_in_pocket = True
            return

        began = time.time()
        travelled = 0.0
        left_home = False
        prev = start
        last_print = 0.0
        # 같은 칸을 몇 번 밟았는지 세어 '뱅뱅 도는' 상태를 잡아낸다.
        visits = {}
        last_cell = None
        cellsize = 0.10
        best_known = self.known
        last_gain = time.time()
        last_health = time.time()
        looped = False
        self.diverged = False
        self.wedged = False
        freed_once = False        # '벽에서 떨어지기' 를 이미 한 번 썼는가
        self.stuck_in_pocket = False
        self.corner = None
        stuck_escape = 0
        self.corner_since = 0   # 모퉁이 상태가 언제부터 이어지나

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - began > self.args.max_time:
                print('\n제한 시간에 도달했습니다.')
                break

            here = self.pose()
            if here is None:
                continue
            travelled += math.dist(here, prev)
            prev = here
            home_dist = math.dist(here, start)

            # 기본 종료 조건은 '맵이 더 이상 안 늘어남' 이다.
            # 출발점 복귀를 종료 조건으로 쓰면 1단계가 끝날 때마다 로봇이
            # 굳이 집으로 돌아왔다가 다음 단계에 다시 나가게 된다.
            # 한 바퀴를 이미 돌았다면 같은 길을 또 도는 것이라 새로 얻을 게
            # 거의 없다. 그때는 훨씬 짧게 기다리고 끝낸다.
            patience = (self.args.idle_after_loop if looped
                        else self.args.idle_time)
            # 아직 1m 도 못 갔으면 '더 볼 게 없다' 가 아니라 '아직 못 나갔다'
            # 이다. 그 상태로 끝내면 훑기가 통째로 날아간다(324회차).
            if travelled < 1.0:
                patience = max(patience, self.args.idle_time * 2.5)
            if self.known > best_known + self.args.idle_gain:
                best_known = self.known
                last_gain = time.time()
            elif time.time() - last_gain > patience:
                print(f'\n{patience:.0f}초 동안 맵이 더 늘지 않아 '
                      f'훑기를 마칩니다 (이동 {travelled:.2f} m, '
                      f'관측 {self.known}칸).')
                break

            # 한 바퀴 완주는 참고용으로만 알리고, 원할 때만 종료한다.
            if not left_home and home_dist > self.args.loop_leave:
                left_home = True
            elif left_home and not looped and home_dist < self.args.loop_close \
                    and travelled > self.args.min_travel:
                looped = True
                print(f'  (한 바퀴 돌아 출발점 부근을 지납니다 — '
                      f'이동 {travelled:.2f} m)')
                if self.args.stop_on_loop:
                    print('\n한 바퀴를 돌아 종료합니다.')
                    break

            # 주의: 매 반복마다 세면 제자리에 잠깐 서 있기만 해도 금세 한도를
            # 넘는다. 칸이 '바뀔 때'만 세야 실제 재방문 횟수가 된다.
            key = (round(here[0] / cellsize), round(here[1] / cellsize))
            if key != last_cell:
                last_cell = key
                visits[key] = visits.get(key, 0) + 1
            if visits.get(key, 0) > self.args.max_revisit:
                print(f'\n같은 지점을 {visits[key]}번 지나 제자리를 도는 것으로 '
                      f'판단했습니다 (이동 {travelled:.2f} m).')
                break

            front = self.beam(0.0) or 9.9
            dist, alpha = self.left_wall()
            mode = ''

            # 맵 좌표계가 틀어졌으면 더 돌수록 손해다. 즉시 멈춘다.
            if time.time() - last_health > 8.0:
                last_health = time.time()
                bad, h = self.map_diverged()
                if bad:
                    print(f'\n맵 좌표계가 틀어졌습니다 '
                          f'({h["size_m"][0]} x {h["size_m"][1]} m, '
                          f'벽 두께 {h["wall_thickness_cm"]}cm). 훑기를 중단합니다.')
                    self.diverged = True
                    break

            # --- 탈출 동작 ---------------------------------------------
            # 몸이 벽에 닿았거나 닿기 직전이면 벽 추종을 중단하고 빠져나온다.
            # 이 동작은 한 번 정하면 붙잡아 둔다(escape_step). 매 주기 새로
            # 고르면 정면/후면에서 각도 부호가 뒤집히며 좌우로 떨기만 한다.
            e = self.escape_step()
            if e is None:
                touch = self.contact_dir()
                if touch is not None:
                    e = self.start_escape(f'접촉({touch:+.0f}도)')
                if e is not None and time.time() - last_print >= 1.0:
                    last_print = time.time()
                    print(f'  ({here[0]:+.2f},{here[1]:+.2f}) 이동 {travelled:5.2f}m '
                          f'탈출> {e["why"]}')
            if e is not None:
                # 제한은 '반복 횟수' 가 아니라 '시간' 이어야 한다. 루프 속도는
                # 콜백 처리량에 따라 크게 달라져서, 횟수로 재면 몇 초 만에
                # 한도를 넘어 멀쩡한 탈출까지 실패로 처리해 버린다.
                if stuck_escape == 0:
                    stuck_escape = time.time()
                elif time.time() - stuck_escape > self.args.escape_timeout:
                    if not freed_once:
                        # 포기하기 전에 '벽에서 떨어지기' 를 한 번 쓴다.
                        # 이 동작은 충돌 예측을 보지 않고 가장 가까운 벽의
                        # 반대로 돌며 앞이 트이면 전진한다. 320회차에서 탈출은
                        # 30초 내내 '회전만' 이었는데, 같은 자리에서 이 동작은
                        # 몇 초 만에 8.8cm 를 확보했다.
                        freed_once = True
                        stuck_escape = 0
                        self.escape = None
                        print(f'\n{self.args.escape_timeout:.0f}초 동안 못 '
                              f'빠져나왔습니다. 벽에서 떨어지기를 시도합니다.')
                        self.park_clear(
                            want=max(0.085, self.args.side_margin * 3),
                            timeout=12.0)
                        continue
                    print(f'\n{self.args.escape_timeout:.0f}초 동안 탈출을 '
                          f'반복해도 못 빠져나왔습니다 '
                          f'(이동 {travelled:.2f} m). 훑기를 멈춥니다.')
                    self.wedged = True
                    break
                twist.linear.x = e['lin']
                twist.angular.z = e['ang']
                self.last_lin = twist.linear.x
                self.pub.publish(twist)
                continue
            stuck_escape = 0

            # 모퉁이 동작만 반복되는데 앞이 안 트이면 막다른 곳이다.
            # 회전을 아무리 해도 안 나오므로 후진으로 빠져나온다.
            # 모퉁이 상태가 얼마나 이어지고 있는지 기억해 둔다.
            # start_corner 가 이 값을 보고 후진 길이를 정한다.
            if self.corner is not None:
                if not self.corner_since:
                    self.corner_since = time.time()
                elif time.time() - self.corner_since > 15.0:
                    # 15초 넘게 모퉁이만 반복하면 짧은 회전으로는 안 되는
                    # 자리다. 실물에서 1단계는 못 나왔는데 2단계로 넘어가
                    # Nav2 가 180도를 돌자 바로 빠져나왔다 — 즉 '충분히
                    # 크게 한 방향으로 도는 것' 이 답이다.
                    # 그 동작은 이미 있다: make_start_room 은 가장 트인
                    # 쪽을 한 번 정하고 앞이 실제로 트일 때까지 돈다.
                    # (출발할 때 쓰던 것을 그대로 쓴다)
                    print('  모퉁이에서 15초째 못 나갑니다. '
                          '트인 쪽으로 크게 돌아섭니다.')
                    self.corner = None
                    self.corner_since = 0
                    self.escape = None
                    self.make_start_room(timeout=20.0)
                    continue
            else:
                self.corner_since = 0

            # 진행 중인 모퉁이 동작이 있으면 그것부터 끝낸다.
            # 앞 거리가 front_stop 근처에서 오르내리면, 매 주기 분기가 바뀌어
            # 전진(벽추종)과 후진(앞막힘)이 번갈아 나오며 제자리를 지킨다.
            # 실제로 38회차에서 앞 0.19~0.21 사이를 오가며 0.86m 만에 멈췄다.
            # 한 번 시작한 모퉁이 동작은 앞이 확실히 트일 때까지 이어간다.
            cmd = self.corner_step()
            if cmd is not None:
                mode = '앞막힘>후진' if cmd[0] < 0 else '앞막힘>회전'
                twist.linear.x, twist.angular.z = cmd
                self.last_lin = twist.linear.x
                self.pub.publish(twist)
                if time.time() - last_print >= 2.0:
                    last_print = time.time()
                    print(f'  ({here[0]:+.2f},{here[1]:+.2f}) 이동 {travelled:5.2f}m '
                          f'앞 {front:4.2f} {mode}')
                continue

            # 따라가는 쪽만 보면 반대쪽 벽에 부딪친다. 넓은 곳에서 원을 그리다
            # 반대쪽 벽으로 밀고 들어가는 일이 실제로 있었다. 양쪽을 다 본다.
            sl, _a = self.clearance_min(20.0, 160.0)
            sr, _b = self.clearance_min(-160.0, -20.0)
            cands = [v for v in (sl, sr) if v is not None]
            side = min(cands) if cands else None
            side_sign = (1 if (sl is not None and side == sl) else -1)
            if side is not None and side < self.args.side_margin:
                # 왼쪽 옆구리가 벽에 닿기 직전이다. 전진을 멈추고 오른쪽으로
                # 떼어낸다. 이 검사가 없으면 모서리에서 벽을 파고든다.
                # 차동 구동은 옆으로 못 움직인다. 제자리 회전이나 후진만으로는
                # 옆 벽에서 멀어지지 않으므로, 오른쪽으로 틀면서 '전진' 해야
                # 호를 그리며 벽에서 벗어난다. 앞이 막혔을 때만 후진한다.
                near = '왼' if side_sign > 0 else '오른'
                mode = f'벽밀착({near} {side*100:+.1f}cm)>탈출'
                if front > 0.22:
                    twist.linear.x = self.args.speed * 0.6
                else:
                    twist.linear.x = -self.args.speed * 0.4
                # 가까운 쪽의 반대로 틀어야 벽에서 멀어진다
                twist.angular.z = -side_sign * self.args.turn_speed
            elif front < self.args.front_stop:
                # 앞이 막혔다. 바로 돌지 않고 먼저 뒤로 물러난다.
                # (start_corner 주석 참고 — 제자리 회전이 벽을 긁는 주범이다)
                self.start_corner()
                cmd = self.corner_step() or (0.0, -self.hand * self.args.turn_speed)
                mode = ('앞막힘>후진' if cmd[0] < 0 else '앞막힘>회전')
                twist.linear.x, twist.angular.z = cmd
            elif dist is None or dist > self.args.lost_wall:
                # 왼쪽 벽이 끊겼다 = 개구부(통로)다. 좌수법은 그 안으로 들어간다.
                # 직진을 충분히 섞어야 제자리에서 뱅뱅 돌지 않는다.
                # 왼쪽이 트였다 = 갈림길이다. 좌수법은 그 안으로 들어간다.
                # 다만 앞왼쪽 모서리가 걸릴 만큼 좁으면 좌회전이 곧 충돌이다.
                # 그때는 일단 곧게 지나간 뒤 다음 기회에 돌아 들어간다.
                if self.opening_since is None:
                    self.opening_since = time.time()
                held = time.time() - self.opening_since
                if held > self.args.seek_after:
                    # 벽이 이만큼 오래 안 잡히면 넓은 공간에서 원을 그리는 중이다.
                    # 좌수법/우수법은 벽이 있어야 성립하므로, 계속 돌아봐야
                    # 같은 원만 그린다. 먼저 가장 가까운 벽으로 곧장 간다.
                    tdir, tdist = self.wall_to_seek()
                    if tdir is not None:
                        err = (tdir + 180) % 360 - 180
                        mode = f'벽찾기>{tdir:+.0f}도 ({tdist*100:.0f}cm)'
                        twist.angular.z = max(-self.args.turn_speed,
                                              min(self.args.turn_speed,
                                                  math.radians(err) * 1.2))
                        # 방향이 얼추 맞으면 그쪽으로 간다. 많이 틀어졌으면
                        # 멈춰 서서 돌지 말고 천천히라도 나아간다.
                        twist.linear.x = (self.args.speed if abs(err) < 35
                                          else self.args.speed * 0.3)
                    else:
                        # 붙을 만한 벽이 안 보이면 가장 트인 쪽으로 나아간다
                        mode = '벽찾기>트인쪽'
                        twist.linear.x = self.args.speed
                        twist.angular.z = 0.0
                elif (self.room(45.0 * self.hand) or 0.0) > 0.20:
                    mode = f'개구부>{"좌" if self.hand > 0 else "우"}회전'
                    twist.linear.x = self.args.speed * 0.7
                    twist.angular.z = self.hand * self.args.turn_speed * 0.6
                else:
                    mode = '개구부>직진(모서리 좁음)'
                    twist.linear.x = self.args.speed * 0.7
                    twist.angular.z = 0.0
            else:
                self.opening_since = None      # 벽을 다시 잡았다
                # 폭 30cm 통로에 몸통 12.7cm 라 좌우 여유가 8.7cm 씩뿐이다.
                # 한쪽 벽만 보고 목표 거리를 맞추면, 그 추정이 1cm만 틀려도
                # 반대쪽으로 쏠려 모서리를 긁는다. 양쪽 벽이 다 보이면
                # '한가운데'를 잡는 편이 훨씬 안정적이다. 좌수법은 갈림길에서
                # 왼쪽을 고르는 규칙이지, 벽에 붙어 가라는 뜻이 아니다.
                rdist, _ = self.side_wall(-self.hand)
                if rdist is not None and dist + rdist < self.args.corridor:
                    err = rdist - dist          # >0 이면 왼쪽에 붙어 있다
                    steer = -self.args.kc * err + self.args.kd * alpha
                    mode = '통로중앙'
                else:
                    # 오른쪽이 트여 있으면 예전처럼 왼쪽 벽을 따라간다.
                    err = target - dist
                    steer = -self.args.kp * err + self.args.kd * alpha
                    mode = '벽추종'
                # 앞이 좁을수록 천천히 간다. 같은 조향이라도 느리면
                # 보정할 시간이 생겨 모서리를 덜 긁는다.
                scale = max(0.35, min(1.0, (front - 0.12) / 0.25))
                twist.linear.x = self.args.speed * scale
                # 반대로 앞이 넓게 트였으면 빨리 간다. 좁은 통로(트랙)에서는
                # 위 식이 1.0 을 못 넘어 예전 속도 그대로이고, 넓은 곳
                # (강의장)에서만 빨라져 매핑 시간이 줄어든다.
                if scale >= 1.0 and front > self.args.open_front:
                    room = (front - self.args.open_front) / 0.6
                    twist.linear.x = min(self.args.max_speed,
                                         self.args.speed * (1.0 + room))
                twist.angular.z = max(-self.args.turn_speed,
                                      min(self.args.turn_speed, steer))
            # 내리려는 명령이 실제로 몸을 벽에 닿게 하는지 궤적으로 본다.
            # 반경 거리로 재면 옆으로 스쳐 지나갈 벽까지 위험으로 잡혀
            # 모퉁이를 영영 못 돈다.
            if self.will_hit(twist.linear.x, twist.angular.z):
                # 곧바로 후진 탈출로 가지 말고, 먼저 '덜 꺾어서' 지나갈 수
                # 있는지 본다. 벽 추종 중에는 기울기 보정이 커져 조향이
                # 과격해지는데, 그때마다 후진하면 앞으로 못 나간다
                # (45회차에서 0.78m 만에 멈췄다).
                soft = None
                for ka, kv in ((0.5, 1.0), (0.25, 1.0), (0.0, 1.0),
                               (0.5, 0.5), (0.0, 0.5)):
                    a2 = twist.angular.z * ka
                    v2 = twist.linear.x * kv
                    if not self.will_hit(v2, a2):
                        soft = (v2, a2, ka, kv)
                        break
                if soft is not None:
                    twist.linear.x, twist.angular.z = soft[0], soft[1]
                    mode += f'(조향{soft[2]:.0%}'
                    mode += f'/속도{soft[3]:.0%})' if soft[3] < 1 else ')'
                    self.last_lin = twist.linear.x
                    self.pub.publish(twist)
                    if time.time() - last_print >= 2.0:
                        last_print = time.time()
                        print(f'  ({here[0]:+.2f},{here[1]:+.2f}) '
                              f'이동 {travelled:5.2f}m 앞 {front:4.2f} {mode}')
                    continue
                e = self.start_escape(f'{mode} 경로막힘')
                if e is not None:
                    if time.time() - last_print >= 1.0:
                        last_print = time.time()
                        print(f'  ({here[0]:+.2f},{here[1]:+.2f}) '
                              f'이동 {travelled:5.2f}m 탈출> {e["why"]}')
                    twist.linear.x, twist.angular.z = e['lin'], e['ang']
                    stuck_escape = stuck_escape or time.time()

            self.last_lin = twist.linear.x
            self.pub.publish(twist)

            if time.time() - last_print >= 2.0:
                last_print = time.time()
                wall_lbl = '오른벽' if self.hand < 0 else '왼벽'
                ds = f'{dist:4.2f}' if dist is not None else ' -- '
                al = f'{math.degrees(alpha):+5.0f}' if alpha is not None else '  -- '
                sd = f'{side*100:+5.1f}' if side is not None else '  -- '
                print(f'  ({here[0]:+.2f},{here[1]:+.2f}) 이동 {travelled:5.2f}m '
                      f'앞 {front:4.2f} {wall_lbl} {ds} 여유 {sd}cm 기울기 {al}도 '
                      f'출발점 {home_dist:4.2f}  {mode}')

        self.stop()
        if not self.args.no_park:
            self.park_clear(want=self.args.park_clearance)
        return travelled


def save_map(path):
    path = path.replace('~', __import__('os').path.expanduser('~'))
    print(f'\n맵을 저장합니다: {path}.yaml / .pgm')
    r = subprocess.run(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path,
         '--ros-args', '-p', 'save_map_timeout:=20.0'],
        capture_output=True, text=True, timeout=60)
    print('저장 완료' if r.returncode == 0 else f'저장 실패:\n{r.stderr.strip()}')


def main():
    p = argparse.ArgumentParser(description='Pinky Pro 좌수법 벽 따라가기 매핑')
    p.add_argument('--sim', action='store_true', help='Gazebo 시뮬레이션에서 실행')
    p.add_argument('--save', default='~/pinky/map_wallfollow',
                   help='맵 저장 경로(확장자 제외)')
    p.add_argument('--no-save', action='store_true')
    p.add_argument('--wall-distance', type=float, default=0.15,
                   help='왼쪽 벽과 유지할 거리(m), 기본 0.16. '
                        '로봇 반폭이 0.064m 라 너무 붙이면 모서리에서 닿는다')
    p.add_argument('--no-park', action='store_true',
                   help='끝날 때 벽에서 떨어진 자리로 옮기지 않는다')
    p.add_argument('--park-clearance', type=float, default=0.085,
                   help='다음 단계로 넘기기 전 확보할 최단 여유(m), 기본 0.085. '
                        '벽에 붙은 채로 넘기면 Nav2 가 충돌로 보고 못 움직인다. '
                        '통로 폭 0.30m 에서 얻을 수 있는 최대치가 0.087m 이므로 '
                        '그보다 크게 잡으면 통로 안에서는 영영 달성할 수 없다')
    p.add_argument('--side-margin', type=float, default=0.030,
                   help='왼쪽 옆구리에 남겨둘 최소 여유(m). 차체 끝 기준 '
                        '(기본 0.030)')
    p.add_argument('--contact-margin', type=float, default=0.005,
                   help='제자리 회전 중에 적용할 최소 여유(m). 회전 중에는 '
                        '모서리가 벽을 스쳐도 실제로 다가가는 게 아니다 '
                        '(기본 0.005)')
    p.add_argument('--safe-margin', type=float, default=0.020,
                   help='몸통에서 벽까지 이보다 여유가 적으면 탈출한다(m). '
                        '라이다 거리가 아니라 차체 끝 기준이다 (기본 0.020)')
    p.add_argument('--escape-timeout', type=float, default=25.0,
                   help='탈출 동작이 이 시간을 넘게 이어지면 끼인 것으로 '
                        '보고 1단계를 접는다(초), 기본 25')
    p.add_argument('--speed', type=float, default=0.07,
                   help='직진 속도(m/s), 기본 0.07. 로그를 보면 가장 아슬아슬한 순간이 모두 전진 중이라(여유 1.6~3.7cm) 조향이 보정할 시간을 준다')
    p.add_argument('--open-front', type=float, default=0.60,
                   help='앞이 이보다 트이면 속도를 올린다(m). 트랙 통로에서는 '
                        '거의 발동하지 않고 넓은 공간에서만 빨라진다')
    p.add_argument('--max-speed', type=float, default=0.18,
                   help='넓은 곳에서의 속도 상한(m/s). --speed 이하로 두면 '
                        '가속 없이 예전과 같게 동작한다')
    p.add_argument('--turn-speed', type=float, default=0.45,
                   help='회전 속도 상한(rad/s), 기본 0.45. '
                        '빠른 제자리 회전은 스캔 매칭을 깨뜨려 맵을 망친다')
    p.add_argument('--idle-time', type=float, default=15.0,
                   help='이 시간 동안 맵이 안 늘면 훑기를 끝낸다(초), 기본 15')
    p.add_argument('--idle-after-loop', type=float, default=6.0,
                   help='한 바퀴를 이미 돈 뒤의 대기 시간(초), 기본 6. '
                        '같은 길을 또 도는 것이라 짧게 잡는다')
    p.add_argument('--idle-gain', type=int, default=15,
                   help='관측 칸이 이만큼 늘어야 "진척 있음"으로 본다, 기본 15')
    p.add_argument('--stop-on-loop', action='store_true',
                   help='한 바퀴 돌아 출발점에 닿으면 곧바로 종료한다. '
                        '기본은 종료하지 않고 계속 훑는다')
    p.add_argument('--max-revisit', type=int, default=25,
                   help='같은 10cm 칸을 이 횟수 넘게 지나면 갇힌 것으로 보고 종료')
    p.add_argument('--front-stop', type=float, default=0.20,
                   help='정면이 이 거리보다 가까우면 제자리 회전(m), 기본 0.20')
    p.add_argument('--lost-wall', type=float, default=0.22,
                   help='왼쪽 벽 수직거리가 이보다 멀면 개구부로 보고 들어간다(m)')
    p.add_argument('--no-auto-hand', dest='auto_hand',
                   action='store_false',
                   help='시작할 때 벽이 있는 쪽을 보고 좌/우수법을 '
                        '자동으로 고르는 기능을 끈다')
    p.add_argument('--right', action='store_true',
                   help='오른쪽 벽을 따라간다(우수법). 좌수법으로 한 바퀴 돈 뒤\n'
                        '되짚을 때 쓰면 벽의 반대면을 훑는다')
    p.add_argument('--back-room', type=float, default=0.16,
                   help='뒤에 이만큼 여유가 있으면 앞이 막혔을 때 '
                        '회전 대신 먼저 후진한다(m), 기본 0.16')
    p.add_argument('--seek-after', type=float, default=5.0,
                   help='벽을 이 시간 넘게 못 잡으면 넓은 공간에서 '
                        '원을 그리는 것으로 보고 가장 가까운 벽을 '
                        '찾아간다(초), 기본 5')
    p.add_argument('--corridor', type=float, default=0.45,
                   help='좌우 벽 거리의 합이 이보다 작으면 통로로 보고 '
                        '한가운데를 잡는다(m), 기본 0.45')
    p.add_argument('--kc', type=float, default=3.0,
                   help='통로 중앙 유지 이득, 기본 3.0')
    p.add_argument('--kp', type=float, default=4.0,
                   help='벽 거리 오차 보정 이득, 기본 4.0')
    p.add_argument('--kd', type=float, default=1.2,
                   help='벽 기울기 보정 이득, 기본 1.2')
    p.add_argument('--max-time', type=float, default=600.0,
                   help='전체 제한 시간(초), 기본 600')
    p.add_argument('--loop-leave', type=float, default=0.50,
                   help='출발점에서 이만큼 멀어져야 "떠났다"고 본다(m)')
    p.add_argument('--loop-close', type=float, default=0.25,
                   help='출발점 이 거리 안에 들어오면 한 바퀴 완주(m)')
    p.add_argument('--min-travel', type=float, default=2.0,
                   help='최소 이 거리는 달려야 완주로 인정(m)')
    args = p.parse_args()

    if args.sim:
        rclpy.init(args=['pinky_wallfollow',
                         '--ros-args', '-p', 'use_sim_time:=true'])
        print('시뮬레이션 모드 (use_sim_time=true)')
    else:
        rclpy.init()

    node = WallFollower(args)
    code = 0
    try:
        travelled = node.run()
        print(f'총 이동 거리 {travelled:.2f} m')
    except KeyboardInterrupt:
        node.stop()
        print('\n사용자 중단')
        code = 130
    except Exception as e:
        node.stop()
        print(f'오류: {e}', file=sys.stderr)
        code = 1
    finally:
        if not args.no_save:
            try:
                save_map(args.save)
            except Exception as e:
                print(f'저장 중 오류: {e}', file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
