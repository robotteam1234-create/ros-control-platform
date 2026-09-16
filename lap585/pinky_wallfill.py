#!/usr/bin/env python3
"""
끊어진 벽 메우기 (Wall gap filling)
====================================
프론티어 탐사는 '미탐색 공간'을 찾는다. 그런데 벽선 중간이 몇 칸 비어 있는
경우는 주변이 이미 다 관측돼 있어서 프론티어로 잡히지 않는다. 웹 뷰어에서
검정 벽이 뚝뚝 끊겨 보이는 게 이 경우다.

여기서는 '벽의 구멍'을 직접 찾는다.
  - 어떤 칸이 미탐색(-1)인데, 서로 마주보는 두 방향 모두 가까이에 벽이 있으면
    그 칸은 벽이어야 하는데 아직 못 본 자리로 본다.
  - 그 구멍을 볼 수 있는 지점(가깝고, 시야가 트여 있고, 로봇이 갈 수 있는 곳)을
    골라 이동한다. 라이다가 그 자리를 훑으면 벽이 채워진다.

  python3 pinky_wallfill.py --sim
"""

import argparse
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSHistoryPolicy, QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

MAP_QOS = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WallFiller:

    def __init__(self, navigator, args):
        self.nav = navigator
        self.args = args
        self.grid = None
        self.res = None
        self.origin = None
        self.costmap = None
        self.tried = []
        self.scan_msg = None
        self._near = {}

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, navigator)
        navigator.create_subscription(OccupancyGrid, '/map', self._map_cb,
                                      MAP_QOS)
        navigator.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._cm_cb, MAP_QOS)
        # 3단계도 Nav2 로 주행한다. 그런데 라이다를 아예 안 보고 있었다.
        # 맵이 조금이라도 틀어지면 Nav2 는 그 맵을 믿고 벽으로 몬다
        # (365회차: 17번 구역에서 접촉). 다른 단계와 같은 기준으로 감시한다.
        navigator.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self._cmd = navigator.create_publisher(Twist, 'cmd_vel', 10)

    def _scan_cb(self, msg):
        self.scan_msg = msg

    def too_close(self, hard=None, watch=None, drop=0.004):
        """지금 멈춰야 하는가. (여유, 이유). 판정 방식은 탐사 쪽과 같다."""
        import os
        if hard is None:
            hard = float(os.environ.get('PINKY_HARD_GAP', 0.015))
        if watch is None:
            watch = float(os.environ.get('PINKY_WATCH_GAP', 0.030))
        from pinky_wallfollow import body_gap
        gap = body_gap(self.scan_msg)
        if gap is None:
            return None, None
        now = time.time()
        prev, t_prev = self._near.get('gap'), self._near.get('t', 0.0)
        self._near['gap'], self._near['t'] = gap, now
        if gap < hard:
            return gap, '벽에 닿기 직전'
        if (gap < watch and prev is not None and now - t_prev < 0.8
                and prev - gap >= drop):
            return gap, '벽으로 가까워지는 중'
        return gap, None

    def back_off(self, seconds=1.5, speed=0.05):
        """벽에서 조금 물러난다. 전방향 여유를 보며 움직인다."""
        from pinky_wallfollow import body_gap
        twist = Twist()
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < seconds:
            g = body_gap(self.scan_msg)
            if g is not None and g < 0.02:
                break
            twist.linear.x = -abs(speed)
            self._cmd.publish(twist)
            rclpy.spin_once(self.nav, timeout_sec=0.05)
        self._cmd.publish(Twist())

    def _map_cb(self, msg):
        self.res = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def _cm_cb(self, msg):
        self.costmap = msg

    def spin(self, seconds):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def robot_xy(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_link',
                                              rclpy.time.Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # ------------------------------------------------------------------
    def cell(self, x, y):
        c = int((x - self.origin[0]) / self.res)
        r = int((y - self.origin[1]) / self.res)
        return r, c

    def world(self, r, c):
        return (self.origin[0] + (c + 0.5) * self.res,
                self.origin[1] + (r + 0.5) * self.res)

    def clear_line(self, x0, y0, x1, y1):
        """두 점 사이가 벽에 막히지 않았는지 (시야 확보 확인)"""
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / self.res) + 1)
        h, w = self.grid.shape
        for i in range(1, n):
            t = i / n
            r, c = self.cell(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if not (0 <= r < h and 0 <= c < w):
                return False
            if self.grid[r, c] > 50:
                return False
        return True

    def navigable(self, x, y):
        """플래너가 들어갈 수 있는 자리인지 (코스트맵 기준)"""
        cm = self.costmap
        if cm is None:
            return True
        c = int((x - cm.info.origin.position.x) / cm.info.resolution)
        r = int((y - cm.info.origin.position.y) / cm.info.resolution)
        if not (0 <= r < cm.info.height and 0 <= c < cm.info.width):
            return False
        # 99 = inscribed(통행 불가). 92 같은 값은 비용이 높을 뿐 지나갈 수 있다.
        # 기준을 90 으로 두면 좁은 통로의 정상 셀까지 전부 걸러진다.
        return cm.data[r * cm.info.width + c] < 99

    # ------------------------------------------------------------------
    def find_gaps(self):
        """벽이어야 하는데 아직 못 본 칸들을 덩어리로 묶어 돌려준다."""
        g = self.grid
        if g is None:
            return []
        occ = g > 50
        unknown = g == -1
        h, w = g.shape
        span = self.args.gap_span          # 몇 칸 안쪽까지 벽을 찾아볼지

        # 마주보는 방향 네 쌍
        pairs = [((0, 1), (0, -1)), ((1, 0), (-1, 0)),
                 ((1, 1), (-1, -1)), ((1, -1), (-1, 1))]
        gap = np.zeros_like(unknown)
        rows, cols = np.nonzero(unknown)
        for r, c in zip(rows.tolist(), cols.tolist()):
            for (dr1, dc1), (dr2, dc2) in pairs:
                hit1 = hit2 = False
                for k in range(1, span + 1):
                    rr, cc = r + dr1 * k, c + dc1 * k
                    if 0 <= rr < h and 0 <= cc < w and occ[rr, cc]:
                        hit1 = True
                        break
                for k in range(1, span + 1):
                    rr, cc = r + dr2 * k, c + dc2 * k
                    if 0 <= rr < h and 0 <= cc < w and occ[rr, cc]:
                        hit2 = True
                        break
                if hit1 and hit2:
                    gap[r, c] = True
                    break

        # 덩어리로 묶기
        cells = set(zip(*[a.tolist() for a in np.nonzero(gap)]))
        out = []
        while cells:
            seed = cells.pop()
            group = [seed]
            dq = deque([seed])
            while dq:
                r, c = dq.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in cells:
                            cells.discard(n)
                            group.append(n)
                            dq.append(n)
            if len(group) >= self.args.min_gap:
                arr = np.array(group)
                r, c = arr[:, 0].mean(), arr[:, 1].mean()
                out.append((*self.world(r, c), len(group)))
        return out

    def viewpoint(self, gx, gy, rx, ry):
        """구멍 (gx,gy) 를 볼 수 있는 지점을 고른다.
        가깝고, 시야가 트여 있고, 로봇이 실제로 갈 수 있는 자리."""
        best = None
        # 통로 폭이 30cm 다. 벽에서 0.25m 떨어진 점은 반대쪽 벽에서 5cm 라
        # 코스트맵상 통행 불가(>=99)로 걸러진다. 그래서 후보가 하나도 안
        # 남아 3단계가 매번 '볼 수 있는 자리를 못 찾아 건너뜁니다' 로
        # 끝났다(324회차: 후보 2곳, 성공 0회, +6칸).
        # 통로 한가운데(0.15m)부터 찾는다. 라이다는 그 거리에서 벽을 잘 본다.
        for radius in (0.15, 0.20, 0.25, 0.35, 0.45, 0.60):
            for deg in range(0, 360, 15):
                a = math.radians(deg)
                px, py = gx + radius * math.cos(a), gy + radius * math.sin(a)
                r, c = self.cell(px, py)
                h, w = self.grid.shape
                if not (0 <= r < h and 0 <= c < w) or self.grid[r, c] != 0:
                    continue
                if not self.navigable(px, py) or not self.clear_line(px, py, gx, gy):
                    continue
                d = math.hypot(px - rx, py - ry)
                if best is None or d < best[0]:
                    best = (d, px, py)
            if best:
                break
        if best:
            return best[1:]

        # 고리 탐색은 반경 6개 x 각도 24개만 찍어본다. 좁은 통로에서는 그
        # 24개가 전부 벽이나 인플레이션에 걸려 '볼 수 있는 자리 없음' 이
        # 되기 쉽다(342회차: 후보 6곳 중 성공 0회, 재현율 92.9%).
        # 그래서 주변 격자를 훑어 조건을 만족하는 칸을 직접 찾는다.
        h, w = self.grid.shape
        step = max(1, int(round(0.025 / self.res)))
        span = int(round(0.60 / self.res))
        gr, gc = self.cell(gx, gy)
        for dr in range(-span, span + 1, step):
            for dc in range(-span, span + 1, step):
                r, c = gr + dr, gc + dc
                if not (0 <= r < h and 0 <= c < w) or self.grid[r, c] != 0:
                    continue
                px, py = self.world(r, c)
                d_gap = math.hypot(px - gx, py - gy)
                # 너무 붙으면 차체가 못 들어가고, 너무 멀면 안 보인다
                if not (0.12 <= d_gap <= 0.60):
                    continue
                if not self.navigable(px, py) or \
                        not self.clear_line(px, py, gx, gy):
                    continue
                d = math.hypot(px - rx, py - ry)
                if best is None or d < best[0]:
                    best = (d, px, py)
        return best[1:] if best else None

    def already_tried(self, x, y):
        return any(math.hypot(x - tx, y - ty) < self.args.skip_radius
                   for tx, ty in self.tried)

    # ------------------------------------------------------------------
    def go(self, px, py, gx, gy):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = float(px)
        goal.pose.position.y = float(py)
        qz, qw = yaw_to_quat(math.atan2(gy - py, gx - px))   # 구멍을 바라보게
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        t0 = time.time()
        self.nav.goToPose(goal)
        self._near.clear()
        while not self.nav.isTaskComplete():
            gap, why = self.too_close()
            if why:
                self.nav.cancelTask()
                print(f'      차체 여유 {gap*100:+.1f}cm ({why}) — '
                      f'목표를 취소하고 물러납니다')
                self.back_off()
                return False, time.time() - t0
            if time.time() - t0 > self.args.goal_timeout:
                self.nav.cancelTask()
                break
        ok = self.nav.getResult() == TaskResult.SUCCEEDED
        return ok, time.time() - t0

    def run(self):
        started = time.time()
        filled = 0
        for round_no in range(1, self.args.max_rounds + 1):
            if time.time() - started > self.args.total_timeout:
                print('\n제한 시간에 도달했습니다.')
                break
            self.spin(2.0)
            gaps = self.find_gaps()
            live = [g for g in gaps if not self.already_tried(g[0], g[1])]
            print(f'\n[{round_no}회차] 끊어진 벽 후보 {len(gaps)}곳 '
                  f'(가볼 수 있는 곳 {len(live)}곳)')
            if not live:
                print('더 메울 벽이 없습니다.')
                break

            pose = self.robot_xy()
            if pose is None:
                self.spin(1.0)
                continue
            rx, ry = pose
            live.sort(key=lambda g: math.hypot(g[0] - rx, g[1] - ry))
            gx, gy, size = live[0]
            vp = self.viewpoint(gx, gy, rx, ry)
            self.tried.append((gx, gy))
            if vp is None:
                print(f'  ({gx:.2f}, {gy:.2f}) {size}칸 — 볼 수 있는 자리를 '
                      f'못 찾아 건너뜁니다')
                continue
            px, py = vp
            print(f'  ({gx:.2f}, {gy:.2f}) {size}칸 -> 관측 지점 '
                  f'({px:.2f}, {py:.2f}) 으로 이동')
            ok, sec = self.go(px, py, gx, gy)
            print(f'    {"도착" if ok else "실패"} ({sec:.1f}초)')
            if ok:
                filled += 1
                self.spin(2.0)          # 새 스캔이 맵에 반영될 시간
        return filled


def main():
    p = argparse.ArgumentParser(description='끊어진 벽 메우기')
    p.add_argument('--sim', action='store_true')
    p.add_argument('--gap-span', type=int, default=4,
                   help='이 칸수 안에 양쪽 벽이 있으면 구멍으로 본다, 기본 4')
    p.add_argument('--min-gap', type=int, default=2,
                   help='이 칸수 이상인 구멍만 대상으로 삼는다, 기본 2')
    p.add_argument('--skip-radius', type=float, default=0.12,
                   help='이미 시도한 구멍 재시도 금지 반경(m), 기본 0.12')
    p.add_argument('--goal-timeout', type=float, default=45.0)
    p.add_argument('--total-timeout', type=float, default=240.0)
    p.add_argument('--max-rounds', type=int, default=20)
    args = p.parse_args()

    if args.sim:
        rclpy.init(args=['pinky_wallfill', '--ros-args', '-p',
                         'use_sim_time:=true'])
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

    filler = WallFiller(navigator, args)
    code = 0
    try:
        print('Nav2 가 준비될 때까지 기다립니다...')
        # 끝없이 기다리지 않는다. 320회차에서 4단계가 Nav2 준비 대기 안에서
        # 9분 넘게 멈춰 한 회차를 통째로 날렸다(pinky_explore.wait_nav2 참고).
        from pinky_explore import wait_nav2
        if not wait_nav2(navigator):
            print('Nav2 액션 서버가 응답하지 않습니다. 벽 메우기를 건너뜁니다.',
                  file=sys.stderr)
            raise RuntimeError('Nav2 준비 실패')
        print('Nav2 준비 완료')
        filler.spin(3.0)
        n = filler.run()
        print(f'\n메우기 시도 성공 {n}회')
    except KeyboardInterrupt:
        print('\n사용자 중단')
        code = 130
    except Exception as e:
        print(f'오류: {e}', file=sys.stderr)
        code = 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
