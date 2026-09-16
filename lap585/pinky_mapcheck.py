#!/usr/bin/env python3
"""
맵 건강도 판정 — SLAM 이 틀어졌는지 스스로 알아내기 위한 공통 모듈.

로봇이 벽에 부딪치면 스캔 매칭이 어긋나 맵 좌표계가 늘어난다. 문제는
로봇이 그걸 모른 채 계속 주행해 맵을 더 망친다는 점이다. 여기서는
'만들어진 맵이 실제 트랙과 물리적으로 말이 되는가' 를 검사한다.

  - 관측된 칸들의 실제 크기가 트랙보다 크면 좌표계가 늘어난 것이다.
  - 벽 두께가 비정상적으로 두꺼우면 벽이 겹쳐 그려진 것이다.

  python3 pinky_mapcheck.py            # 지금 /map 을 검사
  python3 pinky_mapcheck.py map.pgm    # 저장된 맵을 검사
"""

import math
import sys
from collections import deque

import numpy as np

# 주행 공간의 실측 크기 (m). 환경변수로 바꿀 수 있다.
#
#   PINKY_TRACK_W / PINKY_TRACK_H  공간의 긴 변 / 짧은 변
#   둘 중 하나라도 0 이면 '크기를 모른다' 로 보고 크기 검사를 끈다.
#
# 왜 환경변수인가: 이 값이 하드코딩돼 있으면, 실습 트랙보다 넓은 곳에서
# 정상적으로 매핑하는 중에도 '맵이 트랙보다 크다 -> 좌표계 붕괴' 로 판정해
# 주행을 스스로 멈춘다. 2026-09-10 실물 388회차에서 실제로 그렇게 멈췄다
# (맵 5.06 x 1.73 m, 목표 4개 전부 성공하던 중이었다).
import os as _os
TRACK_W = float(_os.environ.get('PINKY_TRACK_W', 2.710))
TRACK_H = float(_os.environ.get('PINKY_TRACK_H', 1.260))
SIZE_CHECK = TRACK_W > 0 and TRACK_H > 0
BBOX_TOLERANCE = 1.35          # 실측 대비 이 배율을 넘으면 틀어진 것으로 본다


def bbox_of_known(grid, res):
    """관측된 칸이 차지하는 실제 크기(m) — 회전과 무관하게 잰다.

    주의: SLAM 맵 좌표계는 로봇이 시작할 때 향한 방향 기준이라 실제 트랙에
    대해 임의로 돌아가 있다. 축 정렬 사각형으로 재면 2.71 x 1.26 m 트랙이
    45도 돌았을 때 2.81 x 2.81 m 로 나와 멀쩡한 맵도 '손상' 으로 오판한다.
    그래서 각도를 훑어 가장 작은 사각형(min-area rect)을 찾는다."""
    # 벽(점유) 칸만 쓴다. 빈칸까지 포함하면, 벽 구멍으로 라이다가 관통해
    # 트랙 밖을 '빈 공간' 으로 칠한 누출분까지 크기에 잡혀 멀쩡한 맵이
    # 부풀어 보인다. 벽은 트랙 밖으로 새어나갈 수 없으므로 기준이 된다.
    known = grid > 50
    if not known.any():
        return 0.0, 0.0
    rows, cols = np.nonzero(known)
    pts = np.stack([cols * res, rows * res], axis=1)
    best = None
    for deg in range(0, 90, 2):
        a = math.radians(deg)
        ca, sa = math.cos(a), math.sin(a)
        u = pts[:, 0] * ca + pts[:, 1] * sa
        v = -pts[:, 0] * sa + pts[:, 1] * ca
        w = u.max() - u.min() + res
        h = v.max() - v.min() + res
        if best is None or w * h < best[0]:
            best = (w * h, max(w, h), min(w, h))
    return best[1], best[2]


def wall_gaps(grid, span=4, min_size=2):
    """벽이어야 하는데 아직 못 본 칸 덩어리 수와 총 칸수.

    어떤 미탐색 칸의 서로 마주보는 두 방향 모두 가까이에 벽이 있으면,
    그 자리는 벽인데 아직 관측 못 한 '구멍' 으로 본다."""
    occ = grid > 50
    unknown = grid == -1
    h, w = grid.shape
    pairs = [((0, 1), (0, -1)), ((1, 0), (-1, 0)),
             ((1, 1), (-1, -1)), ((1, -1), (-1, 1))]
    gap = np.zeros_like(unknown)
    for r, c in zip(*[a.tolist() for a in np.nonzero(unknown)]):
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

    cells = set(zip(*[a.tolist() for a in np.nonzero(gap)]))
    clusters = 0
    total = 0
    while cells:
        seed = cells.pop()
        size = 1
        dq = deque([seed])
        while dq:
            r, c = dq.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    n = (r + dr, c + dc)
                    if n in cells:
                        cells.discard(n)
                        size += 1
                        dq.append(n)
        if size >= min_size:
            clusters += 1
            total += size
    return clusters, total


def wall_thickness(grid, res):
    """벽 칸의 평균 두께(m) 추정. 실제 벽은 5mm 라 격자 한 칸이 정상이고,
    겹쳐 그려지면 두꺼워진다."""
    occ = grid > 50
    if not occ.any():
        return 0.0
    # 가로/세로로 연속된 벽 칸의 길이 분포에서 짧은 쪽(=두께) 을 본다
    runs = []
    for axis in (0, 1):
        arr = occ if axis == 0 else occ.T
        for line in arr:
            n = 0
            for v in line:
                if v:
                    n += 1
                elif n:
                    runs.append(n)
                    n = 0
            if n:
                runs.append(n)
    if not runs:
        return 0.0
    runs.sort()
    return runs[len(runs) // 2] * res      # 중앙값 = 대부분의 두께


def health(grid, res):
    """맵 상태를 한눈에 볼 수 있는 판정 결과"""
    bw, bh = bbox_of_known(grid, res)
    long_side, short_side = max(bw, bh), min(bw, bh)
    if SIZE_CHECK:
        limit_long = TRACK_W * BBOX_TOLERANCE
        limit_short = TRACK_H * BBOX_TOLERANCE
        diverged = long_side > limit_long or short_side > limit_short
    else:
        diverged = False        # 공간 크기를 모르면 이 검사는 못 한다

    clusters, gap_cells = wall_gaps(grid)
    thick = wall_thickness(grid, res)
    known = int((grid != -1).sum())
    free = int((grid == 0).sum())
    occ = int((grid > 50).sum())

    reasons = []
    if diverged:
        reasons.append(f'맵 크기 {long_side:.2f} x {short_side:.2f} m 가 '
                       f'트랙({TRACK_W} x {TRACK_H})보다 큼 — 좌표계 틀어짐')
    # 주의: 이 '두께' 는 행/열 방향 연속 길이의 중앙값이라 벽의 길이와
    # 두께가 섞인다. 실제로 정상 맵과 붕괴 맵이 똑같이 7.5cm 로 나와
    # 판별력이 없으므로 손상 판정에는 쓰지 않고 참고로만 표시한다.

    return {
        'size_m': (round(long_side, 2), round(short_side, 2)),
        'known': known, 'free': free, 'occupied': occ,
        'gap_clusters': clusters, 'gap_cells': gap_cells,
        'wall_thickness_cm': round(thick * 100, 1),
        'diverged': diverged,
        'reasons': reasons,
        # 목표: 벽 구멍이 없고 좌표계가 멀쩡할 것
        'complete': (not diverged and clusters == 0),
    }


def jumped(prev_long, long_side, seconds, min_size=2.0, ratio=1.6):
    """짧은 시간에 맵이 갑자기 커졌는가.

    공간의 실제 크기를 몰라도 쓸 수 있는 붕괴 신호다. 정상 탐사는 로봇이
    움직인 만큼만 맵이 자란다(0.07 m/s). 반면 스캔 매칭이 어긋나면 맵이
    한순간에 몇 배로 늘어난다. 아직 작은 맵은 정상적으로도 빨리 크므로
    min_size 를 넘은 뒤에만 본다."""
    if prev_long is None or prev_long < min_size or seconds <= 0:
        return False
    if seconds > 10.0:            # 너무 오래된 값과는 비교하지 않는다
        return False
    return long_side > prev_long * ratio


def report(h):
    scope = (f'(공간 {TRACK_W} x {TRACK_H})' if SIZE_CHECK
             else '(공간 크기 미지정 — 크기 검사 꺼짐)')
    print(f'  맵 크기      {h["size_m"][0]} x {h["size_m"][1]} m {scope}')
    print(f'  관측 {h["known"]}칸 (빈칸 {h["free"]}, 벽 {h["occupied"]})')
    print(f'  벽 두께      {h["wall_thickness_cm"]} cm')
    print(f'  끊어진 벽    {h["gap_clusters"]}곳 / {h["gap_cells"]}칸')
    if h['reasons']:
        for r in h['reasons']:
            print(f'  [경고] {r}')
    print(f'  판정: ' + ('완성' if h['complete']
                        else ('좌표계 손상' if h['diverged'] else '벽 미완성')))


def from_pgm(path):
    from PIL import Image
    a = np.array(Image.open(path))
    g = np.full(a.shape, -1, np.int8)
    g[a > 220] = 0
    g[a < 60] = 100
    return g


def main():
    if len(sys.argv) > 1:
        g = from_pgm(sys.argv[1])
        print(f'파일: {sys.argv[1]}')
        report(health(g, 0.025))
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSHistoryPolicy, QoSReliabilityPolicy)
    from nav_msgs.msg import OccupancyGrid
    import time
    q = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=QoSReliabilityPolicy.RELIABLE,
                   history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    rclpy.init(args=['mapcheck', '--ros-args', '-p', 'use_sim_time:=true'])
    n = Node('mapcheck')
    box = {}
    n.create_subscription(OccupancyGrid, '/map',
                          lambda m: box.setdefault('m', m), q)
    t0 = time.time()
    while 'm' not in box and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    if 'm' not in box:
        print('/map 을 못 받았습니다'); sys.exit(1)
    m = box['m']
    g = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    report(health(g, m.info.resolution))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
