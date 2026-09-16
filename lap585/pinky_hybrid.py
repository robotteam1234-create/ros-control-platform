#!/usr/bin/env python3
"""
Pinky Pro 하이브리드 탐사 (벽 따라가기 + 프론티어 + 벽 메우기 + 복귀)
==========================================================
네 단계로 나눠, 각 방식의 장점만 쓴다.

  1단계  벽 따라가기      : Nav2 없이 라이다만으로 통로를 체계적으로 훑는다.
                          플래너/코스트맵 문제에서 자유롭고 빠지는 통로가 적다.
                          여기서 선 자리가 '출발 지점' 이 되어 4단계로 넘어간다.
  2단계  프론티어 탐사    : 만들어진 맵에서 미탐색 경계(프론티어)를 직접 계산해
                          "아직 볼 게 남았는가" 를 스스로 결정한다.
                          남아 있으면 Nav2 로 그 구석까지 가서 메운다.
                          (진척이 없으면 우수법 '보강' 을 한 번 더 돌린다)
  3단계  끊어진 벽 메우기 : 프론티어는 '미탐색 공간' 만 찾는다. 주변이 이미
                          관측된 벽선 중간의 구멍은 안 잡히므로, 그 구멍을
                          따로 찾아 옆에서 다시 관측한다.
  4단계  복귀            : 1단계에서 출발한 지점으로 돌아온다.

각 단계 사이에 맵을 채점해 진척이 있었는지 확인하고, 진척이 없으면 멈춘다.

  python3 pinky_hybrid.py --sim --save ~/pinky/map_hybrid
"""

import argparse
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

import pinky_mapcheck as mapcheck

MAP_QOS = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class MapWatcher(Node):
    """맵을 받아 '얼마나 알아냈는지' 와 '남은 경계가 있는지' 를 계산한다."""

    def __init__(self):
        super().__init__('pinky_hybrid_watch')
        self.grid = None
        self.info = None
        self.create_subscription(OccupancyGrid, '/map', self._cb, MAP_QOS)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def robot_pose(self):
        """map 기준 (x, y, yaw). TF 가 아직이면 None.

        주의: TransformListener 는 노드를 spin 해야 버퍼가 찬다.
        예외가 났을 때만 spin 하면 첫 조회가 늘 실패한 채 끝난다."""
        for _ in range(60):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.buffer.lookup_transform('map', 'base_link',
                                                  rclpy.time.Time())
            except Exception:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw
        return None

    def _cb(self, msg):
        self.info = msg.info
        self.grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def wait(self, timeout=20.0):
        t0 = time.time()
        while rclpy.ok() and self.grid is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.grid is not None

    def refresh(self, seconds=3.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def health(self):
        """맵이 물리적으로 말이 되는지 판정 (좌표계 손상 / 벽 구멍)"""
        if self.grid is None or self.info is None:
            return None
        return mapcheck.health(self.grid, self.info.resolution)

    def stats(self):
        """(관측 칸 수, 프론티어 칸 수, 가장 큰 프론티어 덩어리 크기)"""
        if self.grid is None:
            return 0, 0, 0
        g = self.grid
        known = int(np.count_nonzero(g != -1))
        free = (g == 0)
        unknown = (g == -1)
        touch = np.zeros_like(unknown)
        touch[1:, :] |= unknown[:-1, :]
        touch[:-1, :] |= unknown[1:, :]
        touch[:, 1:] |= unknown[:, :-1]
        touch[:, :-1] |= unknown[:, 1:]
        frontier = free & touch

        # 가장 큰 덩어리 크기 (8이웃)
        seen = np.zeros_like(frontier)
        best = 0
        rows, cols = np.nonzero(frontier)
        cells = set(zip(rows.tolist(), cols.tolist()))
        while cells:
            seed = cells.pop()
            size = 1
            stack = [seed]
            while stack:
                r, c = stack.pop()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in cells:
                            cells.discard(n)
                            size += 1
                            stack.append(n)
            best = max(best, size)
        return known, int(frontier.sum()), best


def run(cmd, tag, capture_home=False):
    """하위 스크립트를 돌리며 출력을 그대로 흘려보낸다.
    capture_home 이면 'HOME x y yaw' 줄을 가로채 돌려준다."""
    print(f'\n{"="*60}\n[{tag}]\n{"="*60}', flush=True)
    home = None
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1)
    for line in proc.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        if capture_home and line.startswith('HOME '):
            try:
                _, x, y, deg = line.split()
                home = (float(x), float(y), math.radians(float(deg)))
            except ValueError:
                pass
    proc.wait()
    return home if capture_home else proc.returncode


def broken(watch, args, where):
    """맵 좌표계가 무너졌으면 저장하고 True.

    예전에는 1단계 뒤에만 검사해서, 2·3단계 도중에 맵이 무너지면
    아무도 모른 채 끝까지 돌며 맵을 더 망가뜨렸다(316회차)."""
    watch.refresh(3.0)
    h = watch.health()
    if not h['diverged']:
        return False
    print(f'\n{where}에서 맵 좌표계가 손상되었습니다. '
          f'이후 단계는 의미가 없으므로 중단합니다.')
    mapcheck.report(h)
    save_map(args.save)
    return True


def save_map(path):
    import os
    path = os.path.expanduser(path)
    r = subprocess.run(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path,
         '--ros-args', '-p', 'save_map_timeout:=20.0'],
        capture_output=True, text=True, timeout=90)
    print('맵 저장 완료' if r.returncode == 0 else f'맵 저장 실패\n{r.stderr[-400:]}')


def main():
    p = argparse.ArgumentParser(description='Pinky Pro 하이브리드 탐사')
    p.add_argument('--sim', action='store_true')
    p.add_argument('--save', default='~/pinky/map_hybrid')
    p.add_argument('--wall-time', type=float, default=300.0,
                   help='1단계 벽 따라가기 제한 시간(초), 기본 300')
    p.add_argument('--frontier-rounds', type=int, default=2,
                   help='2단계를 최대 몇 번까지 반복할지, 기본 2')
    p.add_argument('--min-frontier', type=int, default=3,
                   help='이 칸수 이상 남아야 2단계를 돌린다, 기본 3')
    p.add_argument('--no-return', action='store_true')
    p.add_argument('--no-wallfill', action='store_true',
                   help='끊어진 벽 메우기 단계를 건너뛴다')
    p.add_argument('--wallfill-time', type=float, default=200.0,
                   help='벽 메우기 제한 시간(초), 기본 200')
    args = p.parse_args()

    rclpy.init(args=['pinky_hybrid', '--ros-args', '-p', 'use_sim_time:=true']
               if args.sim else None)
    watch = MapWatcher()
    sim = ['--sim'] if args.sim else []

    try:
        if not watch.wait():
            print('/map 을 못 받았습니다. SLAM 이 켜져 있는지 확인하세요.',
                  file=sys.stderr)
            sys.exit(1)
        known0, fr0, big0 = watch.stats()
        print(f'시작 시점: 관측 {known0}칸, 프론티어 {fr0}칸(최대덩어리 {big0})')

        # ---------- 1단계 : 벽 따라가기 ----------
        # 진짜 출발 지점은 1단계가 시작하는 자리다. 단계마다 프로세스를 새로
        # 띄우므로 3단계에 명시적으로 넘겨야 한다(안 넘기면 3단계가 자기가
        # 시작한 자리를 출발점으로 착각해 '이미 도착' 처리해버린다).
        home = run(['python3', '-u', 'pinky_wallfollow.py', *sim,
                    '--max-time', str(args.wall_time), '--no-save',
                    # 1단계는 Nav2 를 안 쓰는 순수 지역 제어라 맵을 망치지
                    # 않는다. 반면 2단계는 회차마다 결과가 갈린다(11회차 성공,
                    # 12회차 경로 실패, 13회차 맵 손상). 믿을 수 있는 쪽이
                    # 더 많이 훑도록 기다리는 시간을 넉넉히 준다.
                    '--idle-time', '30', '--idle-after-loop', '20'],
                   '1단계  벽 따라가기 — 통로를 체계적으로 훑는다',
                   capture_home=True)
        if home:
            print(f'\n출발 지점 확보: ({home[0]:.2f}, {home[1]:.2f}, '
                  f'{math.degrees(home[2]):.0f}도)')
        else:
            print('\n출발 지점을 못 받았습니다. 복귀를 건너뜁니다.',
                  file=sys.stderr)
        watch.refresh(4.0)
        retried = False
        known1, fr1, big1 = watch.stats()
        print(f'\n[1단계 결과] 관측 {known0} -> {known1}칸 '
              f'(+{known1-known0}),  남은 프론티어 {fr1}칸(최대 {big1})')
        h = watch.health()
        if h:
            mapcheck.report(h)
            if h['diverged']:
                print('\n맵 좌표계가 손상되어 이후 단계는 의미가 없습니다. 중단합니다.')
                save_map(args.save)
                return

        # ---------- 2단계 : 스스로 판단해 구석 메우기 ----------
        prev_known = known1
        for i in range(1, args.frontier_rounds + 1):
            watch.refresh(2.0)
            known, fr, big = watch.stats()
            if big < args.min_frontier:
                print(f'\n[판단] 남은 최대 프론티어가 {big}칸으로 '
                      f'기준({args.min_frontier})보다 작다 -> 2단계 생략')
                break
            print(f'\n[판단] 최대 프론티어 {big}칸이 남았다 -> '
                  f'프론티어 탐사 {i}회차를 돌린다')
            rc = run(['python3', '-u', 'pinky_explore.py', *sim,
                      # 2~3칸짜리 프론티어까지 쫓으면 얻는 것(맵의 0.05%)
                      # 보다 잃는 것이 크다. 18번 구역처럼 좁은 모서리에서
                      # 그런 목표를 쫓다 끼여 벽에 닿고, 그 충돌이 맵을
                      # 통째로 틀어놓는다(381회차: 3칸 목표, 거리 0.14m).
                      # 남는 벽 구멍은 3단계가 따로 메운다.
                      '--min-frontier', '4', '--min-distance', '0.05',
                      '--blacklist-radius', '0.08', '--goal-timeout', '45',
                      '--goal-retract', '0.10', '--no-return', '--no-save',
                      '--total-timeout', '240'],
                     f'2단계  프론티어 탐사 {i}회차 — 남은 구석을 메운다')
            if rc == 4:
                print('\nNav2 가 목표를 받지 못하는 상태입니다. '
                      '이후 단계도 같은 이유로 실패하므로 중단합니다.\n'
                      '  PC 에서:  pkill -f nav2_ ; ./pc_stack_start.sh')
                save_map(args.save)
                return
            if rc == 3 or broken(watch, args, f'2단계 {i}회차'):
                save_map(args.save)
                return
            watch.refresh(4.0)
            known, fr, big = watch.stats()
            gain = known - prev_known
            print(f'\n[2단계 {i}회차 결과] 관측 {prev_known} -> {known}칸 '
                  f'(+{gain}),  남은 프론티어 {fr}칸(최대 {big})')
            if gain < 20:
                if big >= args.min_frontier and not retried:
                    # Nav2 가 좁은 통로 끝의 프론티어로 가는 경로를 아예
                    # 못 짜는 경우가 있다('남은 거리 0.00 m' 만 반복).
                    # 그때는 Nav2 를 안 쓰는 벽 따라가기로 다시 훑는다.
                    # 12회차에서 프론티어 67칸을 남긴 채 2단계가 +8칸으로
                    # 끝나 트랙 오른쪽 끝을 통째로 놓쳤다.
                    retried = True
                    print(f'[판단] Nav2 가 프론티어 {big}칸에 못 간다 -> '
                          f'Nav2 를 쓰지 않는 벽 따라가기로 다시 훑는다')
                    run(['python3', '-u', 'pinky_wallfollow.py', *sim,
                         '--right',          # 반대 손으로 벽의 반대면을 훑는다
                         '--max-time', str(args.wall_time), '--no-save',
                         '--idle-time', '30'],
                        '2단계 보강  우수법으로 벽의 반대면 훑기')
                    watch.refresh(4.0)
                    known, fr, big = watch.stats()
                    print(f'\n[2단계 보강 결과] 관측 {prev_known} -> {known}칸 '
                          f'(+{known - prev_known}),  남은 프론티어 {fr}칸')
                    prev_known = known
                    continue
                print('[판단] 늘어난 관측이 거의 없다 -> 더 돌려도 소용없다고 보고 중단')
                break
            prev_known = known

        # ---------- 3단계 : 끊어진 벽 메우기 ----------
        # 프론티어는 '미탐색 공간'을 찾을 뿐, 벽선 중간이 몇 칸 빈 곳은
        # 주변이 이미 관측돼 있어 잡히지 않는다. 복귀하기 전에 그 구멍들을
        # 따로 찾아 메운다. (복귀 후 다시 나가지 않도록 순서를 여기에 둔다)
        if not args.no_wallfill:
            run(['python3', '-u', 'pinky_wallfill.py', *sim,
                 '--total-timeout', str(args.wallfill_time)],
                '3단계  끊어진 벽 메우기 — 벽선의 구멍을 찾아 관측한다')
            if broken(watch, args, '3단계'):
                return
            watch.refresh(4.0)
            known, fr, big = watch.stats()
            print(f'\n[3단계 결과] 관측 {prev_known} -> {known}칸 '
                  f'(+{known-prev_known})')

        # ---------- 4단계 : 복귀 ----------
        if not args.no_return and home:
            run(['python3', '-u', 'pinky_explore.py', *sim,
                 '--min-frontier', '9999',        # 탐사는 건너뛰고 복귀만
                 # 값이 '-' 로 시작하면 argparse 가 옵션으로 오해하므로
                 # 반드시 '--home=값' 형태로 붙여서 넘긴다.
                 f'--home={home[0]:.4f},{home[1]:.4f},'
                 f'{math.degrees(home[2]):.2f}',
                 '--home-timeout', '150', '--home-tolerance', '0.08',
                 # 시도마다 back_off/free_self 로 자세가 조금씩 바뀌므로,
                 # 몇 번 더 해보는 편이 성공률이 높다(360회차에서 3회로는
                 # 부족했다). 실패해도 각 시도가 몇 초 만에 끝난다.
                 '--max-retries', '5',
                 '--no-save'],
                '4단계  출발 지점으로 복귀')

        watch.refresh(3.0)
        known, fr, big = watch.stats()
        print(f'\n{"="*60}\n최종: 관측 {known}칸 (시작 대비 +{known-known0}), '
              f'남은 프론티어 {fr}칸\n{"="*60}')
        save_map(args.save)
    finally:
        watch.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
