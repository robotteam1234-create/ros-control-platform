// Stage metadata mirrors lap585/pinky_hybrid.py pipeline docstring.
// Order and ids must stay in sync with backend STAGES.
export type StageRequirement = 'lidar-only' | 'slam-nav2'

export interface StageInfo {
  id: string
  label: string
  description: string
  requirement: StageRequirement
}

export const STAGE_INFO: StageInfo[] = [
  {
    id: 'wall_follow',
    label: '벽 따라가기',
    description: 'Nav2 없이 라이다만으로 통로를 훑습니다. 플래너/코스트맵 문제에서 자유롭습니다.',
    requirement: 'lidar-only',
  },
  {
    id: 'frontier_explore',
    label: '프론티어 탐사',
    description: '미탐색 경계를 계산해 Nav2로 구석까지 이동합니다. SLAM 맵이 필요합니다.',
    requirement: 'slam-nav2',
  },
  {
    id: 'wall_fill',
    label: '끊어진 벽 메우기',
    description: '관측된 벽선 중간의 구멍을 옆에서 다시 관측합니다. SLAM 맵이 필요합니다.',
    requirement: 'slam-nav2',
  },
  {
    id: 'return_home',
    label: '복귀',
    description: '1단계 출발 지점으로 돌아옵니다.',
    requirement: 'slam-nav2',
  },
]

// Safe caps enforced by pinky_control_watchdog (manual_timeout 0.35s).
// Displayed so operators know the envelope; values are enforced robot-side.
export const SAFE_CAPS = { maxLinearMps: 0.15, maxAngularRps: 0.5 } as const
