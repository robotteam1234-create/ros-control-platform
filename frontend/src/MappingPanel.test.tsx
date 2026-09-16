import { render, within } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { MappingPanel } from './MappingPanel'
import { MapHealth } from './MapHealth'
import { STAGE_INFO } from './mappingStages'

describe('MappingPanel', () => {
  it('shows mapping stages', () => {
    const view = render(<MappingPanel lease={null} mapId="map_260905" />)
    expect(within(view.container).getByText(/wall_follow/i)).toBeTruthy()
  })

  it('shows lap585 stage requirements and safe speed caps', () => {
    const view = render(<MappingPanel lease="lease-1" mapId="map_260905" />)
    expect(within(view.container).getByText(/라이다만/i)).toBeTruthy()
    expect(within(view.container).getByText(/0\.15 m\/s/i)).toBeTruthy()
    expect(within(view.container).getAllByText(/\(SLAM 필요\)/)).toHaveLength(3)
  })
})

describe('mappingStages', () => {
  it('matches lap585 hybrid pipeline order and ids', () => {
    expect(STAGE_INFO.map(stage => stage.id)).toEqual([
      'wall_follow',
      'frontier_explore',
      'wall_fill',
      'return_home',
    ])
  })
})

describe('MapHealth', () => {
  const sample = {
    size_m: [2.7, 1.8] as [number, number],
    known: 1200, free: 900, occupied: 300,
    gap_clusters: 0, gap_cells: 0,
    wall_thickness_cm: 7.5,
    diverged: false, reasons: [] as string[],
    complete: true,
  }

  it('renders health metrics when provided', () => {
    const view = render(<MapHealth health={sample} />)
    expect(within(view.container).getByText(/2\.7.*1\.8/i)).toBeTruthy()
    expect(within(view.container).getByText(/완료/i)).toBeTruthy()
  })

  it('renders empty state when no health data', () => {
    const view = render(<MapHealth health={null} />)
    expect(within(view.container).getByText(/맵 상태 없음/i)).toBeTruthy()
  })
})
