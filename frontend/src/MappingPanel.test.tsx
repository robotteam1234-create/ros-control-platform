import { render, within } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { MappingPanel } from './MappingPanel'
import { MapHealth } from './MapHealth'
import { STAGE_INFO } from './mappingStages'

describe('MappingPanel', () => {
  it('shows mapping stages', () => {
    const view = render(<MappingPanel lease={null} mapId="map_260905" />)
    expect(within(view.container).getByText(/벽 따라가기/)).toBeTruthy()
    expect(within(view.container).getByText(/프론티어 탐사/)).toBeTruthy()
  })

  it('shows status chip and idles before start', () => {
    const view = render(<MappingPanel lease="lease-1" mapId="map_260905" />)
    expect(within(view.container).getByTestId('mapping-state-chip').textContent).toBe('IDLE')
    expect(within(view.container).getByRole('button', { name: '자동 매핑 시작' })).toBeTruthy()
  })

  it('shows lap585 stage requirements and safe speed caps', () => {
    const view = render(<MappingPanel lease="lease-1" mapId="map_260905" />)
    expect(within(view.container).getByText(/라이다만/i)).toBeTruthy()
    expect(within(view.container).getByText(/0\.15 m\/s/i)).toBeTruthy()
    expect(within(view.container).getAllByText(/SLAM 필요/)).toHaveLength(3)
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

import { act, cleanup, fireEvent, waitFor } from '@testing-library/react'
import { afterEach, vi } from 'vitest'
import { decodeMappingFrame, openMappingStream } from './mappingStream'

function framedMapFrame(meta: object): ArrayBuffer {
  const metaBytes = new TextEncoder().encode(JSON.stringify(meta))
  const bytes = new Uint8Array(4 + metaBytes.length + 4)
  new DataView(bytes.buffer).setUint32(0, metaBytes.length)
  bytes.set(metaBytes, 4)
  bytes.set([0x89, 0x50, 0x4e, 0x47], 4 + metaBytes.length)
  return bytes.buffer
}

class FakeSocket {
  static all: FakeSocket[] = []
  url: string
  onmessage: ((event: MessageEvent) => void) | null = null
  onclose: (() => void) | null = null
  constructor(url: string) { this.url = url; FakeSocket.all.push(this) }
  close() { this.onclose?.() }
  send() {}
}

afterEach(() => { cleanup(); vi.unstubAllGlobals() })

describe('mappingStream', () => {
  it('decodes the 4-byte framed PNG payload', () => {
    const buffer = framedMapFrame({ seq: 3, width: 8, height: 6, resolution: 0.05, origin: { x: -1, y: -2 } })
    const { meta } = decodeMappingFrame(buffer)
    expect(meta.seq).toBe(3)
    expect(meta.width).toBe(8)
    expect(meta.origin.x).toBe(-1)
  })

  it('rejects non-PNG payloads', () => {
    const bytes = new Uint8Array(10)
    new DataView(bytes.buffer).setUint32(0, 2)
    bytes.set([1, 2, 3, 4, 5, 6], 4)
    expect(() => decodeMappingFrame(bytes.buffer)).toThrow()
  })

  it('opens a robot stream and delivers blob urls', () => {
    vi.stubGlobal('WebSocket', FakeSocket)
    vi.stubGlobal('URL', { createObjectURL: vi.fn(() => 'blob:map'), revokeObjectURL: vi.fn() })
    vi.stubGlobal('Blob', class { parts: unknown[]; constructor(parts: unknown[]) { this.parts = parts } })
    FakeSocket.all = []
    const seen: string[] = []
    const close = openMappingStream('robot_1', (_meta, url) => seen.push(url))
    const socket = FakeSocket.all[0]
    expect(socket.url).toContain('/ws/mapping/robot_1')
    act(() => socket.onmessage?.(new MessageEvent('message', { data: framedMapFrame({ seq: 1, width: 1, height: 1, resolution: 0.05, origin: { x: 0, y: 0 } }) })))
    expect(seen).toEqual(['blob:map'])
    close()
  })
})

describe('MappingPanel live run', () => {
  it('start reaches COMPLETED and import registers the map', async () => {
    let mappingState = 'IDLE'
    const json = (body: unknown, status = 200) => ({ ok: status < 400, status, json: async () => body })
    vi.stubGlobal('fetch', vi.fn(async (input: RequestInfo | URL, init?: RequestInit) => {
      const url = String(input)
      const method = init?.method ?? 'GET'
      const body = typeof init?.body === 'string' ? JSON.parse(init.body) : {}
      if (url === '/api/v1/mappings' && method === 'POST') return json({ mapping_id: 'mapping-9', state: 'DRAFT' }, 201)
      if (url.endsWith('/actions') && method === 'POST') return json({ command_id: 'cmd-1' }, 202)
      if (url.startsWith('/api/v1/commands/')) return json({ state: 'SUCCEEDED' })
      if (url.endsWith('/import')) return json({ imported_map_id: 'map_auto_1' }, 201)
      if (url.endsWith('/mappings/mapping-9')) return json({ mapping_id: 'mapping-9', state: mappingState, stage_index: 3 })
      return json({}, 404)
    }))
    const view = render(<MappingPanel lease="lease-1" mapId="mock_lab" />)
    fireEvent.click(within(view.container).getByRole('button', { name: '자동 매핑 시작' }))
    mappingState = 'COMPLETED'
    await waitFor(() => expect(within(view.container).getByTestId('mapping-import')).toBeTruthy(), { timeout: 4000 })
    fireEvent.click(within(view.container).getByTestId('mapping-import'))
    await waitFor(() => expect(within(view.container).getByTestId('mapping-imported').textContent).toContain('map_auto_1'), { timeout: 4000 })
  })
})
