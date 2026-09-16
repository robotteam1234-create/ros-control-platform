// Renders lap585 pinky_mapcheck.health() output shape.
// health=null until Plan 2 wires live map health from the backend.
export interface MapHealthData {
  size_m: [number, number]
  known: number
  free: number
  occupied: number
  gap_clusters: number
  gap_cells: number
  wall_thickness_cm: number
  diverged: boolean
  reasons: string[]
  complete: boolean
}

export function MapHealth({ health }: { health: MapHealthData | null }) {
  if (!health) return <p className="map-warning">맵 상태 없음 (실시간 맵 점수는 Plan 2에서 연결됩니다).</p>
  return (
    <dl className="map-health">
      <dt>맵 크기</dt>
      <dd>
        {health.size_m[0]} x {health.size_m[1]} m
      </dd>
      <dt>관측 칸</dt>
      <dd>
        {health.known} (빈칸 {health.free}, 벽 {health.occupied})
      </dd>
      <dt>끊어진 벽</dt>
      <dd>
        {health.gap_clusters}곳 / {health.gap_cells}칸
      </dd>
      <dt>판정</dt>
      <dd>{health.complete ? '완료' : health.diverged ? '좌표계 틀어짐' : '진행 중'}</dd>
      {health.reasons.map(reason => (
        <dd key={reason}>{reason}</dd>
      ))}
    </dl>
  )
}

export default MapHealth
