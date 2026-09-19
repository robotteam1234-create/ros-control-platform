// frontend/src/mappingStream.ts
export type MappingFrameMeta = { seq: number; width: number; height: number; resolution: number; origin: { x: number; y: number } }

export function decodeMappingFrame(data: ArrayBuffer): { meta: MappingFrameMeta; png: Uint8Array } {
  if (data.byteLength < 5) throw new Error('frame too short')
  const view = new DataView(data)
  const length = view.getUint32(0)
  if (length > 65536 || length + 4 >= data.byteLength) throw new Error('invalid metadata length')
  const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(data, 4, length))) as MappingFrameMeta
  const png = new Uint8Array(data, 4 + length)
  if (!meta.width || !meta.height || png.length < 4 || png[0] !== 0x89 || png[1] !== 0x50 || png[2] !== 0x4e || png[3] !== 0x47) throw new Error('invalid map frame')
  return { meta, png }
}

/** Subscribe to the live SLAM map for one robot. Returns a close function. */
export function openMappingStream(robotId: string, onFrame: (meta: MappingFrameMeta, url: string) => void): () => void {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws'
  const socket = new WebSocket(`${proto}://${location.host}/ws/mapping/${robotId}`)
  socket.binaryType = 'arraybuffer'
  let lastUrl: string | null = null
  socket.onmessage = event => {
    try {
      const { meta, png } = decodeMappingFrame(event.data as ArrayBuffer)
      const blob = new Blob([png.slice().buffer as ArrayBuffer], { type: 'image/png' })
      if (lastUrl) URL.revokeObjectURL(lastUrl)
      lastUrl = URL.createObjectURL(blob)
      onFrame(meta, lastUrl)
    } catch {
      /* malformed frame: skip */
    }
  }
  return () => {
    socket.close()
    if (lastUrl) URL.revokeObjectURL(lastUrl)
  }
}
