import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { MappingPanel } from './MappingPanel'

describe('MappingPanel', () => {
  it('shows mapping stages', () => {
    render(<MappingPanel lease={null} mapId="map_260905" />)
    expect(screen.getByText(/wall_follow/i)).toBeTruthy()
  })
})
