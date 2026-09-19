export type DashboardTab = 'control' | 'drive' | 'mission' | 'alerts' | 'settings'

export function parseHash(hash: string): DashboardTab {
  const clean = hash.replace('#', '')
  if (clean === 'camera') return 'control' // legacy camera tab merged into control
  return clean === 'control' || clean === 'drive' || clean === 'mission' || clean === 'alerts' || clean === 'settings' ? clean as DashboardTab : 'drive'
}

const TABS: { id: DashboardTab; label: string }[] = [
  { id: 'control', label: '제어 Control' },
  { id: 'drive', label: '관제 Drive' },
  { id: 'mission', label: '편대·임무' },
  { id: 'alerts', label: '알림·이력' },
  { id: 'settings', label: '설정' },
]

export default function DashboardTabs({ active, onChange }: { active: DashboardTab; onChange: (tab: DashboardTab) => void }) {
  return (
    <nav className="tabs" role="tablist" aria-label="관제 섹션">
      {TABS.map(tab => (
        <button key={tab.id} role="tab" aria-selected={active === tab.id} className={active === tab.id ? 'tab active' : 'tab'} onClick={() => onChange(tab.id)}>
          {tab.label}
        </button>
      ))}
    </nav>
  )
}
