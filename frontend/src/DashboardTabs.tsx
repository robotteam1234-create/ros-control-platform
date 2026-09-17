export type DashboardTab = 'drive' | 'mission' | 'camera' | 'alerts' | 'settings'

const TABS: { id: DashboardTab; label: string }[] = [
  { id: 'drive', label: '관제 Drive' },
  { id: 'mission', label: '편대·임무' },
  { id: 'camera', label: '카메라' },
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
