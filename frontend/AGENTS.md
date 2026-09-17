<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# frontend

## Purpose
React 18 + TypeScript SPA (`pinky-control-frontend`) — the operator dashboard. Single page, **no router**, no state library, no CSS framework: plain `fetch` + `WebSocket`, one global stylesheet, Korean-language UI. The browser never touches ROS; everything goes through the FastAPI backend at `127.0.0.1:8081`.

## Key Files
| File | Description |
|------|-------------|
| `index.html` | Minimal Vite entry: `<div id="root">` + `/src/main.tsx` |
| `vite.config.ts` | React plugin + **identical `/api`+`/ws` proxy under both `server` and `preview`** — the preview entry is load-bearing; without it `vite preview` serves a dead static page |
| `vitest.config.ts` | jsdom test environment, no setup file (each test imports jest-dom itself), globals off |
| `tsconfig.json` | Strict TS, bundler resolution, `noEmit` (typecheck via `npx tsc --noEmit`) |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `src/` | All source + colocated `*.test.ts(x)` files (see `src/AGENTS.md`) |

## For AI Agents

### Working In This Directory
- Commands: `npm install`; tests `npx vitest run [path]`; typecheck `npx tsc --noEmit`; build `npm run build` (`tsc -b && vite build`); dev `npm run dev` (port 5173, proxies to 8081).
- Backend must be running on `127.0.0.1:8081` for the UI to work (mock mode command in root AGENTS.md).
- WS endpoints used: `/ws/state`, `/ws/teleop`, `/ws/cameras/{robot_id}?quality=low|default|high`; REST base `/api/v1/...`.
- Keep `vite.config.ts` preview proxy intact when touching Vite config.

### Testing Requirements
- `npx vitest run` from this directory; ~55 tests across colocated test files.
- Prefer `within(view.container)` queries over `screen.getBy` (render binds to body).
- Stop-related copy in tests/components must say software-only ("물리 안전 장치가 아닙니다"), never "긴급 정지" — enforced by T05 tests.

### Common Patterns
- Default-export function components, dense JSX, `data-testid` for status readouts.
- Error bubbling: panels take `onError(message)` props; App renders one banner.

## Dependencies

### External
- Runtime: `react`/`react-dom` ^18.3.1 — nothing else
- Dev: `typescript` ^5.7.2, `vite` ^5.4.14, `@vitejs/plugin-react` ^4.3.4, `vitest` ^2.1.8, `@testing-library/react` ^16.1.0, `@testing-library/jest-dom` ^6.6.3, `jsdom` ^24.1.3

### Internal
- Consumes backend contracts from `../backend/pinky_control_center/api/` and `../docs/02-functional-spec.md`.

<!-- MANUAL: -->
