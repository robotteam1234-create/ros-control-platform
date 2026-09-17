<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# docs

## Purpose
Contract-grade project documentation, written in Korean: requirements, functional/interface spec (the contract source of truth), implementation plan, contract clarifications, integration report, user guide, plus per-task TDD evidence records and superpowers plan/design pairs.

## Key Files
| File | Description |
|------|-------------|
| `01-requirements.md` | Requirements: R01+ table with acceptance criteria; roles = operator/observer/admin |
| `02-functional-spec.md` | Functional & interface spec — **"names and numbers in this doc are the contract for the new implementation"** |
| `03-implementation-plan.md` | Implementation plan: file structure, task order T01–T10 + T12–T15 (T11 video recording excluded) |
| `04-contract-clarifications.md` | Supplements overriding the spec where ambiguous (follow-start semantics, robot registration deferral, sensor-layer payload, fixed domains 12/13) |
| `integration-report.md` | T12 rosbridge integration: verified real-robot contracts, IPs/MACs, Wi-Fi latency, explicitly listed unverified items |
| `user-guide.md` | End-user guide: run the platform, mock-first workflow, robot_2 watchdog install, Nav2 goals |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `tdd/` | Per-task TDD records: RED command + real failure reason → GREEN result tables (T04–T12) |
| `superpowers/` | Brainstorm/plan workflow artifacts — date-paired `plans/` + `specs/` files per feature |

## For AI Agents

### Working In This Directory
- Contract precedence: `04-contract-clarifications.md` overrides `02-functional-spec.md`; when code and docs disagree, record the verified actual interface and the reason for the change.
- `tdd/` records are evidence logs — write real output only; separate config/collection failures from actual assertion failures.
- `superpowers/plans/` and `superpowers/specs/` are paired by date-prefixed filename; new features following that workflow add both.
- Never mark unrun verification as PASS — that rule lives in `../acceptance-report.md`.

### Testing Requirements
- Documentation only — no tests. TDD evidence goes in `tdd/T<number>-<name>.md` after implementing the matching task.

<!-- MANUAL: -->
