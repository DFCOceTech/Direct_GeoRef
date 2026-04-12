# Integrated BMAD + OpenSpec Workflow

> Version: 1.0 | Status: Living Document | Last updated: {{DATE}}

## Development Model: Spec-Anchored

This project uses **spec-anchored development** — the middle tier of Specification-Driven Development (SDD):

- **Specs are authoritative** for *what should be built* (requirements, behavior, acceptance criteria)
- **Code is authoritative** for *what is built* (implementation details, runtime behavior)
- **Reconciliation** bridges the two — specs and code are kept in agreement through explicit update rituals

This is NOT spec-as-source (humans edit both specs and code). This is NOT spec-first-then-forget (specs live beyond initial implementation).

## Lifecycle Phases

### Phase 1: Planning
- **Owner**: PM Agent
- **Outputs**: Project Brief, PRD (FRs, NFRs)
- **Documents**: `_bmad/project-brief.md`, `_bmad/prd.md`

### Phase 2: Architecture
- **Owner**: Architect Agent
- **Outputs**: Architecture doc with ADRs, deployment topology
- **Documents**: `_bmad/architecture.md`

### Phase 3: Spec Authoring
- **Owner**: Architect + Dev Agents
- **Outputs**: OpenSpec capability specs and designs
- **Documents**: `openspec/capabilities/*/spec.md`, `design.md`

### Phase 4: Epic/Story Decomposition
- **Owner**: Scrum Master Agent
- **Outputs**: Epics with story tables, individual story files
- **Documents**: `epics/epic-*.md`, `epics/stories/s*-*.md`

### Phase 5: Implementation
- **Owner**: Dev Agent
- **Inputs**: Story file + referenced spec.md + design.md
- **Process**: Change proposal (if significant) → Tests → Code → Verify
- **Documents**: `openspec/change-proposals/CP-*.md` (if needed)

### Phase 6: Reconciliation (MANDATORY after each story)
- **Owner**: Dev Agent
- **Updates**:
  1. `openspec/capabilities/*/spec.md` — Implementation Status section
  2. `epics/stories/s*-*.md` — Status field, acceptance criteria checkboxes
  3. `epics/epic-*.md` — Story table status
  4. `_bmad/traceability.md` — Impl Status column
  5. `ops/status.md` — What's working, what's next
  6. `ops/changelog.md` — What was done

### Course Correction
- If implementation diverges from spec: **update the spec** with rationale, don't silently disagree
- If architecture changes: update `_bmad/architecture.md` and reset "Last Reconciled" date
- If requirements change: update PRD, cascade to affected specs and stories

## Document Ownership

| Document | Owner | Update Trigger |
|----------|-------|----------------|
| Project Brief | PM | Scope change |
| PRD | PM | Requirements change |
| Architecture | Architect | New component, ADR, or quarterly review |
| Capability Specs | Architect + Dev | New feature, bug fix, or reconciliation |
| Epics/Stories | Scrum Master | Sprint planning, story completion |
| Traceability | Scrum Master | Story completion |
| Ops status | Dev | Session end, deploy, or milestone |

## Story Status Lifecycle

```
Backlog → In Progress → Review → Done
                ↓
            Blocked (with blocker note)
```
