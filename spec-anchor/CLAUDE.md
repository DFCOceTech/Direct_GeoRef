# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Workflow (MANDATORY)

This project uses **spec-anchored development** (BMAD + OpenSpec). Every code change follows:

1. **Spec First** — Update `openspec/capabilities/*/spec.md` with new REQ-* and SCENARIO-*. Create/update story in `epics/stories/`.
2. **Write Tests** — Tests reference REQ-* and SCENARIO-* in comments.
3. **Implement** — Code to satisfy spec requirements.
4. **Verify** — Run tests, type checks, builds per commands below.
5. **Reconcile Specs** — Update Implementation Status in spec.md. Update story status. Update `_bmad/traceability.md` impl status column. If implementation diverged from spec, update spec to match reality with rationale.
6. **Update Ops** — Update `ops/status.md` (what's working/next) and `ops/changelog.md` (what you did).

Never leave specs and code disagreeing silently.

### Architecture Freshness Check

If `_bmad/architecture.md` "Last Reconciled" date is >30 days old, flag to user before starting new capability work.

## Build / Test / Deploy

```bash
# {{FILL: build commands}}
# {{FILL: test commands}}
# {{FILL: lint/type-check commands}}
# {{FILL: deploy commands}}
```

## Build Environment

<!-- {{FILL: WSL quirks, nvm, toolchain notes}} -->

## Key Paths

| What | Where |
|------|-------|
| BMAD strategic docs | `_bmad/` |
| Capability specs | `openspec/capabilities/*/spec.md` |
| Capability designs | `openspec/capabilities/*/design.md` |
| Change proposals | `openspec/change-proposals/` |
| Epics & stories | `epics/` |
| Operational status | `ops/status.md` |
| Server & credentials | `ops/server.md` |
| Work log | `ops/changelog.md` |
| Known issues | `ops/known-issues.md` |

## When to Read Deeper

- **Before starting a new capability**: Read the relevant `openspec/capabilities/*/spec.md` and `design.md`
- **Before deploying or debugging server issues**: Read `ops/server.md` and `ops/known-issues.md`
- **Before architectural decisions or adding new components**: Read `_bmad/architecture.md`
- **To understand project scope or requirements**: Read `_bmad/prd.md`
- **To check what's built vs. spec'd**: Read `_bmad/traceability.md` (has implementation status per FR)
