# CLAUDE.md — Direct_GeoRef

This file provides guidance to Claude Code when working with this project.

## Development Workflow (MANDATORY)

This project uses **spec-anchored development** (BMAD + OpenSpec). Every code change follows:

1. **Spec First** — Update `spec-anchor/openspec/capabilities/*/spec.md` with new REQ-* and SCENARIO-*. Create/update story in `spec-anchor/epics/stories/`.
2. **Write Tests** — Tests reference REQ-* and SCENARIO-* in comments.
3. **Implement** — Code to satisfy spec requirements. Use Jupyter notebooks in `notebooks/`.
4. **Verify** — Run tests and notebook cells per commands below.
5. **Reconcile Specs** — Update Implementation Status in spec.md. Update story status. Update `spec-anchor/_bmad/traceability.md` impl status column. If implementation diverged from spec, update spec to match reality with rationale.
6. **Update Ops** — Update `spec-anchor/ops/status.md` (what's working/next) and `spec-anchor/ops/changelog.md` (what you did).

Never leave specs and code disagreeing silently.

## Build / Test / Deploy

```bash
# Activate environment
conda activate direct-georef

# Launch Jupyter
jupyter lab

# Run tests
pytest tests/

# Lint / type-check
ruff check src/
mypy src/
```

## Build Environment

- Python environment: conda env `direct-georef` (Python 3.11)
- Do NOT modify the base conda environment
- Environment spec: `environment.yml` at project root
- Platform: macOS (Apple Silicon)

## Key Paths

| What | Where |
|------|-------|
| BMAD strategic docs | `spec-anchor/_bmad/` |
| Capability specs | `spec-anchor/openspec/capabilities/*/spec.md` |
| Capability designs | `spec-anchor/openspec/capabilities/*/design.md` |
| Change proposals | `spec-anchor/openspec/change-proposals/` |
| Epics & stories | `spec-anchor/epics/` |
| Operational status | `spec-anchor/ops/status.md` |
| Work log | `spec-anchor/ops/changelog.md` |
| Known issues | `spec-anchor/ops/known-issues.md` |
| Jupyter notebooks | `notebooks/` |
| Source modules | `src/direct_georef/` |
| Tests | `tests/` |
| Input data | `data/` (not committed — large files) |
| Output products | `output/` |

## When to Read Deeper

- **Before starting a new capability**: Read the relevant `spec-anchor/openspec/capabilities/*/spec.md` and `design.md`
- **Before architectural decisions**: Read `spec-anchor/_bmad/architecture.md`
- **To understand project scope**: Read `spec-anchor/_bmad/prd.md`
- **To check what's built vs. spec'd**: Read `spec-anchor/_bmad/traceability.md`
