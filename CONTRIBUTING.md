# Contributing Guidelines

## Branching
- Stable branch: `main`
- Feature branches: `feature/<short-topic>`
- Fix branches: `fix/<short-topic>`

## Pull request requirements
1. CI must pass.
2. Changes must include documentation updates when behavior changes.
3. Platform profiles (`config/profiles/*.json`) must stay valid.
4. Avoid breaking public SDK API without explicit versioning decision.

## Quality checks
Run locally before opening PR:
```bash
python3 -m py_compile $(find software -type f -name "*.py")
.venv/bin/ruff check software
.venv/bin/ruff format --check software
```

## Commit style
Use concise imperative subject lines, for example:
- `Add labctl smoke command`
- `Fix firmware macro for PlatformIO build`

## Release flow
- Update `CHANGELOG.md`
- Run `docs/release_checklist.md`
- Tag on `main` with semantic version `vX.Y.Z`
