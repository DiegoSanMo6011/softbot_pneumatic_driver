# Release Checklist (SoftBot Lab Platform)

## Pre-release
- [ ] `python3 -m py_compile $(find software -type f -name "*.py")`
- [ ] `.venv/bin/ruff check software`
- [ ] `.venv/bin/ruff format --check software`
- [ ] `./scripts/labctl doctor --profile default`
- [ ] `./scripts/labctl firmware build --profile default`
- [ ] `./scripts/labctl smoke --profile default` (hardware available)
- [ ] `./scripts/labctl hardware test --component valve_inflate --duration-s 0.8` (hardware available)
- [ ] `./scripts/tester_report.sh` (software-only baseline)

## Documentation
- [ ] README quickstart is current
- [ ] ES/EN installation docs are consistent with current commands
- [ ] Runbook reflects current lab procedure
- [ ] `CHANGELOG.md` updated

## Governance
- [ ] CI passing on target branch
- [ ] Version selected (`vX.Y.Z`)
- [ ] Tag created and pushed
- [ ] Release notes published
