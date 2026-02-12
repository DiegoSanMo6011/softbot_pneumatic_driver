# Maintenance and Handover - EN

## Branch flow
- `main`: stable, deployable branch.
- work branches: `feature/*`, `fix/*`.
- merge via PR with green CI.

## Version policy
- Semantic tags: `vX.Y.Z`.
- `X`: breaking changes.
- `Y`: backward-compatible features.
- `Z`: fixes.

## Maintenance responsibilities
1. Keep `config/profiles/*.json` aligned with real hardware.
2. Execute release checklist before tagging.
3. Update `CHANGELOG.md` for each release.
4. Regenerate offline bundle at least quarterly.

## Recommended quarterly update
1. `git pull` on maintenance branch.
2. `./scripts/labctl doctor --profile default`
3. `./scripts/create_offline_bundle.sh`
4. Archive offline artifacts in institutional storage.

## Onboarding transfer flow
1. Run onboarding using `docs/installation_quickstart_en.md`.
2. Practice daily runbook.
3. Execute one full supervised smoke test.
4. Sign internal handover checklist.
