# Mantenimiento y handover - ES

## Flujo de ramas
- `main`: rama estable y desplegable.
- ramas de trabajo: `feature/*`, `fix/*`.
- integrar por PR con CI verde.

## Politica de versiones
- Tags semanticos: `vX.Y.Z`.
- `X`: cambios incompatibles.
- `Y`: nuevas funciones compatibles.
- `Z`: fixes.

## Responsabilidades de mantenimiento
1. Mantener perfiles `config/profiles/*.json` sincronizados con hardware real.
2. Ejecutar release checklist antes de tag.
3. Actualizar `CHANGELOG.md` en cada release.
4. Regenerar bundle offline al menos trimestralmente.

## Actualizacion trimestral sugerida
1. `git pull` en rama de mantenimiento.
2. `./scripts/labctl doctor --profile default`
3. `./scripts/create_offline_bundle.sh`
4. Guardar artefactos offline en almacenamiento institucional.

## Transferencia a nuevos ingenieros
1. Sesion de onboarding con `docs/installation_quickstart_es.md`.
2. Practica guiada del runbook operativo.
3. Ejecucion de un smoke test completo supervisado.
4. Firma de checklist de handover interno.
