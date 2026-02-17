#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

log() {
  printf '[test_no_hw] %s\n' "$1"
}

RUFF_BIN=".venv/bin/ruff"
if [[ ! -x "$RUFF_BIN" ]]; then
  RUFF_BIN="ruff"
fi

log "Python syntax check"
python3 -m py_compile $(find software -type f -name '*.py')

log "Ruff lint"
"$RUFF_BIN" check software

log "Ruff format check"
"$RUFF_BIN" format --check software

log "Profile JSON validation"
python3 -m json.tool config/profiles/default_lab.json >/dev/null
python3 -m json.tool config/profiles/esp32_devkit_v1.json >/dev/null

log "CLI command surface"
./scripts/labctl --help >/dev/null
./scripts/labctl firmware --help >/dev/null
./scripts/labctl agent --help >/dev/null
./scripts/labctl gui --help >/dev/null
./scripts/labctl example run --help >/dev/null
./scripts/labctl smoke --help >/dev/null
./scripts/labctl hardware --help >/dev/null
./scripts/labctl hardware test --help >/dev/null
./scripts/labctl hardware gui --help >/dev/null

log "No-hardware checks completed"
