# labctl CLI

`labctl` is the operational entrypoint for SoftBot Lab Platform.

Wrapper:
- `scripts/labctl`

Implementation:
- `software/cli/labctl.py`

Core commands:
- `labctl doctor`
- `labctl setup --online|--offline <bundle>`
- `labctl firmware build|flash`
- `labctl agent start`
- `labctl gui start`
- `labctl gui pump-eval [--foreground]`
- `labctl example run <name>`
- `labctl smoke`
- `labctl benchmark pumps --pump-label <label>`
- `labctl hardware test --component <name>`
- `labctl hardware panel`
- `labctl hardware gui [--foreground]`
- `labctl hardware verify [--timeout-s 8 --sample-timeout-s 3]`
- `labctl hardware off`
- `labctl stop`

Notes:
- Camera selection uses bitmask values `1..7` (`A=1`, `B=2`, `C=4`).
- Hardware component `valve_chamber_c` maps to the legacy BOOST electrical pin.
