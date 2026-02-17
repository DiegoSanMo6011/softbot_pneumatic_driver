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
- `labctl example run <name>`
- `labctl smoke`
- `labctl hardware test --component <name>`
- `labctl hardware panel`
- `labctl hardware gui [--foreground]`
- `labctl hardware verify [--timeout-s 8 --sample-timeout-s 3]`
- `labctl hardware off`
- `labctl stop`
