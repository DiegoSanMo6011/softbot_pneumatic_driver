# SoftBot Lab Platform v1.0

SoftBot Lab Platform is a reproducible pneumatic robotics stack for ESP32 + ROS 2,
prepared for daily laboratory use and generational handover.

## Scope
- Official runtime platform: **Ubuntu 22.04 + ROS 2 Humble**
- Deployment model: **Dual Boot Ubuntu + Windows**
- Windows role: **complementary** (FEM/ANSYS/mechanics workflows)
- micro-ROS agent: **Docker (official path)**
- Firmware toolchain: **PlatformIO CLI (official path)**

## Quick start (Linux official flow)
1. Install platform dependencies (online):
```bash
./scripts/install_lab.sh --online
```
2. Validate environment:
```bash
./scripts/labctl doctor --profile default
```
3. Build firmware:
```bash
./scripts/labctl firmware build --profile default
```
4. Flash firmware:
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```
5. Start micro-ROS agent:
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```
6. Start GUI:
```bash
./scripts/labctl gui start
```
7. Run smoke test:
```bash
./scripts/labctl smoke --profile default
```
8. (Optional) test a single hardware component:
```bash
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0
```
9. Stop started processes/containers:
```bash
./scripts/labctl stop
```

## Offline bootstrap
To prepare a reusable offline bundle on a machine with internet:
```bash
./scripts/create_offline_bundle.sh
```
Then install offline on lab machine:
```bash
./scripts/install_lab.sh --offline /path/to/offline_bundle_YYYYMMDD_HHMMSS.tar.gz
```

## Software-only validation (no ESP32 required)
Run this from your laptop before touching lab hardware:
```bash
./scripts/test_no_hw.sh
```

Tester-friendly automated report:
```bash
./scripts/tester_report.sh
```

## Main interfaces
- CLI contract: `labctl <subcommand>`
- Profile contract: `config/profiles/*.json`
- Firmware entry: `firmware/softbot_controller/softbot_controller.ino`

## Repository map
```text
config/profiles/          Platform hardware/runtime profiles (JSON)
docs/                     Institutional docs (ES/EN)
firmware/                 ESP32 firmware + PlatformIO project
scripts/                  Installer, offline bundle creator, labctl wrapper
software/cli/             labctl implementation
software/sdk/             Python SDK for ROS topics
software/gui/             Real-time GUI
software/ejemplos/        Operational examples
software/tools/           Validation/calibration/smoke scripts
experiments/              Experimental datasets and operational logs
```

## Documentation hub
- `docs/setup_lab_pc_dualboot_es.md`
- `docs/setup_lab_pc_dualboot_en.md`
- `docs/installation_quickstart_es.md`
- `docs/installation_quickstart_en.md`
- `docs/operations_runbook_es.md`
- `docs/operations_runbook_en.md`
- `docs/maintenance_handover_es.md`
- `docs/maintenance_handover_en.md`
- `docs/legacy_compatibility.md`
- `docs/release_checklist.md`
- `docs/hardware_diagnostics.md`
- `docs/tester_guide_es.md`

## Release policy
- Stable branch: `main`
- Versioning: semantic tags `vX.Y.Z`
- Release gate: checklist + smoke tests + CI green

## Notes for future engineers
- Linux is the official control runtime. Keep Windows for simulation/analysis tooling.
- Keep profiles in JSON and avoid hardcoded machine-specific paths in source files.
- Preserve backward compatibility for legacy scripts under `software/ejemplos/`.
