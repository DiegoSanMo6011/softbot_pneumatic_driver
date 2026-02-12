# Changelog

All notable changes to this project will be documented in this file.

The format is based on Keep a Changelog.

## [Unreleased]

## [1.0.0] - 2026-02-12
### Added
- PlatformIO project file for ESP32 firmware (`firmware/platformio.ini`).
- Lab installer (`scripts/install_lab.sh`) and offline bundle generator (`scripts/create_offline_bundle.sh`).
- Unified CLI (`scripts/labctl` + `software/cli/labctl.py`).
- Platform profiles (`config/profiles/default_lab.json`, `config/profiles/esp32_devkit_v1.json`).
- Safe smoke-test tool (`software/tools/smoke_lab.py`).
- Institutional documentation set in Spanish and English.
- Governance artifacts: `LICENSE`, `CONTRIBUTING.md`, CI workflow, release checklist.

### Changed
- Fixed firmware `RCCHECK` macro to restore C/C++ compilation.
- Fixed `set_pwm` call order in `software/ejemplos/04_demo_completa.py`.
- Updated `software/ejemplos/02_Identificacion_Sistema.py` to use physical PID setpoints (kPa).
- Updated `README.md` as the platform hub with `labctl`-based workflow.
