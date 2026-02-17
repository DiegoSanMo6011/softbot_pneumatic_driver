# Installation Quickstart - EN

## Requirements
- Ubuntu 22.04 LTS
- Python 3.10+
- sudo access
- ESP32 connected over USB

## Online (1 command)
```bash
./scripts/install_lab.sh --online
```

## Validation
```bash
./scripts/labctl doctor --profile default
```

## Software-only check (no ESP32)
```bash
./scripts/test_no_hw.sh
```

## Minimal operating flow
1. Build firmware:
```bash
./scripts/labctl firmware build --profile default
```
2. Flash firmware:
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```
3. Start micro-ROS agent:
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```
4. Start GUI:
```bash
./scripts/labctl gui start
```
5. Run smoke test:
```bash
./scripts/labctl smoke --profile default
```
6. Stop `labctl` processes/containers:
```bash
./scripts/labctl stop
```

## Offline
Create bundle on internet-connected machine:
```bash
./scripts/create_offline_bundle.sh
```
Install on offline machine:
```bash
./scripts/install_lab.sh --offline /path/to/bundle.tar.gz
```
