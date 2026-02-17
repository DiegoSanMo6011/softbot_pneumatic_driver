# Operations Runbook - EN

## Start of day
1. Connect ESP32 and verify serial port:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```
2. Validate environment:
```bash
./scripts/labctl doctor --profile default
```
3. Build/flash firmware when needed:
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```
4. Start agent:
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```
5. Run smoke test:
```bash
./scripts/labctl smoke --profile default
```
6. Start GUI or example:
```bash
./scripts/labctl gui start
./scripts/labctl example run 09_tank_fill
```

## Component-level diagnostics (when hardware faults are suspected)
```bash
./scripts/labctl hardware test --component valve_inflate --duration-s 0.8
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0
./scripts/labctl hardware panel --pwm 120
./scripts/labctl hardware off
```

## End of day
1. Stop processes/containers managed by `labctl`:
```bash
./scripts/labctl stop
```
2. Back up `experiments/` outputs.
3. Record issues in the lab logbook.

## Quick troubleshooting
- `doctor` fails on Docker daemon:
  - log out/in (docker group refresh)
  - or run `sudo systemctl start docker`
- `/dev/ttyUSB0` missing:
  - inspect cable/driver
  - run `dmesg | tail -n 50`
- GUI does not start:
  - verify ROS env: `source /opt/ros/humble/setup.bash`
  - inspect logs under `experiments/logs/ops/`

## Operational logs
`labctl` writes logs to:
- `experiments/logs/ops/labctl_events.log`
- `experiments/logs/ops/*.log`
