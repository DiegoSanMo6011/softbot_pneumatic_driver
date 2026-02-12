# Firmware (ESP32)

Official board profile: `esp32dev` (ESP32 DevKit v1).

## Build with PlatformIO
```bash
./scripts/labctl firmware build --profile default
```

## Flash to board
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```

## Entry source
- `firmware/softbot_controller/softbot_controller.ino`
