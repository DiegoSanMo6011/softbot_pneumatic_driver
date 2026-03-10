# Firmware (ESP32)

## Archivo principal
```text
firmware/softbot_controller/softbot_controller.ino
```

## Qué implementa
- Control PI discreto de presión y vacío.
- Protocolo atómico `/pneumatic_command`.
- Telemetría de estado `/pneumatic_state`.
- Compatibilidad temporal con `/active_chamber`, `/pressure_mode` y `/pressure_setpoint`
  en modo `DIRECT`.
- Diagnóstico hardware por `/hardware_test`.

## Build
```bash
./scripts/labctl firmware build --profile default
```

## Flash
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```

## Contrato ROS 2 documentado en
```text
docs/protocolo_neumatico_atomico.md
```

## Perfil oficial
- Board: `esp32dev`
- Framework: Arduino + micro-ROS
