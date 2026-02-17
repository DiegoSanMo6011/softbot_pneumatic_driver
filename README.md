# SoftBot Lab Platform

Plataforma de control para robot suave neumático con ESP32 + ROS 2 (Linux como runtime oficial).

## Primera vez en una laptop nueva
```bash
cd ~/softbot_pneumatic_driver
./scripts/install_lab.sh --online
exec su -l $USER
```

## Inicio rápido (operación diaria con hardware)
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
./scripts/labctl firmware build --profile default
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
./scripts/labctl hardware verify --timeout-s 8 --sample-timeout-s 3
./scripts/labctl hardware gui --foreground
./scripts/labctl smoke --profile default
./scripts/labctl hardware off
./scripts/labctl stop
```

## Documentación activa (ES)
- `docs/playbook_operacion_equipo_es.md` (documento principal)
- `docs/arquitectura.md`
- `docs/neumatica.md`
- `docs/locomocion.md`
- `docs/installation_quickstart_es.md` (puente temporal)
- `docs/operations_runbook_es.md` (puente temporal)
- `docs/uso_sistema_instalado_es.md` (puente temporal)

## Notas clave
- Flashea firmware antes de levantar el agent.
- Si el agent está activo, el puerto serial queda ocupado.
- Cierre seguro obligatorio: `hardware off` y `labctl stop`.
- Documentación histórica: `docs/archive/`.
