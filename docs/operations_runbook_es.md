# Runbook operativo - ES

## Inicio de jornada
1. Conectar ESP32 y verificar puerto serial:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```
2. Validar entorno:
```bash
./scripts/labctl doctor --profile default
```
3. Compilar/cargar firmware si aplica:
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```
4. Levantar agente:
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```
5. Ejecutar smoke test:
```bash
./scripts/labctl smoke --profile default
```
6. Iniciar GUI o ejemplo:
```bash
./scripts/labctl gui start
./scripts/labctl example run 09_tank_fill
```

## Modo tester (reporte completo)
- Sin hardware:
```bash
./scripts/tester_report.sh
```
- Con hardware:
```bash
./scripts/tester_report.sh --with-hardware --port /dev/ttyUSB0 --baud 115200
```

## Diagnóstico por componente (cuando se sospecha falla de hardware)
```bash
./scripts/labctl hardware test --component valve_inflate --duration-s 0.8
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0
./scripts/labctl hardware panel --pwm 120
./scripts/labctl hardware off
```

## Cierre de jornada
1. Detener procesos/contenedores gestionados por `labctl`:
```bash
./scripts/labctl stop
```
2. Respaldar resultados de `experiments/`.
3. Documentar incidencias en bitacora del laboratorio.

## Troubleshooting rapido
- `doctor` falla en Docker daemon:
  - cerrar sesion e iniciar sesion nuevamente (grupo docker)
  - o ejecutar `sudo systemctl start docker`
- No aparece `/dev/ttyUSB0`:
  - revisar cable/driver
  - usar `dmesg | tail -n 50`
- GUI no abre:
  - verificar ROS: `source /opt/ros/humble/setup.bash`
  - revisar log en `experiments/logs/ops/`

## Logs operativos
`labctl` guarda eventos y logs en:
- `experiments/logs/ops/labctl_events.log`
- `experiments/logs/ops/*.log`
