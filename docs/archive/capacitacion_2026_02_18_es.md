# Capacitacion de arranque del equipo (18 feb 2026, 10:30 AM)

## Objetivo de la sesion
- Transferir operacion base del sistema Linux + ROS2 + firmware + diagnostico de hardware.
- Alinear la forma de trabajo del equipo para iniciar mejora de electronica/PCB desde el dia 1.
- Reducir dependencia operativa en una sola persona.

## Alcance (90 minutos)
- Flujo oficial Linux en vivo.
- Diagnostico de hardware (MOSFET, valvulas, bombas, mux).
- Arranque del flujo de trabajo KiCad en la PC del laboratorio.
- Asignacion inicial de responsables por frente de electronica.

## Requisitos previos
- ESP32 conectada por USB.
- Docker operativo.
- ROS2 Humble disponible en `/opt/ros/humble/setup.bash`.
- KiCad instalado en la PC del laboratorio con esquematicos/footprints.

## Agenda minuto a minuto
## 10:30-10:40 Contexto y seguridad
- Arquitectura: Linux + ROS2 + ESP32 + neumatica + GUI de diagnostico.
- Reglas de seguridad:
  - no dejar actuadores energizados sin supervision,
  - cerrar con `./scripts/labctl hardware off` y `./scripts/labctl stop`.

## 10:40-11:00 Flujo Linux oficial (hands-on)
- Cada integrante ejecuta:
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
```
- Validar puerto serial:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```
- Explicar secuencia oficial:
```bash
./scripts/labctl firmware build --profile default
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```

## 11:00-11:20 Demo hardware en vivo
- Abrir GUI de diagnostico:
```bash
./scripts/labctl hardware gui --foreground
```
- Validar:
  - valvulas independientes,
  - bombas por grupo (presion/vacio),
  - mux_a y mux_b,
  - auto-off y cierre seguro.

## 11:20-11:45 Modulo KiCad (laboratorio)
- Abrir el proyecto oficial local en KiCad.
- Revisar librerias y footprints.
- Ejecutar ERC/DRC.
- Exportar evidencia minima (PDF esquema + gerber de prueba).

## 11:45-12:00 Asignacion inicial de trabajo
- Mariano:
  - normalizacion de librerias y `footprint table`.
- Daniel:
  - revision electrica (drivers MOSFET, protecciones, conectores).
- Sebastian:
  - layout/placement/ruteo + reglas DRC.
- Diego:
  - soporte transversal + validacion contra hardware real.

## Escenarios de validacion de la sesion
- S0: `doctor` sin fallas criticas (serial puede faltar antes de conectar ESP32).
- S1: build + flash + agent + GUI sin bloqueos.
- S2: diagnostico fisico de actuadores con confirmacion visual.
- S3: KiCad abre proyecto, pasa ERC/DRC base y exporta artefacto.
- S4: cierre seguro final (`hardware off`, `stop`).

## Criterio de salida de la sesion
- Los 3 nuevos integrantes ejecutan el flujo base con supervision minima.
- Se completa checklist de handover y se firma.
- Quedan tareas semanales asignadas con fecha objetivo.
