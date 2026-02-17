# Guía para Tester Externo (rápida y sencilla)

Esta guía está pensada para un colega que clona el repo por primera vez y debe validar
si la plataforma funciona correctamente.

## 1) Preparación (5 min)
1. Clonar repo y entrar al proyecto.
2. Instalar dependencias:
```bash
./scripts/install_lab.sh --online
```

## 2) Prueba sin hardware (10 min)
Ejecutar reporte automático software-only:
```bash
./scripts/tester_report.sh
```

Qué genera:
- Reporte markdown con PASS/FAIL por paso en `experiments/logs/ops/`
- Logs por paso en carpeta `tester_run_...`

Criterio:
- Si `No-HW suite` falla, reportar como bloqueo de software.
- Si solo falla `Doctor` por serial/Docker daemon, reportar como entorno no listo
  (no necesariamente bug del código).

## 3) Prueba con hardware (cuando conecten ESP32)
1. Conectar ESP32 y confirmar puerto (`/dev/ttyUSB0` o `/dev/ttyACM0`).
2. Ejecutar reporte hardware completo:
```bash
./scripts/tester_report.sh --with-hardware --port /dev/ttyUSB0 --baud 115200
```

Opcional si ya está flasheado el firmware:
```bash
./scripts/tester_report.sh --with-hardware --skip-flash --port /dev/ttyUSB0
```

Incluye:
- Build/flash firmware
- Agent start
- Smoke test
- Test individual de válvulas, mux y bombas
- Apagado final de salidas (`hardware off`) y `labctl stop`

## 4) Evidencia mínima a compartir
Enviar al equipo:
1. Archivo `tester_report_*.md`
2. Carpeta `tester_run_*` correspondiente
3. Comentario breve de sensaciones físicas del hardware (sonido de válvulas, vibración de bombas, etc.)

## 5) Qué hacer si falla algo
- Revisar el log del paso fallido en `tester_run_*`.
- Ejecutar el paso manualmente para reproducir.
- Si falla hardware, usar:
```bash
./scripts/labctl hardware panel --pwm 120
```
- Al terminar, forzar apagado:
```bash
./scripts/labctl hardware off
./scripts/labctl stop
```
