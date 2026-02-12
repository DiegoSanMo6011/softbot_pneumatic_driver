# SoftBot Lab Platform v1.0

Plataforma reproducible para robots neumáticos suaves con ESP32 + ROS 2, lista para operación diaria en laboratorio y transferencia a futuras generaciones.

## Alcance oficial
- Runtime oficial de control: **Ubuntu 22.04 + ROS 2 Humble**
- Despliegue de laboratorio: **Dual Boot Ubuntu + Windows**
- Rol de Windows: **complementario** (ANSYS/FEM/mecánica), no runtime principal de control
- micro-ROS Agent: **Docker**
- Toolchain de firmware: **PlatformIO CLI**
- Contrato de operación: **`labctl <subcomando>`**

## Instalación en computadora nueva (online, 1 comando)
Estos pasos son para dejar una máquina lista desde cero.

### 1) Clonar repositorio
```bash
git clone https://github.com/DiegoSanMo6011/softbot_pneumatic_driver.git
cd softbot_pneumatic_driver
```

### 2) Instalar dependencias de plataforma
```bash
./scripts/install_lab.sh --online
```

### 3) Abrir nueva terminal y cargar ROS 2
```bash
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
```

Notas:
- Si el instalador agregó tu usuario al grupo `docker`, necesitas nueva sesión de terminal.
- `doctor` puede marcar `Serial devices: none` si no hay ESP32 conectado; eso es esperado.

## Validación rápida sin hardware (sin ESP32)
Úsalo antes de tocar el hardware del laboratorio.

```bash
./scripts/test_no_hw.sh
./scripts/tester_report.sh
```

Se generan logs y reportes en `experiments/logs/ops/`.

## Flujo completo con ESP32 conectada
Reemplaza el puerto si tu equipo usa otro (`/dev/ttyACM0`, etc.).

### 1) Compilar firmware
```bash
./scripts/labctl firmware build --profile default
```

### 2) Flashear firmware
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```

### 3) Levantar micro-ROS Agent
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```

### 4) Abrir GUI
```bash
./scripts/labctl gui start
```

### 5) Ejecutar smoke test seguro
```bash
./scripts/labctl smoke --profile default
```

### 6) Probar componentes de hardware de forma independiente
Ejemplo de prueba puntual:
```bash
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0 --repeat 1
```

Panel interactivo para diagnóstico:
```bash
./scripts/labctl hardware panel --pwm 120
```

Apagado forzado de salidas:
```bash
./scripts/labctl hardware off
```

### 7) Detener procesos y contenedores lanzados por la CLI
```bash
./scripts/labctl stop
```

## Instalación offline (sin internet)
En una máquina con internet:
```bash
./scripts/create_offline_bundle.sh
```

En la máquina objetivo sin internet:
```bash
./scripts/install_lab.sh --offline /ruta/al/offline_bundle_YYYYMMDD_HHMMSS.tar.gz
```

## Flujo para tester externo (rápido)
Para un colega que solo validará la plataforma:
```bash
./scripts/install_lab.sh --online
./scripts/tester_report.sh
```

Con hardware:
```bash
./scripts/tester_report.sh --with-hardware --port /dev/ttyUSB0 --baud 115200
```

Guía detallada: `docs/tester_guide_es.md`

## Mapa del repositorio
```text
config/profiles/          Perfiles de plataforma (JSON)
docs/                     Documentación institucional (ES/EN)
firmware/                 Firmware ESP32 + PlatformIO
scripts/                  Instalador, offline bundle, wrapper labctl
software/cli/             Implementación de labctl
software/sdk/             SDK Python (ROS 2 topics)
software/gui/             GUI de operación en tiempo real
software/ejemplos/        Ejemplos operativos legacy/actuales
software/tools/           Smoke tests, diagnóstico y utilidades
experiments/              Datos experimentales y logs operativos
```

## Documentación principal
- `docs/installation_quickstart_es.md`
- `docs/installation_quickstart_en.md`
- `docs/operations_runbook_es.md`
- `docs/operations_runbook_en.md`
- `docs/setup_lab_pc_dualboot_es.md`
- `docs/setup_lab_pc_dualboot_en.md`
- `docs/hardware_diagnostics.md`
- `docs/maintenance_handover_es.md`
- `docs/maintenance_handover_en.md`
- `docs/release_checklist.md`

## Política de release
- Rama estable: `main`
- Versionado: tags semánticos `vX.Y.Z`
- Gate de release: checklist + smoke tests + CI en verde

## Notas para futuras generaciones
- Mantener Linux como runtime oficial de control.
- Mantener Windows para simulación/análisis (ANSYS/FEM), no control en tiempo real.
- Mantener perfiles versionados en `config/profiles/*.json`.
- Evitar rutas hardcodeadas por máquina.
