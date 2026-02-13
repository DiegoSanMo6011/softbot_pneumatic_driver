# SoftBot Lab Platform v1.0

Plataforma reproducible para robots neumáticos suaves con ESP32 + ROS 2, lista para operación diaria en laboratorio y transferencia a futuras generaciones.

## Alcance oficial
- Runtime oficial de control: **Ubuntu 22.04 + ROS 2 Humble**
- Despliegue de laboratorio: **Dual Boot Ubuntu + Windows**
- Rol de Windows: **complementario** (ANSYS/FEM/mecánica), no runtime principal de control
- micro-ROS Agent: **Docker**
- Toolchain de firmware: **PlatformIO CLI**
- Contrato de operación: **`labctl <subcomando>`**

## Recursos y permisos que usa la plataforma
- Puerto serial para ESP32: `/dev/ttyUSB*` o `/dev/ttyACM*` (acceso exclusivo).
- Daemon de Docker (`/var/run/docker.sock`) para levantar el micro-ROS Agent.
- Grupos de usuario requeridos: `docker`, `dialout` y opcionalmente `uucp`.
- `docker` permite ejecutar contenedores sin `sudo`.
- `dialout`/`uucp` permite acceder al puerto serial.
- ROS 2 Humble instalado en `/opt/ros/humble/`.
- Logs operativos en `experiments/logs/ops/`.
- Contenedor usado por `labctl agent start`: `softbot_microros_agent` con `--net=host`, `--privileged` y `-v /dev:/dev`.

## Orden oficial de comandos (operación diaria con ESP32)
Ejecuta esta secuencia en este orden:

```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
./scripts/labctl firmware build --profile default
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
./scripts/labctl gui start
./scripts/labctl smoke --profile default
./scripts/labctl stop
```

Notas de operación:
- Flashea **antes** de levantar el agent.
- Si el agent está corriendo, el puerto serial queda ocupado para comunicación ROS.
- `labctl stop` libera procesos y contenedores iniciados por la CLI.

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
- Si el instalador agregó tu usuario a `dialout`/`uucp`, también necesitas nueva sesión.
- `doctor` puede marcar `Serial devices: none` si no hay ESP32 conectado; eso es esperado.

## Validación rápida sin hardware (sin ESP32)
Úsalo antes de tocar el hardware del laboratorio.

```bash
./scripts/test_no_hw.sh
./scripts/tester_report.sh
```

Se generan logs y reportes en `experiments/logs/ops/`.

## Troubleshooting rápido (doctor/instalación)
### 1) `[FAIL] Serial devices: none`
Sin ESP32 conectada: **esperado**.

Con ESP32 conectada:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```
Si no aparece nada, revisar cable USB (datos), puerto físico y reconectar.

### 2) `[FAIL] Docker daemon: not reachable`
```bash
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
newgrp docker
docker info
```
Si sigue fallando:
```bash
sudo systemctl status docker --no-pager
sudo journalctl -u docker -n 60 --no-pager
```
Si cerraste solo una terminal y abriste otra, puede no aplicar el grupo nuevo.
Haz logout/login de escritorio o:
```bash
exec su -l $USER
```

### 3) `bash: /opt/ros/humble/setup.bash: No such file or directory`
ROS 2 Humble no está instalado. Reintentar instalación:
```bash
./scripts/install_lab.sh --online
```

### 4) `E: Unable to locate package docker-compose-plugin`
Actualiza repo y reintenta. El instalador actual ya incluye fallback automático para
`docker-compose-v2` o `docker-compose`:
```bash
git pull
./scripts/install_lab.sh --online
```

### 5) `Could not open /dev/ttyUSB0 ... Permission denied` al flashear
El firmware compila, pero el usuario no tiene permisos de acceso al puerto serial.

Verificar permisos/grupos:
```bash
ls -l /dev/ttyUSB0
groups
```

Agregar usuario a grupos serial (Ubuntu normalmente usa `dialout`):
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG uucp $USER 2>/dev/null || true
```

Cerrar sesión y volver a entrar. Luego reintentar:
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```

Si sigue ocupado el puerto, detectar proceso bloqueando:
```bash
lsof /dev/ttyUSB0
```
Detener procesos lanzados por esta plataforma:
```bash
./scripts/labctl stop
```

### 6) `permission denied ... /var/run/docker.sock` al levantar agent
La sesión actual no tiene permisos efectivos del grupo `docker`.

Verificar sesión y socket:
```bash
id -nG
ls -l /var/run/docker.sock
```

Aplicar permisos y refrescar sesión:
```bash
sudo usermod -aG docker $USER
newgrp docker
docker info
```

Si sigue fallando:
```bash
sudo systemctl enable --now docker
sudo systemctl restart docker
exec su -l $USER
id -nG
docker info
```
Si `id -nG` no muestra `docker`, hacer logout/login completo o reiniciar.

Desbloqueo temporal (si urge probar):
```bash
sudo ./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```

### 7) `labctl gui start` dice "Started gui" pero no aparece ventana
`labctl gui start` ejecuta la GUI en background. Para depurar, correr en foreground:
```bash
./scripts/labctl gui start --foreground
```

Si estás por SSH/TTY sin entorno gráfico, no abrirá ventana. Verificar:
```bash
echo "$XDG_SESSION_TYPE $DISPLAY"
```
Debe haber sesión gráfica y `DISPLAY` no vacío.

Si aparece error de plugin Qt `xcb`, instalar dependencias:
```bash
sudo apt update
sudo apt install -y \
  libxcb-cursor0 libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 \
  libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-xinerama0 \
  libxcb-xinput0 libxcb-xfixes0
```

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
- `docs/dualboot_step_by_step_es.md`
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
