# Uso del sistema ya instalado - ES

Guia operativa para usar la plataforma en una PC donde ya se completo la instalacion.
Esta guia evita reinstalar y se enfoca en operacion diaria.

## Objetivo
- Levantar el sistema de forma reproducible en pocos minutos.
- Evitar errores de permisos, puertos y procesos ocupados.
- Estandarizar el flujo para operadores nuevos del laboratorio.

## Antes de iniciar (check rapido)
1. Estar en sesion grafica local (no solo TTY/SSH) si se abrira GUI.
2. Verificar grupos del usuario:
```bash
id -nG
```
Debe incluir `docker` y `dialout` (y `uucp` si existe).
3. Verificar Docker:
```bash
docker info
```
4. Verificar puerto ESP32:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

## Flujo recomendado (orden oficial)
Ejecutar en este orden:

```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
./scripts/labctl firmware build --profile default
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
./scripts/labctl gui start --foreground
```

Con sistema estable, en otra terminal puedes ejecutar:
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl smoke --profile default
```

## Operacion por terminales (recomendado)
- Terminal A: micro-ROS Agent
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```
- Terminal B: GUI
```bash
./scripts/labctl gui start --foreground
```
- Terminal C: pruebas (smoke, ejemplos, hardware)
```bash
./scripts/labctl smoke --profile default
./scripts/labctl example run 09_tank_fill --foreground
```

## Diagnostico de hardware por componente
Pruebas unitarias de actuadores:
```bash
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0 --repeat 1
./scripts/labctl hardware test --component valve_inflate --duration-s 0.8 --repeat 1
./scripts/labctl hardware panel --pwm 120
./scripts/labctl hardware off
```

## Cierre seguro de jornada
```bash
./scripts/labctl hardware off
./scripts/labctl stop
```

Respaldar logs/resultados:
- `experiments/logs/ops/`
- `experiments/`

## Errores comunes y accion inmediata
1. Error `permission denied ... /var/run/docker.sock`
```bash
sudo usermod -aG docker $USER
exec su -l $USER
docker info
```
2. Error `Could not open /dev/ttyUSB0 ... Permission denied`
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG uucp $USER 2>/dev/null || true
exec su -l $USER
```
3. `labctl gui start` no muestra ventana
```bash
./scripts/labctl gui start --foreground
echo "$XDG_SESSION_TYPE $DISPLAY"
```
4. `ModuleNotFoundError: PySide6`
```bash
./scripts/install_lab.sh --online
```

## Que NO hacer
- No correr `install_lab.sh` en cada inicio de jornada.
- No abrir agent y flash al mismo tiempo sobre el mismo puerto serial.
- No dejar el sistema activo al retirarse del laboratorio.
