# Instalacion rapida - ES

## Requisitos
- Ubuntu 22.04 LTS
- Python 3.10+
- Acceso sudo
- ESP32 conectado por USB

## Online (1 comando)
```bash
./scripts/install_lab.sh --online
```

## Verificacion
```bash
./scripts/labctl doctor --profile default
```

## Prueba sin hardware (sin ESP32)
```bash
./scripts/test_no_hw.sh
```

## Reporte automático para tester
```bash
./scripts/tester_report.sh
```

## Flujo minimo de uso
1. Compilar firmware:
```bash
./scripts/labctl firmware build --profile default
```
2. Cargar firmware:
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```
3. Levantar micro-ROS agent:
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```
4. Abrir GUI:
```bash
./scripts/labctl gui start
```
5. Ejecutar smoke test:
```bash
./scripts/labctl smoke --profile default
```
6. Detener procesos/contenedores lanzados por `labctl`:
```bash
./scripts/labctl stop
```

## Offline
Generar bundle en equipo con internet:
```bash
./scripts/create_offline_bundle.sh
```
Instalar en equipo sin internet:
```bash
./scripts/install_lab.sh --offline /ruta/al/bundle.tar.gz
```
