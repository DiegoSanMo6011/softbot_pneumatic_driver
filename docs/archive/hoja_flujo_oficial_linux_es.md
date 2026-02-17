# Hoja rapida - Flujo oficial Linux (SoftBot)

## Preparacion de terminal
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
```

## Pre-check
```bash
./scripts/labctl doctor --profile default
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

## Secuencia operativa oficial
```bash
./scripts/labctl firmware build --profile default
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
./scripts/labctl hardware gui --foreground
```

## Prueba minima recomendada
```bash
./scripts/labctl smoke --profile default
./scripts/labctl hardware off
```

## Cierre de jornada
```bash
./scripts/labctl hardware off
./scripts/labctl stop
```

## Regla de oro
- Flashear firmware antes de arrancar agent.
- Si el agent esta activo, el puerto serial queda ocupado.
