# Rebuild micro-ROS ESP32 (habilitar `/hardware_test` sin perder PID embebido)

## Objetivo
Mantener control PID embebido en la ESP32 y recuperar el tópico `/hardware_test` para diagnóstico independiente de bombas y válvulas.

## Causa raíz del problema
La librería precompilada de `micro_ros_arduino` para ESP32 puede venir con:

- `RMW_UXRCE_MAX_SUBSCRIPTIONS=5`

El firmware de este proyecto usa 6 suscripciones:

1. `/pressure_setpoint`
2. `/pressure_mode`
3. `/active_chamber`
4. `/tuning_params`
5. `/boost_valve`
6. `/hardware_test`

Cuando el límite es 5, la inicialización del nodo puede fallar parcial o totalmente y romper el pipeline de tópicos.

## Solución recomendada
Rebuild de `micro_ros_arduino` para ESP32 con:

- `RMW_UXRCE_MAX_SUBSCRIPTIONS=8`

## Script automatizado
Se agregó:

- `scripts/rebuild_microros_esp32.sh`

Ejecuta:

```bash
cd ~/softbot_pneumatic_driver
./scripts/rebuild_microros_esp32.sh --max-subscriptions 8
```

El script:

1. Garantiza que exista `micro_ros_arduino` en cache de PlatformIO.
2. Modifica `colcon.meta` dentro de la librería.
3. Corre el builder Docker oficial `microros/micro_ros_static_library_builder:humble` para target `esp32`.
4. Verifica que el `config.h` generado tenga `RMW_UXRCE_MAX_SUBSCRIPTIONS=8`.

## Flujo completo después del rebuild
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

./scripts/labctl firmware build --profile default
PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)
./scripts/labctl stop
./scripts/labctl firmware flash --profile default --port "$PORT"
./scripts/labctl agent start --profile default --port "$PORT" --baud 115200
./scripts/labctl hardware verify --timeout-s 12 --sample-timeout-s 5
```

## Resultado esperado en `hardware verify`
Debe salir todo en `OK`, incluyendo:

- `command_types`
- `command_subscribers`

y una muestra válida en:

- `/system_debug`

## Verificación puntual de `/hardware_test`
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
./scripts/labctl hardware verify --timeout-s 12 --sample-timeout-s 5
```

Si se desea inspección adicional:
```bash
docker logs --tail 120 softbot_microros_agent
```

## Advertencias operativas
- `firmware/.pio/` es cache de PlatformIO. Si se limpia cache, puede perderse el rebuild y hay que correr el script otra vez.
- El builder Docker puede generar archivos con dueño `root` en `firmware/.pio/libdeps/...`.
  Si aparecen errores de permisos en builds futuros:
  ```bash
  sudo chown -R $USER:$USER firmware/.pio/libdeps/esp32dev/micro_ros_arduino
  ```

## Cambio de firmware incluido en esta rama
En `firmware/softbot_controller/softbot_controller.ino` se reemplazó `rcl_publish(...)` por un macro `RCSOFTCHECK(...)` que contabiliza errores de publicación y elimina warnings de retorno ignorado.
