# Prueba del Turbo (Tanque + Válvula BOOST)

## 1. Cableado
1. Conecta la válvula BOOST al pin definido en el firmware:
   - `PIN_VALVE_BOOST` en `firmware/softbot_controller/softbot_controller.ino`
2. Verifica que la válvula sea **NC** y que en reposo quede **A->R abierto / P cerrado**.
3. Revisa que el tanque esté con sus **check valves** de carga y descarga.

## 2. Compilar y cargar firmware
```bash
# Compilar y cargar con Arduino IDE o PlatformIO
# Archivo: firmware/softbot_controller/softbot_controller.ino
```

## 3. Ejecutar micro-ROS agent
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
```

## 4. Probar desde GUI
```bash
source /opt/ros/humble/setup.bash
python3 software/gui/softbot_gui.py
```
- Usa **Inflar Turbo+PID** para probar el flujo recomendado (modo `5`).
- Usa **BOOST: ON/OFF** o **Pulso BOOST** para diagnóstico manual.
- Usa **Llenar tanque** para cargar hasta el setpoint.
- Observa presión vs tiempo.

## 5. Probar desde CLI
```bash
source /opt/ros/humble/setup.bash
python3 software/ejemplos/08_boost_test.py
```

## 5.1 Probar llenado de tanque (CLI)
```bash
source /opt/ros/humble/setup.bash
python3 software/ejemplos/09_tank_fill.py
```

## 5.2 Probar venteo (GUI)
Usa el botón **Ventear** para liberar presión a atmósfera.

## 5.3 Prueba rápida del modo 5 por tópico
```bash
ros2 topic pub /active_chamber std_msgs/msg/Int8 "{data: 3}" -1
ros2 topic pub /pressure_setpoint std_msgs/msg/Float32 "{data: 35.0}" -1
ros2 topic pub /pressure_mode std_msgs/msg/Int8 "{data: 5}" -1
```

## 5.4 Rutina de banco automatizada (PID vs Turbo+PID)
```bash
source /opt/ros/humble/setup.bash
python3 software/tools/bench_turbo_validation.py --target-kpa 35 --chamber 3 --runs 5
```

Salida:
- `experiments/YYYY-MM/bench_turbo_raw_*.csv`
- `experiments/YYYY-MM/bench_turbo_summary_*.csv`

Métricas clave:
- `rise_t10_90_s`
- `time_to_target_s`
- `overshoot_kpa`

## 6. Nota de seguridad
- No actives BOOST sin tener las cámaras conectadas y límites seguros configurados.
- Ajusta `max_safe` y `min_safe` si hay sobrepresión.
