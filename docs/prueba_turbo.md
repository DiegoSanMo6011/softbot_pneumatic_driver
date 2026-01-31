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
- Usa **BOOST: ON/OFF** o **Pulso BOOST**.
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

## 6. Nota de seguridad
- No actives BOOST sin tener las cámaras conectadas y límites seguros configurados.
- Ajusta `max_safe` y `min_safe` si hay sobrepresión.
