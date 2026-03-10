# Arquitectura de control y comunicación

## 1. Vista general
El sistema está dividido en dos capas:

- **Capa embebida (ESP32 + micro-ROS):** control en tiempo real de bombas, válvulas,
  sensores y seguridad.
- **Capa de alto nivel (PC/ROS 2):** GUI, secuencias de locomoción, benchmark, tuning
  y operación diaria.

La ruta operativa vigente usa un **protocolo atómico** para mandar comandos neumáticos,
evitando el problema histórico de publicar cámara, modo y setpoint por tópicos separados.

## 2. Componentes principales
- **ESP32** con firmware en `firmware/softbot_controller/softbot_controller.ino`
- **ADS1115** con dos canales activos:
  - `Ch0` presión
  - `Ch1` vacío
- **Bombas**
  - 2 de presión en paralelo
  - 2 de vacío en paralelo
- **Selección neumática**
  - `MUX A`
  - `MUX B`
  - `Valve Chamber C` sobre el pin legacy BOOST

## 3. Contrato ROS 2 vigente
Los tópicos de control/estado se documentan completos en:

```text
docs/protocolo_neumatico_atomico.md
```

Resumen operativo:

| Tópico | Tipo | Uso |
| --- | --- | --- |
| `/pneumatic_command` | `std_msgs/Int16MultiArray` | Comando atómico nuevo |
| `/pneumatic_state` | `std_msgs/Int16MultiArray` | Estado alto nivel del firmware |
| `/sensor/pressure` | `std_msgs/Float32` | Presión en kPa |
| `/sensor/vacuum` | `std_msgs/Float32` | Vacío en kPa |
| `/system_debug` | `std_msgs/Int16MultiArray` | Debug de bajo nivel |
| `/tuning_params` | `std_msgs/Float32MultiArray` | Ganancias PI y límites |
| `/hardware_test` | `std_msgs/Int16` | Diagnóstico por bitmask |

Tópicos legacy mantenidos temporalmente:

- `/active_chamber`
- `/pressure_mode`
- `/pressure_setpoint`

Esos tópicos están **deprecated** y el firmware los interpreta solo como comportamiento `DIRECT`.

## 4. Máquina de estados neumática
El firmware usa la siguiente secuencia:

```text
IDLE -> PRECHARGE -> READY_HOLD -> DELIVER
  ^         |            |            |
  |         +----> FAULT +----FIRE----+
  +---------------- STOP / VENT ------+
```

Interpretación:

- `PRECHARGE`: la línea de presión o vacío se construye upstream del manifold.
- `READY_HOLD`: la línea queda lista y sostenida, todavía sin abrir hacia cámaras.
- `DELIVER`: se abre la válvula de modo y la máscara de cámaras pedida.
- `DIRECT`: bypass de precarga para compatibilidad legacy o PWM.
- `FAULT`: timeout o safety break.

## 5. Selección de cámaras
La selección sigue siendo bitmask `0..7`:

- `1` = A
- `2` = B
- `4` = C
- combinaciones por OR (`3`, `5`, `6`, `7`)

Notas:

- `0` significa bloqueado en comandos normales.
- En `VENT`, una máscara `0` se interpreta como `ABC`.

## 6. Seguridad
- Límites dinámicos `max_safe` / `min_safe` siguen entrando por `/tuning_params`.
- Si la presión fuente sale del rango seguro, el firmware activa `FAULT` y deja trazabilidad en
  `/system_debug` y `/pneumatic_state`.
- El estado de emergencia queda latched hasta recibir `STOP`.
