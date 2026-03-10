# Protocolo neumático atómico y máquina de estados

Documento operativo de referencia para la arquitectura vigente de control neumático.

## 1. Objetivo
El firmware ahora soporta dos rutas de control:

- **Ruta atómica nueva**: un solo comando describe `modo + cámara(s) + comportamiento + target`.
- **Ruta legacy**: `/active_chamber`, `/pressure_mode` y `/pressure_setpoint` siguen existiendo,
  pero quedan **deprecated** y solo producen comportamiento `DIRECT`.

La ruta atómica elimina las ventanas de carrera entre tópicos y agrega precarga real de la línea
antes de abrir el manifold hacia las cámaras.

## 2. Topología física asumida
La lógica de precarga se basa en esta secuencia física por modo:

- **Presión**: bombas de presión -> sensor de presión -> válvula de presión -> manifold.
- **Vacío**: bombas de vacío -> sensor de vacío -> válvula de vacío -> manifold.
- Del manifold salen tres rutas de cámara:
  - `MUX A`
  - `MUX B`
  - `Valve Chamber C` (pin legacy BOOST)

Durante `PRECHARGE` y `READY_HOLD` el firmware mantiene cerrada la válvula de modo
(`valve_inflate` o `valve_suction`) y la presión/vacío se regula en la sección upstream del sensor.

## 3. Tópicos ROS 2 vigentes
| Tópico | Tipo | Dirección | Productor | Consumidor | Propósito |
| --- | --- | --- | --- | --- | --- |
| `/pneumatic_command` | `std_msgs/Int16MultiArray` | Host -> firmware | SDK/GUI/scripts | ESP32 | Comando atómico de control neumático |
| `/pneumatic_state` | `std_msgs/Int16MultiArray` | Firmware -> host | ESP32 | SDK/GUI/scripts | Estado alto nivel de la máquina neumática |
| `/sensor/pressure` | `std_msgs/Float32` | Firmware -> host | ESP32 | SDK/GUI/scripts | Presión medida por ADS1115 Ch0 |
| `/sensor/vacuum` | `std_msgs/Float32` | Firmware -> host | ESP32 | SDK/GUI/scripts | Vacío medido por ADS1115 Ch1 |
| `/system_debug` | `std_msgs/Int16MultiArray` | Firmware -> host | ESP32 | SDK/GUI/scripts | Debug de bajo nivel `[pwm_main,pwm_aux,ch0_x10,ch1_x10,mode,flags]` |
| `/tuning_params` | `std_msgs/Float32MultiArray` | Host -> firmware | GUI/SDK/tools | ESP32 | Ganancias PI y límites de seguridad |
| `/hardware_test` | `std_msgs/Int16` | Host -> firmware | GUI/tools | ESP32 | Diagnóstico hardware por bitmask |
| `/active_chamber` | `std_msgs/Int8` | Host -> firmware | Clientes legacy | ESP32 | **Deprecated**: cache de cámara, comportamiento `DIRECT` |
| `/pressure_mode` | `std_msgs/Int8` | Host -> firmware | Clientes legacy | ESP32 | **Deprecated**: modo legacy, comportamiento `DIRECT` |
| `/pressure_setpoint` | `std_msgs/Float32` | Host -> firmware | Clientes legacy | ESP32 | **Deprecated**: target legacy, comportamiento `DIRECT` |

## 4. Esquema de `/pneumatic_command`
`std_msgs/Int16MultiArray`, longitud fija `6`.

| Índice | Campo | Valor |
| --- | --- | --- |
| `0` | `version` | `1` |
| `1` | `mode` | `0 stop`, `1 pid_inflate`, `-1 pid_suction`, `2 pwm_inflate`, `-2 pwm_suction`, `4 vent` |
| `2` | `chamber_mask` | bitmask `0..7` (`A=1`, `B=2`, `C=4`) |
| `3` | `behavior` | `0 direct`, `1 auto`, `2 arm`, `3 fire` |
| `4` | `target` | `kPa x10` para PID, `0..255` para PWM, `0` para `stop/vent` |
| `5` | `command_token` | entero ecoado por `/pneumatic_state` |

### 4.1 Reglas de comportamiento
- `DIRECT`: abre la ruta al manifold inmediatamente y regula/actúa sin precarga.
- `AUTO`: precarga la línea, espera target estable y luego entra a `DELIVER`.
- `ARM`: precarga la línea, entra a `READY_HOLD` y espera un `FIRE`.
- `FIRE`: solo es válido desde `READY_HOLD`.

### 4.2 Reglas por modo
- `AUTO`, `ARM` y `FIRE` solo aplican a `PID_INFLATE` y `PID_SUCTION`.
- Los modos PWM son siempre `DIRECT`.
- `VENT` y `STOP` se ejecutan como `DIRECT`.

### 4.3 Regla de `FIRE`
- Si `chamber_mask` es `0`, usa la máscara armada previamente.
- Si `chamber_mask` es distinto de `0`, sobreescribe la máscara armada para la entrega.
- Si no existe un contexto armado válido, el comando se rechaza y se levanta
  `PNEUMATIC_FLAG_COMMAND_REJECTED`.

## 5. Esquema de `/pneumatic_state`
`std_msgs/Int16MultiArray`, longitud fija `10`.

| Índice | Campo | Valor |
| --- | --- | --- |
| `0` | `version` | `1` |
| `1` | `state` | `0 idle`, `1 direct`, `2 precharge`, `3 ready_hold`, `4 deliver`, `5 vent`, `6 fault` |
| `2` | `mode` | modo activo actual |
| `3` | `behavior` | comportamiento del comando vigente |
| `4` | `requested_chamber_mask` | máscara pedida por el último comando aceptado |
| `5` | `applied_chamber_mask` | máscara efectivamente aplicada al manifold |
| `6` | `flags` | bitmask de estado |
| `7` | `target` | target ecoado (`kPa x10` o PWM) |
| `8` | `source_pressure_deci_kpa` | presión/vacío de la línea upstream (`kPa x10`) |
| `9` | `command_token` | eco del último comando aceptado |

### 5.1 Flags de `/pneumatic_state`
| Bit | Flag | Significado |
| --- | --- | --- |
| `0` | `ready` | El sistema está en `READY_HOLD` |
| `1` | `armed` | Existe un contexto armado listo para disparar |
| `2` | `delivering` | El manifold está conectado a cámaras |
| `3` | `timeout` | La precarga expiró antes de alcanzar el target |
| `4` | `command_rejected` | El último comando atómico fue rechazado |
| `5` | `emergency_stop` | El safety break dejó el sistema en fault |
| `6` | `legacy_input` | El comando actual entró por la ruta legacy |

## 6. Máquina de estados
| Estado | Qué hace | Válvula de modo | Cámaras | Bombas |
| --- | --- | --- | --- | --- |
| `IDLE` | Todo apagado | Cerrada | Bloqueadas | Off |
| `DIRECT` | Comportamiento legacy/directo | Abierta | Máscara solicitada | Activas según modo |
| `PRECHARGE` | Construye presión/vacío upstream | Cerrada | Bloqueadas | PI sobre sensor upstream |
| `READY_HOLD` | Mantiene la línea lista sin entregar | Cerrada | Bloqueadas | PI con histéresis |
| `DELIVER` | Entrega al manifold y sostiene regulación | Abierta | Máscara solicitada | PI activo |
| `VENT` | Libera a atmósfera | Desenergizada | Máscara solicitada o `ABC` si `0` | Off |
| `FAULT` | Estado de fallo/timeout/emergency | Cerrada tras secuencia de seguridad | Bloqueadas | Off |

### 6.1 Transiciones principales
- `IDLE -> DIRECT`: comando `DIRECT` o entrada legacy válida.
- `IDLE -> PRECHARGE`: comando `AUTO` o `ARM`.
- `PRECHARGE -> DELIVER`: target estable con comportamiento `AUTO`.
- `PRECHARGE -> READY_HOLD`: target estable con comportamiento `ARM`.
- `READY_HOLD -> DELIVER`: comando `FIRE`.
- `PRECHARGE -> FAULT`: timeout de precarga.
- `* -> VENT`: comando `VENT`.
- `* -> IDLE`: comando `STOP`.
- `* -> FAULT`: safety break por límites de seguridad.

## 7. Umbrales globales embebidos
No son configurables por comando en esta iteración.

| Parámetro | Valor |
| --- | --- |
| `READY_BAND_KPA` | `2.0` |
| `READY_HOLD_MS` | `120` |
| `PRECHARGE_TIMEOUT_MS` | `2500` |
| `READY_RECHARGE_BAND_KPA` | `3.0` |

## 8. Semántica legacy (`deprecated`)
La ruta antigua se conserva para no romper clientes externos de golpe.

- `/active_chamber`, `/pressure_mode` y `/pressure_setpoint` siguen siendo aceptados.
- El firmware ya no intenta volverlos atómicos.
- Cualquier cambio legacy termina en comportamiento `DIRECT`.
- La ruta legacy no soporta `PRECHARGE`, `READY_HOLD`, `ARM` ni `FIRE`.
- Las GUIs y scripts activos del repo ya fueron migrados al tópico atómico.

## 9. Ejemplos de uso desde SDK
```python
from sdk.softbot_interface import SoftBot

bot = SoftBot()

# Precarga y entrega automática
bot.send_pneumatic_command(mode=1, chamber_mask=3, target=35.0, behavior="auto")

# Armar línea de presión sin seleccionar cámara todavía
bot.arm_inflate(30.0, chamber_mask=0)

# Disparar después hacia cámara C
bot.fire(chamber_mask=4)

# PWM directo
bot.direct_command(mode=2, chamber_mask=7, target=180)
```

## 10. Diagnóstico recomendado
- Usar `/pneumatic_state` para validar que la secuencia siga `PRECHARGE -> READY_HOLD/DELIVER`.
- Usar `/system_debug` para ver PWM, sensores codificados y flags de debug del firmware.
- Para validar cableado/MOSFET usar `MODE_HARDWARE_DIAGNOSTIC` vía `/hardware_test`.
