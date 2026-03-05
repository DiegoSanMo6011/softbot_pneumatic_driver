# Arquitectura de Control y Comunicación

## 1. Vista general
El sistema está dividido en dos capas:
- **Capa embebida (ESP32 + micro-ROS):** controla bombas, válvulas y seguridad crítica.
- **Capa de alto nivel (PC/ROS 2):** planificación, experimentación, tuning y GUI.

## 2. Hardware base
- MCU: **ESP32**
- Sensor: **ADS1115** (I2C, 16-bit) con sensor de presión analógico
- Actuadores: bombas de inflado y succión + válvulas direccionales
- Selección neumática:
  - `MUX A` (pin dedicado)
  - `MUX B` (pin dedicado)
  - `Válvula Cámara C` (pin legacy BOOST)

## 3. Selección de cámara por bitmask
El tópico `/active_chamber` usa `std_msgs/Int8` con máscara de 3 bits:

- bit 0 (`1`): cámara A
- bit 1 (`2`): cámara B
- bit 2 (`4`): cámara C

Combinaciones válidas:
- `0`: bloqueado
- `1`: A
- `2`: B
- `3`: A+B
- `4`: C
- `5`: A+C
- `6`: B+C
- `7`: A+B+C

Regla especial en modo `VENT`:
- Si llega `active_chamber=0`, el firmware ventea `A+B+C` (`mask=7`).

## 4. Tópicos ROS 2
| Tópico | Tipo | Descripción |
| --- | --- | --- |
| `/active_chamber` | `std_msgs/Int8` | Bitmask 0..7 (A/B/C y combinaciones) |
| `/pressure_mode` | `std_msgs/Int8` | 1: PID inflado, -1: PID succión, 2: PWM inflado, -2: PWM succión, 4: venteo, 9: diagnóstico de hardware |
| `/pressure_setpoint` | `std_msgs/Float32` | Setpoint (kPa o PWM según modo) |
| `/sensor/pressure` | `std_msgs/Float32` | Sensor ADS1115 Ch0 (kPa) |
| `/sensor/vacuum` | `std_msgs/Float32` | Sensor ADS1115 Ch1 (kPa) |
| `/system_debug` | `std_msgs/Int16MultiArray` | [PWM_main, PWM_aux, ch0*10, ch1*10, mode, flags] |
| `/tuning_params` | `std_msgs/Float32MultiArray` | [Kp_pos, Ki_pos, Kp_neg, Ki_neg, max_safe, min_safe] |
| `/hardware_test` | `std_msgs/Int16` | Bitmask de salidas para modo 9 (bombas/válvulas/mux) |

### 4.1 Bitmask de `/hardware_test` (modo 9)
- bit 0: bomba inflado main
- bit 1: bomba inflado aux
- bit 2: bomba succión main
- bit 3: bomba succión aux
- bit 4: válvula inflado
- bit 5: válvula succión
- bit 6: válvula cámara C (pin legacy BOOST)
- bit 7: mux cámara A
- bit 8: mux cámara B

## 5. Seguridad
- Límites dinámicos de presión (max_safe, min_safe)
- E-STOP inmediato ante sobrepresión
- Reset de integradores al cambiar modo

## 6. Firmware
Código principal:
```
firmware/softbot_controller/softbot_controller.ino
```

## 7. SDK
Interfaz Python:
```
software/sdk/softbot_interface.py
```
