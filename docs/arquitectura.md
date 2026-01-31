# Arquitectura de Control y Comunicación

## 1. Vista general
El sistema está dividido en dos capas:
- **Capa embebida (ESP32 + micro-ROS):** controla bombas, válvulas y seguridad crítica.
- **Capa de alto nivel (PC/ROS 2):** planificación, experimentación, tuning y GUI.

## 2. Hardware base
- MCU: **ESP32**
- Sensor: **ADS1115** (I2C, 16-bit) con sensor de presión analógico
- Actuadores: bombas de inflado y succión + válvulas direccionales
- Mux neumático: selección de cámara A/B o ambas

## 3. Topología neumática (resumen)
- Inflado: bomba principal + bomba auxiliar (modo turbo)
- Succión: bomba principal + bomba auxiliar
- Válvulas: inflado/succión + selección de cámara

> El diagrama detallado se documenta en `docs/neumatica.md`.

## 4. Tópicos ROS 2
| Tópico | Tipo | Descripción |
| --- | --- | --- |
| `/active_chamber` | `std_msgs/Int8` | 0: bloqueado, 1: A, 2: B, 3: A+B |
| `/pressure_mode` | `std_msgs/Int8` | 1: PID inflado, -1: PID succión, 2: PWM inflado, -2: PWM succión, **3: llenado de tanque**, **4: venteo** |
| `/pressure_setpoint` | `std_msgs/Float32` | Setpoint (kPa o PWM según modo) |
| `/pressure_feedback` | `std_msgs/Float32` | Presión medida en kPa |
| `/system_debug` | `std_msgs/Int16MultiArray` | [PWM_main, PWM_aux, error*10, mode] |
| `/tuning_params` | `std_msgs/Float32MultiArray` | [Kp_pos, Ki_pos, Kp_neg, Ki_neg, max_safe, min_safe] |
| `/boost_valve` | `std_msgs/Int8` | 0: cerrada, 1: abierta (turbo/tanque) |
| `/tank_state` | `std_msgs/Int8` | 0: idle, 1: llenando, 2: lleno, 3: timeout |

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
