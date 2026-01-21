# SoftBot Pneumatic Driver: Interfaz de Control Neumático Basada en ROS 2

> **Arquitectura de Control en Lazo Cerrado para Actuadores Blandos mediante Micro-ROS y ESP32.**

## 1. Resumen del Proyecto

El presente repositorio alberga la implementación de firmware y software para el **SoftBot Pneumatic Driver**, un sistema embebido diseñado para la gestión precisa de flujo de aire en robots blandos (*soft robotics*).

El sistema actúa como un puente de bajo nivel entre la lógica de control de alto nivel (ejecutada en **ROS 2 Humble/Iron**) y la etapa de potencia neumática. El controlador implementa algoritmos PI discretos para la regulación de presión positiva y negativa (vacío), integra protocolos de seguridad por interbloqueo (*safety interlocks*) y provee una interfaz de telemetría en tiempo real para la identificación de sistemas.

## 🎥 Video de Demostración del Software

El siguiente video presenta una demostración completa del **SoftBot Pneumatic Driver**, mostrando las principales capacidades del sistema, entre ellas:

- Operación en **lazo abierto** tanto en **presión** como en **vacío** para experimentación e identificación de sistemas
- Control **PI en lazo cerrado** para regulación de presión positiva y negativa
- **Sintonización dinámica en tiempo real** de las ganancias del controlador (`Kp`, `Ki`) y de los límites de presión segura (`P_max`, `P_min`)
- Activación inmediata del **Paro de Emergencia (E-STOP)** ante condiciones inseguras

[![SoftBot Pneumatic Driver – Video de Demostración](https://img.youtube.com/vi/uC6NLilY3fU/0.jpg)](https://youtu.be/uC6NLilY3fU)


## 2. Especificaciones Técnicas

### 2.1 Arquitectura del Sistema
El sistema opera bajo una arquitectura distribuida **Maestro-Esclavo**:
* **Esclavo (ESP32):** Gestiona los lazos de control rápido y la seguridad crítica.
* **Maestro/Agente (PC):** Gestiona la planificación de alto nivel.

| Componente | Especificación |
| :--- | :--- |
| **Unidad de Procesamiento (MCU)** | ESP32 (Xtensa® Dual-Core 32-bit LX6) |
| **Middleware** | micro-ROS (DDS sobre transporte Serial UART) |
| **Frecuencia de Muestreo** | 50 Hz (Periodo de control: 20ms) |
| **Resolución PWM** | 8-bit (255 pasos) @ 1 kHz |

### 2.2 Topología de Hardware
El sistema está configurado para manejar una topología neumática compleja:
* **Actuación:** 3x Bombas de diafragma (1x Inflado, 2x Succión en configuración Turbo) y 2x Electroválvulas de control de flujo.
* **Distribución (Multiplexor):** Subsistema de enrutamiento neumático de 2 canales (Cámara A / Cámara B).
* **Adquisición de Datos:** Sensor de presión analógico diferencial interfaseado vía ADS1115 (I2C, 16-bit ADC).

## 3. Funcionalidades Avanzadas

### Sintonización Paramétrica en Tiempo Real (Zero-Flash Tuning)
Implementación de una capa de abstracción que permite la modificación dinámica de las constantes del controlador ($K_p$, $K_i$) y los umbrales de seguridad ($P_{max}$, $P_{min}$) durante la ejecución, eliminando la necesidad de recompilación para ajustes experimentales.

### Modos de Operación Dual
1.  **Control en Lazo Cerrado (PID):** Regulación automática de la presión interna basada en el error respecto al setpoint:
    
2.  **Identificación de Sistemas (Lazo Abierto):** Inyección directa de señales de control (PWM) para caracterización de la planta y obtención de la respuesta al escalón (*Step Response*).

### Protocolo de Seguridad Activa
El firmware incluye un monitor de estado que ejecuta un **Paro de Emergencia (E-STOP)** inmediato, desenergizando la etapa de potencia (MOSFETs) si la presión detectada excede los límites operativos definidos dinámicamente.

## 4. Protocolo de Despliegue e Instalación

### 4.1 Firmware (Microcontrolador)
El código fuente se encuentra estructurado en el directorio `softbot_controller/`.

* **Entorno requerido:** PlatformIO o Arduino IDE 2.0+.
* **Dependencias:** `micro_ros_arduino` (v2.0.7-humble).
* **Procedimiento:** Compilar y cargar `softbot_controller.ino`.

### 4.2 Agente de Comunicación (Host)
Para establecer el enlace DDS, ejecute el agente micro-ROS en el computador principal:

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
```

## 5. Interfaz de Operación (ROS 2 API)

La interacción con el controlador se realiza mediante tópicos estandarizados de ROS 2.
> **Nota:** Para la activación de cualquier actuador, se debe cumplir la condición lógica: `active_chamber != 0`.

### Tabla de Tópicos y Semántica

| Tópico | Tipo de Mensaje | Descripción Funcional |
| :--- | :--- | :--- |
| `/active_chamber` | `std_msgs/Int8` | **Selector de Multiplexor:**<br>`0`: Bloqueo (Idle)<br>`1`: Cámara A<br>`2`: Cámara B<br>`3`: Dual (A+B) |
| `/pressure_mode` | `std_msgs/Int8` | **Selector de Estrategia:**<br>`1`: PI Inflado<br>`-1`: PI Succión<br>`2`: Lazo Abierto Inflado<br>`-2`: Lazo Abierto Succión |
| `/pressure_setpoint` | `std_msgs/Float32` | **Referencia de Control:**<br>En PI: Presión Objetivo ($kPa$)<br>En Lazo Abierto: Ciclo de Trabajo PWM ($0.0 - 255.0$) |
| `/pressure_feedback`| `std_msgs/Float32` | **Variable de Proceso:** Lectura actual del sensor ($kPa$). |
| `/system_debug` | `std_msgs/Int16MultiArray`| **Vector de Telemetría:**<br>`[PWM_Main, PWM_Aux, Error*100, Modo]` |

### Ejemplo de Comando: Secuencia de Inflado Controlado
El siguiente comando activa la Cámara A e inicia el algoritmo PID con un objetivo de 15 kPa:

```bash
ros2 topic pub --once /active_chamber std_msgs/msg/Int8 "{data: 1}" && \
ros2 topic pub --once /pressure_mode std_msgs/msg/Int8 "{data: 1}" && \
ros2 topic pub --once /pressure_setpoint std_msgs/msg/Float32 "{data: 15.0}"
```
## 6. Configuración Dinámica (Tuning Vectorial)

Para la reconfiguración de parámetros en tiempo de ejecución, se debe publicar el vector de configuración en el tópico `/tuning_params`.

**Estructura del Vector:**
`[Kp_Pos, Ki_Pos, Kp_Neg, Ki_Neg, Max_Safe, Min_Safe]`

**Ejemplo: Aumentar ganancia proporcional de inflado a 25.0**

```bash
ros2 topic pub --once /tuning_params std_msgs/msg/Float32MultiArray "{data: [25.0, 300.0, -75.0, -750.0, 45.0, -60.0]}"
```
