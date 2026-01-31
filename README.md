# SoftBot Pneumatic Driver — RoboSoft inPipe Locomotion

> **Sistema neumático en lazo cerrado para locomoción en robots blandos (SoftBot).**
> Repositorio técnico con firmware, software de alto nivel, experimentos y documentación
> orientada a la competencia **RoboSoft – inPipe Locomotion**.

## Highlights técnicos (visión rápida)
- **Control PI dual** (presión y vacío) con **micro‑ROS** sobre **ESP32**.
- **Topología neumática biestable** con modo **BOOST** para impulsos de salto.
- **SDK Python (ROS 2)** para teleoperación, tuning dinámico y experimentación.
- **GUI en tiempo real** para telemetría, debugging y logging.
- **Scripts de locomoción** (salto sincronizado, caminata alternada, giros, desatasque).
- **Seguridad activa**: E‑STOP, límites dinámicos y venteo controlado.

## Resumen
El proyecto integra un sistema completo para locomoción neumática en robots suaves:

- **Firmware embebido (ESP32)** con micro‑ROS para control en lazo cerrado.
- **SDK en Python (ROS 2)** para control de alto nivel y pruebas reproducibles.
- **GUI de escritorio** con telemetría y logs experimentales.
- **Documentación técnica** (arquitectura, neumática, locomoción, competencia).

## Arquitectura en 30 segundos

```
[ ROS 2 / PC ]
   |  (SDK + GUI + Scripts de locomoción)
   v
[ micro‑ROS Agent ]  <->  [ ESP32 ]  ->  Bombas / Válvulas / Tanque
                                ^
                             Sensor presión (ADS1115)
```

- **Capa embebida:** control rápido, seguridad y telemetría.
- **Capa alto nivel:** experimentos, tuning y ejecución de locomoción.

## Modos de operación (resumen)
- **PID inflado / succión** (regulación a setpoint en kPa).
- **PWM inflado / succión** (lazo abierto para identificación).
- **BOOST** para salto (tanque y válvula dedicada).
- **Venteo** controlado para seguridad y reset rápido.

## Seguridad
- Límites dinámicos de presión (max_safe / min_safe).
- E‑STOP inmediato por sobrepresión.
- Reset de integradores al cambiar de modo.

## Estructura del repositorio
```
firmware/
software/
  sdk/
  locomotion/
  tools/
  gui/
  ejemplos/
experiments/
  2026-01/
  logs/
docs/
hardware/
  pneumatica/
media/
```

## Quick Start
### 1) Firmware (ESP32)
Código principal en:
```
firmware/softbot_controller/softbot_controller.ino
```
Requiere PlatformIO o Arduino IDE 2.0+ y `micro_ros_arduino`.

### 2) Agente micro‑ROS (PC)
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
```

### 3) SDK y ejemplos (Python)
```bash
source /opt/ros/humble/setup.bash
cd software/ejemplos
python3 01_Locomocion_Gusano.py
```

### 4) Locomoción de competencia (X‑CRABS)
Script principal:
```
software/locomotion/x_crabs.py
```

### 5) GUI en tiempo real
Instrucciones completas en:
```
docs/gui.md
```

## Documentación (todo en español)
- `docs/arquitectura.md` — arquitectura de control, tópicos ROS 2 y seguridad
- `docs/locomocion.md` — estrategias de locomoción y parámetros clave
- `docs/experimentos.md` — estructura y uso de datos experimentales
- `docs/gui.md` — GUI de escritorio en tiempo real
- `docs/neumatica.md` — esquema neumático y diagrama actualizado
- `docs/competencia_inpipe.md` — checklist y objetivos para RoboSoft
- `docs/prueba_turbo.md` — guía de prueba del tanque BOOST

## Datos experimentales
Los CSV históricos están en `experiments/2026-01/` y los nuevos logs se guardan en
carpetas por mes: `experiments/YYYY-MM/`.

## Demo
[![SoftBot Pneumatic Driver – Video de Demostración](https://img.youtube.com/vi/uC6NLilY3fU/0.jpg)](https://youtu.be/uC6NLilY3fU)

## Estado actual
- Enfoque en locomoción de competencia (**salto sincronizado** como estrategia principal).
- Integración de **tanque BOOST** para mayor impulso.
- Registro continuo de datos con GUI y logs mensuales.

