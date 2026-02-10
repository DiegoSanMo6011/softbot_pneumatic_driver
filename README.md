# SoftBot Pneumatic Driver — RoboSoft inPipe Locomotion (Competencia)

> **Sistema neumático en lazo cerrado para locomoción en robots blandos (SoftBot).**
> Este repositorio contiene firmware, software de alto nivel, experimentos y documentación
> orientada a la competencia **RoboSoft – inPipe Locomotion**.

## 1. Resumen
El proyecto integra:
- **Firmware en ESP32** con micro-ROS para control neumático en lazo cerrado.
- **SDK en Python (ROS 2)** para teleoperación, experimentación y tuning.
- **Scripts de locomoción** (incluye salto y caminata alternada).
- **GUI de escritorio** para telemetría en tiempo real y debugging.
- **Modo 5 Turbo pre-PID** para inflado rápido y transición automática a PID.
- **Datos de experimentación** y documentación técnica.

## 2. Estructura del repositorio
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

## 3. Quick Start
### 3.1 Firmware (ESP32)
Código principal en:
```
firmware/softbot_controller/softbot_controller.ino
```
Requiere PlatformIO o Arduino IDE 2.0+ y la librería `micro_ros_arduino`.

### 3.2 Agente micro-ROS (PC)
Ejecuta el agente:
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
```

### 3.3 SDK y ejemplos (Python)
```bash
source /opt/ros/humble/setup.bash
cd software/ejemplos
python3 01_Locomocion_Gusano.py
```

### 3.4 Locomoción de competencia (X-CRABS)
Script principal:
```
software/locomotion/x_crabs.py
```

### 3.5 GUI en tiempo real
Instrucciones completas en:
```
docs/gui.md
```

## 4. Documentación (todo en español)
- `docs/arquitectura.md` — arquitectura de control, tópicos ROS 2 y seguridad
- `docs/locomocion.md` — estrategias de locomoción y parámetros clave
- `docs/experimentos.md` — estructura y uso de datos experimentales
- `docs/gui.md` — GUI de escritorio en tiempo real
- `docs/neumatica.md` — esquema neumático y diagrama actualizado
- `docs/competencia_inpipe.md` — checklist y objetivos para RoboSoft
- `docs/prueba_turbo.md` — guía de prueba del tanque BOOST

## 5. Datos experimentales
Los CSV históricos están en `experiments/2026-01/` y los nuevos logs se guardan en
carpetas por mes: `experiments/YYYY-MM/`.

## 6. Video de demo
[![SoftBot Pneumatic Driver – Video de Demostración](https://img.youtube.com/vi/uC6NLilY3fU/0.jpg)](https://youtu.be/uC6NLilY3fU)

## 7. Calidad de código
Validaciones recomendadas:
```bash
python3 -m py_compile $(find software -type f -name "*.py")
ruff check software
ruff format --check software
clang-format --dry-run --Werror firmware/softbot_controller/softbot_controller.ino
```
