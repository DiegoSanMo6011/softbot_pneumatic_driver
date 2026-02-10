# SoftBot Python SDK & Examples

Este directorio contiene las herramientas de software para interactuar con el **driver neumático SoftBot** desde un entorno de **alto nivel en Python**, utilizando **ROS 2** como middleware de comunicación.
## 📋 Requisitos Previos

- **ROS 2** instalado  
  - Versiones soportadas: **Humble** o **Iron**
- **Python 3**
- Librerías base de ROS 2 (`rclpy`)
## ⚙️ Configuración del Entorno
```bash
### Cargar el entorno de ROS 2
source /opt/ros/humble/setup.bash
```
> ⚠️ Asegúrese de ejecutar este comando en cada nueva terminal antes de correr los scripts.

### Instalación de Dependencias

La librería utiliza exclusivamente módulos estándar de **ROS 2** (`rclpy`).

> ✅ **No se requieren instalaciones adicionales** siempre que el entorno ROS esté correctamente activado.

## 📁 Estructura del Directorio
software/
├── sdk/
│   ├── __init__.py
│   └── softbot_interface.py
└── ejemplos/
    ├── 01_Locomocion_Gusano.py
    ├── 02_Identificacion_Sistema.py
    └── 03_Configurar_Seguridad.py
### Descripción de Archivos

- **`sdk/softbot_interface.py`**  
  Librería principal.  
  Contiene la clase `SoftBot`, encargada de gestionar la comunicación asíncrona con el microcontrolador mediante tópicos ROS 2.
  Incluye `inflate_turbo(pressure_kpa)` para modo 5 (turbo pre-PID).
  Incluye control de **BOOST** via `/boost_valve`.
  Incluye llenado de tanque (modo 3) y lectura de `/tank_state`.
  Incluye **venteo** (modo 4) para liberar presión a atmósfera.

- **`ejemplos/`**  
  Conjunto de scripts listos para ejecución directa que demuestran distintos modos de operación y validación del sistema.
## ▶️ Ejecución de los Ejemplos
1. Asegúrese de que:
   - El **Agente micro-ROS** esté en ejecución.
   - El **ESP32** esté conectado y sincronizado correctamente  
     *(indicador típico: LED parpadeando)*.
2. Navegue al directorio de ejemplos:
```bash
cd software/ejemplos
```
3. Ejecute el script deseado con **Python 3**:
```bash
python3 01_Locomocion_Gusano.py
```
## 🧪 Descripción de los Ejemplos

| Archivo | Descripción Funcional |
|-------|------------------------|
| `01_Locomocion_Gusano.py` | Implementa una **máquina de estados finitos (FSM)** para coordinar las Cámaras A y B en un patrón de **movimiento peristáltico**. |
| `02_Identificacion_Sistema.py` | Barrido multi-escalón para identificación; genera `.csv` con telemetría. |
| `02a_Identificacion_Escalon_Simple.py` | Escalón simple (Step Response) para sintonización rápida. |
| `03_Configurar_Seguridad.py` | **Interfaz de línea de comandos (CLI)** interactiva para ajustar dinámicamente los umbrales de seguridad \(P\_{max}, P\_{min}\) durante la operación. |

## 📌 Notas

- Todos los ejemplos están diseñados para ejecutarse **en tiempo real** con el sistema neumático activo.
- La arquitectura permite una integración directa con **ROS 2**, facilitando la extensión hacia nodos de planeación, control avanzado o supervisión.
