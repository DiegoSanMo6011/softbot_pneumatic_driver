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
├── softbot_interface.py
└── ejemplos/
    ├── 01_locomocion_gusano.py
    ├── 02_identificacion_sistema.py
    └── 03_configurar_seguridad.py
### Descripción de Archivos

- **`softbot_interface.py`**  
  Librería principal.  
  Contiene la clase `SoftBot`, encargada de gestionar la comunicación asíncrona con el microcontrolador mediante tópicos ROS 2.

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
python3 01_locomocion_gusano.py
```
## 🧪 Descripción de los Ejemplos

| Archivo | Descripción Funcional |
|-------|------------------------|
| `01_locomocion_gusano.py` | Implementa una **máquina de estados finitos (FSM)** para coordinar las Cámaras A y B en un patrón de **movimiento peristáltico**. |
| `02_identificacion_sistema.py` | Realiza una **prueba de respuesta al escalón (Step Response)** y genera un archivo `.csv` con precisión en **milisegundos**, adecuado para análisis científico y control. |
| `03_configurar_seguridad.py` | **Interfaz de línea de comandos (CLI)** interactiva para ajustar dinámicamente los umbrales de seguridad \(P\_{max}, P\_{min}\) durante la operación. |

## 📌 Notas

- Todos los ejemplos están diseñados para ejecutarse **en tiempo real** con el sistema neumático activo.
- La arquitectura permite una integración directa con **ROS 2**, facilitando la extensión hacia nodos de planeación, control avanzado o supervisión.



