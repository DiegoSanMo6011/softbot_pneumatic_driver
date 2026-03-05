# Playbook operativo del equipo (ES)

Documento principal para operación en laboratorio, diagnóstico de hardware, capacitación y handover.

## 1) Qué es cada bloque del sistema
- **Host Linux (PC del laboratorio):** aquí corres `labctl`, GUI, scripts Python y herramientas de desarrollo.
- **ESP32 (firmware):** ejecuta el controlador neumático en tiempo real.
- **micro-ROS Agent (Docker):** puente entre ROS 2 en Linux y la ESP32 por serial.
- **GUI principal (`gui start`):** telemetría + control + benchmark de bombas para experimentación.
- **GUI de hardware (`hardware gui`):** interfaz dedicada para validar conexión eléctrica/MOSFET y actuadores.
- **GUI de evaluación de bombas (`gui pump-eval`):** caracterización dedicada de presión + vacío y ranking histórico.
- **GUI de locomoción (`gui locomotion`):** editor/runner de secuencias con soporte A/B/C (`1..7`) y pruebas reproducibles por JSON.

## 1.1) GUIs: qué hace cada una y cómo correrla
### GUI principal (`gui start`)
- **Qué hace:** control general (PID/PWM/vent), telemetría en vivo y benchmark básico.
- **Cuándo usarla:** operación diaria, tuning rápido y monitoreo durante pruebas.
- **Cómo correrla:**
```bash
./scripts/labctl gui start --foreground
```

### GUI de hardware (`hardware gui`)
- **Qué hace:** diagnóstico eléctrico por componente (bombas, válvulas y mux) con enfoque MOSFET.
- **Cuándo usarla:** validación previa de cableado/actuadores antes de locomoción.
- **Cómo correrla:**
```bash
./scripts/labctl hardware gui --foreground
```

### GUI de evaluación de bomba (`gui pump-eval`)
- **Qué hace:** evaluación dual de presión + vacío con métricas de tope/tiempo, estado `APTA` y ranking histórico.
- **Cuándo usarla:** comparar etiquetas de bombas y decidir la bomba idónea.
- **Cómo correrla:**
```bash
./scripts/labctl gui pump-eval --foreground
```

### GUI de locomoción (`gui locomotion`)
- **Qué hace:** permite diseñar, ejecutar y validar secuencias por fases (PID/PWM/vent/stop) con bitmask de 3 cámaras (`A/B/C` y combinaciones).
- **Cuándo usarla:** iteración rápida de locomoción, validación de secuencias ya funcionales y experimentación controlada.
- **Cómo correrla:**
```bash
./scripts/labctl gui locomotion --foreground
```

## 2) Flujo oficial y propósito de cada comando
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
./scripts/labctl firmware build --profile default
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
./scripts/labctl hardware gui --foreground
./scripts/labctl gui start --foreground
./scripts/labctl gui locomotion --foreground
./scripts/labctl smoke --profile default
```

### `source /opt/ros/humble/setup.bash`
- **Qué hace:** carga entorno ROS 2 en la terminal actual.
- **Dónde corre:** host Linux.
- **Para qué:** sin esto, GUI/scripts ROS no encuentran paquetes y tópicos.

### `./scripts/labctl doctor --profile default`
- **Qué hace:** valida entorno (Linux, Python, ROS, Docker, PlatformIO, perfil, serial).
- **Dónde corre:** host Linux.
- **Para qué:** detectar bloqueos antes de iniciar demo/pruebas.

### `./scripts/labctl firmware build --profile default`
- **Qué hace:** compila el firmware ESP32.
- **Dónde corre:** host Linux (PlatformIO local).
- **Para qué:** confirmar que el código compila antes de flashear.

### `./scripts/labctl firmware flash --profile default --port <PORT>`
- **Qué hace:** graba el firmware en la ESP32 por serial.
- **Dónde corre:** host Linux; usa el puerto USB de la ESP32.
- **Para qué:** cargar versión actual de control al microcontrolador.

### `./scripts/labctl agent start --profile default --port <PORT> --baud 115200`
- **Qué hace:** levanta contenedor `softbot_microros_agent`.
- **Dónde corre:** Docker en host Linux (`--net=host`, acceso a `/dev`).
- **Para qué:** habilitar comunicación ROS 2 <-> ESP32.
- **Importante:** cuando el agent está activo, el puerto serial queda ocupado por el agent.

### `./scripts/labctl hardware gui --foreground`
- **Qué hace:** abre GUI dedicada de diagnóstico MOSFET.
- **Dónde corre:** host Linux con entorno gráfico (`DISPLAY` activo).
- **Para qué está pensada:** validar que cada salida eléctrica/actuador responde como esperado antes de experimentar locomoción.
- **Uso típico:** chequeo rápido pre-demo/pre-entrenamiento.

### `./scripts/labctl gui start --foreground`
- **Qué hace:** abre la GUI principal de telemetría/control.
- **Dónde corre:** host Linux con entorno gráfico (`DISPLAY` activo).
- **Para qué está pensada:** experimentación de control y locomoción (incluye panel de benchmark de bombas).
- **Uso típico:** ajustar setpoints/tuning, ver telemetría en vivo y correr pruebas comparativas de bombas.

### `./scripts/labctl gui pump-eval --foreground`
- **Qué hace:** abre la GUI dedicada para evaluar bombas en protocolo dual (capacidad + target).
- **Dónde corre:** host Linux con entorno gráfico (`DISPLAY` activo).
- **Para qué está pensada:** selección de bomba ideal midiendo presión y vacío sobre cámaras seleccionables A/B/C (bitmask `1..7`, default `ABC=7`).
- **Uso típico:** comparar etiquetas de bomba (`pump_label`) y revisar ranking histórico por `score_final`.

### `./scripts/labctl gui locomotion --foreground`
- **Qué hace:** abre la GUI dedicada de secuencias de locomoción con editor de fases, presets y runner con loop configurable.
- **Dónde corre:** host Linux con entorno gráfico (`DISPLAY` activo).
- **Para qué está pensada:** experimentar locomoción con soporte de 3 cámaras manteniendo PID embebido en firmware.
- **Uso típico:** cargar preset funcional, ajustar tiempos/tolerancias, correr pruebas y exportar JSON/CSV para reproducibilidad.

### `./scripts/labctl smoke --profile default`
- **Qué hace:** ejecuta secuencia segura mínima de comandos de control.
- **Dónde corre:** host Linux (publica por ROS 2).
- **Para qué:** sanity check de pipeline completo.
- **Nota:** puede “pasar” aunque no haya hardware real si la ruta de control no está verificando respuesta física.

### `./scripts/labctl benchmark pumps --pump-label <label> ...`
- **Qué hace:** corre benchmark repetible de tiempo a presión objetivo para comparar setups de bombas.
- **Dónde corre:** host Linux (publica por ROS 2).
- **Para qué está pensada:** validar rápido si un cambio de bombas mejora tiempo de llegada sin perder control.

## 3) Catálogo completo de scripts (`scripts/`)
### `./scripts/labctl`
- **Qué hace:** wrapper principal de la plataforma. Ejecuta `software/cli/labctl.py`.
- **Dónde corre:** host Linux.
- **Para qué:** punto único de operación diaria (doctor, firmware, agent, gui, benchmark, hardware, smoke, stop).

### `./scripts/install_lab.sh`
- **Qué hace:** instala dependencias de plataforma (modo online/offline).
- **Dónde corre:** host Linux (requiere permisos de sistema en varios pasos).
- **Para qué:** preparar una máquina nueva o reparar entorno roto.

### `./scripts/create_offline_bundle.sh`
- **Qué hace:** genera bundle de instalación para máquinas sin internet.
- **Dónde corre:** host Linux con internet.
- **Para qué:** despliegue offline en laboratorio o equipos restringidos.

### `./scripts/test_no_hw.sh`
- **Qué hace:** ejecuta validaciones sin hardware (lint, formato, checks de CLI/perfiles).
- **Dónde corre:** host Linux.
- **Para qué:** validar software/base del repo antes de usar ESP32.

### `./scripts/tester_report.sh`
- **Qué hace:** produce reporte automático de validación (software-only o con hardware).
- **Dónde corre:** host Linux.
- **Para qué:** evidencia formal de estado de plataforma para testers/equipo.

### `./scripts/requirements_lab.txt`
- **Qué es:** lista de dependencias Python de la plataforma.
- **Dónde se usa:** por `install_lab.sh`/bootstrap de entorno.
- **Para qué:** mantener reproducibilidad de entorno.

## 4) Pre-check rápido antes de sesión
```bash
id -nG
docker info
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
./scripts/labctl doctor --profile default
```

Interpretación rápida:
- Si no aparece serial: revisar cable/puerto/ESP32.
- Si Docker falla: no podrá arrancar `agent start`.

## 4.1) Recuperación de laptop nueva (errores de `doctor`)
Si al arrancar ves algo como:
- `[FAIL] ROS 2 Humble setup: /opt/ros/humble/setup.bash`
- `[FAIL] docker command: missing`
- `[FAIL] PlatformIO: PlatformIO not found...`
- `[FAIL] Docker daemon: docker not available`

aplica esta secuencia exacta:
```bash
cd ~/softbot_pneumatic_driver
./scripts/install_lab.sh --online
exec su -l $USER
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
```

Qué corrige esta secuencia:
1. Instala ROS 2 Humble (`/opt/ros/humble/setup.bash`).
2. Instala Docker y habilita servicio.
3. Crea `.venv` e instala dependencias Python/PlatformIO.
4. Aplica grupos de usuario (`docker`, `dialout`/`uucp`) al re-login.

Si aún falla Docker después de instalar:
```bash
sudo systemctl enable --now docker
docker info
```

Si sale `[FAIL] Serial devices: none`:
- Es normal si la ESP32 no está conectada.
- Si sí está conectada, revisar cable USB de datos y usar:
```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg | tail -n 50
```

## 5) Requisitos para que las GUIs funcionen con hardware real
Las GUIs pueden abrir aunque no haya conexión real, pero para que **controlen la ESP32** debe existir este camino activo:
1. ESP32 conectada por USB y puerto detectado.
2. Firmware cargado en la ESP32.
3. `agent start` corriendo con el puerto correcto.
4. GUI abierta en sesión gráfica (`DISPLAY` válido).

Secuencia recomendada de arranque:
```bash
PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)
./scripts/labctl firmware flash --profile default --port "$PORT"
./scripts/labctl agent start --profile default --port "$PORT" --baud 115200
./scripts/labctl hardware gui --foreground
```

Si no corre `agent start`, la GUI abre pero no tendrá puente ROS2<->ESP32 activo.

Para abrir la GUI principal de control (en otra terminal):
```bash
./scripts/labctl gui start --foreground
```

Resumen de uso recomendado:
- `hardware gui`: primero, para validar electrónica y actuadores.
- `gui start`: después, para telemetría/control/experimentos.

Verificación rápida del puente:
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
ros2 topic list | egrep "sensor/|pressure_mode|hardware_test|system_debug|active_chamber"
```

Resultado esperado con sistema activo:
- Nodo `soft_robot_node` visible.
- Tópicos de control/telemetría visibles (`/pressure_mode`, `/sensor/pressure`, `/sensor/vacuum`, `/system_debug`, etc.).
- Selección de cámara por bitmask en `/active_chamber`: `A=1`, `B=2`, `C=4`, combinaciones hasta `7`.

## 6) Diagnóstico de hardware (intención y secuencia)
Comando recomendado:
```bash
./scripts/labctl hardware gui --foreground
```

Orden de validación sugerido:
1. Válvulas independientes.
2. Bombas por grupo (presión/vacío).
3. `mux_a` y `mux_b`.
4. Confirmar auto-OFF/cierre seguro.

Propósito del diagnóstico:
- Confirmar cableado, MOSFET, conectores y salidas.
- Evitar perder tiempo depurando algoritmos cuando el problema es eléctrico.

CLI alternativa rápida:
```bash
./scripts/labctl hardware test --component valve_inflate --duration-s 0.8
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0
./scripts/labctl hardware off
```

Test robusto recomendado (contrato ROS de firmware + `system_debug`):
```bash
./scripts/labctl hardware verify --timeout-s 8 --sample-timeout-s 3
```

Qué valida `hardware verify`:
1. Existe el nodo de firmware `soft_robot_node`.
2. Existen tópicos de telemetría esperados y tienen publisher (`/sensor/pressure`, `/sensor/vacuum`, `/system_debug`).
3. Existen tópicos de comando esperados y tienen subscriber del firmware.
4. Llega al menos un mensaje real en `/system_debug` (si no usas `--no-system-debug-sample`).

## 6.1) Benchmark de bombas para competencia (antes/después del cambio)
Objetivo:
- Medir si las bombas nuevas llegan más rápido al mismo target de presión.
- Dejar evidencia en CSV para decisión técnica rápida.

Flujo recomendado:
1. Corre baseline con bombas actuales.
2. Cambia bombas.
3. Repite exactamente la misma prueba con nueva etiqueta.
4. Compara `time_to_target_mean_s` y `rise_t10_90_mean_s`.

Comandos ejemplo (misma configuración para comparar):
```bash
./scripts/labctl benchmark pumps --pump-label actuales --mode pid --chamber 7 --target-kpa 35 --runs 7
./scripts/labctl benchmark pumps --pump-label nuevas_v1 --mode pid --chamber 7 --target-kpa 35 --runs 7
```

Archivos que genera:
- `experiments/YYYY-MM/pump_bench_raw_*.csv`
- `experiments/YYYY-MM/pump_bench_summary_*.csv`
- `experiments/pump_benchmark_registry.csv` (histórico acumulado)

Alternativa por GUI principal:
1. Abre `./scripts/labctl gui start --foreground`.
2. En panel **Benchmark bombas (competencia)** define etiqueta (`actuales`, `nuevas_v1`, etc).
3. Ejecuta benchmark y guarda resultados automáticos en `experiments/`.

## 6.2) Evaluación dual de bomba (presión + vacío) con GUI dedicada
Objetivo:
- Seleccionar bomba con métricas de tope y tiempo tanto en presión como en vacío.
- Usar score balanceado con penalización por variabilidad entre corridas.

Comando:
```bash
./scripts/labctl gui pump-eval --foreground
```

Flujo recomendado:
1. Define `pump_label` (ej: `actuales`, `nuevas_v2`).
2. Selecciona cámaras A/B/C (por defecto `ABC (7)`) y configura targets.
3. Ajusta ganancias PID (`Kp+`, `Ki+`, `Kp-`, `Ki-`) si vas a evaluar control cerrado.
4. Ajusta límites de seguridad (`Safety +max`, `Safety -min`) para habilitar corte preventivo.
5. Ejecuta corrida dual y confirma estado `APTA/NO_APTA`.
6. Repite con otra etiqueta y compara en histórico (solo `APTA`, ordenado por `score_final`).
7. En el histórico, selecciona una o varias filas para ver comparativo visual de curvas en la gráfica.

Archivos generados por sesión:
- `experiments/YYYY-MM/pump_eval_raw_*.csv`
- `experiments/YYYY-MM/pump_eval_summary_*.csv`
- `experiments/pump_eval_registry.csv`

## 6.3) Experimentación de locomoción con GUI dedicada
Para iterar secuencias de locomoción con editor visual y telemetría:

Terminal 1:
```bash
PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)
./scripts/labctl agent start --profile default --port "$PORT" --baud 115200
```

Terminal 2:
```bash
./scripts/labctl gui locomotion --foreground
```

Flujo recomendado:
1. Cargar preset base (`AB settle`, `x_crabs sync AB`, `3-chamber wave`, etc.).
2. Ajustar tiempos (`min/max`), tolerancia y `snap` por fase.
3. Definir `loop count` (0 = infinito) y ejecutar.
4. Exportar JSON de la secuencia validada y CSV de telemetría.

## 7) Troubleshooting esencial
## Docker daemon no reachable
```bash
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
exec su -l $USER
docker info
```

## Puerto serial no aparece
- Revisar cable USB de datos.
- Cambiar puerto físico.
- Verificar `dmesg | tail -n 50`.

## Permission denied en serial
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG uucp $USER 2>/dev/null || true
exec su -l $USER
```

## GUI no abre
```bash
./scripts/labctl hardware gui --foreground
./scripts/labctl gui start --foreground
echo "$XDG_SESSION_TYPE $DISPLAY"
```

## `agent start` corre, pero no aparecen nodos/tópicos de ESP32
Esto indica que Docker/agent están arriba, pero la sesión micro-ROS con la ESP32 no quedó enlazada.

Secuencia de recuperación recomendada:
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
./scripts/labctl stop
PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)
./scripts/labctl agent start --profile default --port "$PORT" --baud 115200
docker logs -f softbot_microros_agent
```

Después de levantar el agent:
1. Presionar `EN/RESET` en la ESP32 (o desconectar/conectar USB).
2. En otra terminal:
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list | egrep "sensor/|pressure_mode|hardware_test|system_debug|active_chamber"
```

Notas importantes:
- El mensaje `No such container: softbot_microros_agent` al arrancar `agent start` es normal (intenta borrar el contenedor anterior).
- Si el firmware arrancó sin agent disponible, puede requerir reset físico de la ESP32 para registrar correctamente el nodo.

## Cómo confirmar que la GUI de hardware sí envía señal a bombas/MOSFET
Abrir monitoreo ROS mientras operas la GUI:
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /hardware_test
ros2 topic echo /pressure_mode
ros2 topic echo /system_debug
```

Validaciones mínimas:
1. En GUI activar **Bombas de Presión**:
   - `/pressure_mode` debe ir a `9` (modo diagnóstico).
   - `/hardware_test` debe mostrar máscara `3`.
2. En GUI activar **Bombas de Vacío**:
   - `/hardware_test` debe mostrar máscara `12`.
3. En GUI activar ambas:
   - `/hardware_test` debe mostrar máscara `15`.
4. En `/system_debug`, el payload debe ser `[pwm_main,pwm_aux,ch0_x10,ch1_x10,mode,flags]`.

Interpretación:
- Si ROS muestra cambios y el LED MOSFET no enciende: problema en ruta eléctrica (driver, MOSFET, alimentación, cableado o board).
- Si no hay cambios en `/hardware_test`: problema de publicación GUI/ROS.
- Si hay cambios en `/hardware_test` pero no aparece `soft_robot_node`: revisar enlace agent-ESP32 y reset.
- Si `hardware verify` falla en `/system_debug sample`: revisar que la ESP32 esté enlazada al agent, resetear ESP32 y repetir.

## `/hardware_test` ausente pero telemetría presente
Si `soft_robot_node` aparece y tienes `/sensor/pressure`, `/sensor/vacuum` y `/system_debug`, pero falta `/hardware_test`,
la causa probable es límite de suscripciones en la librería precompilada de `micro_ros_arduino`.

Rebuild recomendado:
```bash
cd ~/softbot_pneumatic_driver
./scripts/rebuild_microros_esp32.sh --max-subscriptions 8
```

Después del rebuild:
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)
./scripts/labctl stop
./scripts/labctl firmware flash --profile default --port "$PORT"
./scripts/labctl agent start --profile default --port "$PORT" --baud 115200
./scripts/labctl hardware verify --timeout-s 12 --sample-timeout-s 5
```

Referencia detallada:
- `docs/microros_rebuild_esp32_es.md`

## 8) Flujo KiCad (PC de laboratorio)
Ubicar proyecto oficial:
```bash
find "$HOME" -name '*.kicad_pro' 2>/dev/null
```

Iteración mínima:
1. Abrir esquema y PCB.
2. Verificar librerías/footprints.
3. Ejecutar ERC y DRC.
4. Exportar PDF de esquema y Gerber.
5. Registrar cambios del día.

Snapshot diario recomendado:
- `YYYYMMDD_softbot_pcb_snapshot.zip`.

## 9) Checklist de handover (resumen)
- Cada integrante ejecuta `doctor`, identifica puerto y entiende secuencia completa.
- Cada integrante corre diagnóstico hardware y cierre seguro.
- Cada integrante abre KiCad, corre ERC/DRC y exporta evidencia.

## 10) Plan mensual resumido (4 semanas)
1. Semana 1: baseline PCB + backlog priorizado + handover inicial.
2. Semana 2: cambios eléctricos de mayor impacto.
3. Semana 3: prototipo/banco y validación con firmware/diagnóstico.
4. Semana 4: cierre de transferencia + demo interna.

## 11) Cierre seguro obligatorio
```bash
./scripts/labctl hardware off
./scripts/labctl stop
```
