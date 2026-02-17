# Playbook operativo del equipo (ES)

Documento principal para operación en laboratorio, diagnóstico de hardware, capacitación y handover.

## 1) Qué es cada bloque del sistema
- **Host Linux (PC del laboratorio):** aquí corres `labctl`, GUI, scripts Python y herramientas de desarrollo.
- **ESP32 (firmware):** ejecuta el controlador neumático en tiempo real.
- **micro-ROS Agent (Docker):** puente entre ROS 2 en Linux y la ESP32 por serial.
- **GUI de hardware (`hardware gui`):** interfaz para validar conexión eléctrica/MOSFET y actuadores.

## 2) Flujo oficial y propósito de cada comando
```bash
cd ~/softbot_pneumatic_driver
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
./scripts/labctl firmware build --profile default
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
./scripts/labctl hardware gui --foreground
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

### `./scripts/labctl smoke --profile default`
- **Qué hace:** ejecuta secuencia segura mínima de comandos de control.
- **Dónde corre:** host Linux (publica por ROS 2).
- **Para qué:** sanity check de pipeline completo.
- **Nota:** puede “pasar” aunque no haya hardware real si la ruta de control no está verificando respuesta física.

## 3) Pre-check rápido antes de sesión
```bash
id -nG
docker info
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
./scripts/labctl doctor --profile default
```

Interpretación rápida:
- Si no aparece serial: revisar cable/puerto/ESP32.
- Si Docker falla: no podrá arrancar `agent start`.

## 4) Diagnóstico de hardware (intención y secuencia)
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

## 5) Troubleshooting esencial
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
echo "$XDG_SESSION_TYPE $DISPLAY"
```

## 6) Flujo KiCad (PC de laboratorio)
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

## 7) Checklist de handover (resumen)
- Cada integrante ejecuta `doctor`, identifica puerto y entiende secuencia completa.
- Cada integrante corre diagnóstico hardware y cierre seguro.
- Cada integrante abre KiCad, corre ERC/DRC y exporta evidencia.

## 8) Plan mensual resumido (4 semanas)
1. Semana 1: baseline PCB + backlog priorizado + handover inicial.
2. Semana 2: cambios eléctricos de mayor impacto.
3. Semana 3: prototipo/banco y validación con firmware/diagnóstico.
4. Semana 4: cierre de transferencia + demo interna.

## 9) Cierre seguro obligatorio
```bash
./scripts/labctl hardware off
./scripts/labctl stop
```
