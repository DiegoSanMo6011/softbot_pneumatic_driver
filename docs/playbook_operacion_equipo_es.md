# Playbook operativo del equipo (ES)

Documento único para operación diaria, diagnóstico, capacitación y arranque de trabajo del equipo.

## 1) Flujo operativo oficial (Linux + ESP32)
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

## 2) Pre-check rápido antes de sesión/prueba
```bash
id -nG
docker info
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
./scripts/labctl doctor --profile default
```

## 3) Troubleshooting esencial
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

## 4) Diagnóstico de hardware
## Opción recomendada (GUI MOSFET)
```bash
./scripts/labctl hardware gui --foreground
```

Validar en orden:
1. Válvulas independientes.
2. Bombas por grupo (presión/vacío).
3. `mux_a` y `mux_b`.
4. Auto-OFF y cierre seguro.

## Opción CLI rápida
```bash
./scripts/labctl hardware test --component valve_inflate --duration-s 0.8
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0
./scripts/labctl hardware off
```

## 5) Flujo KiCad (PC de laboratorio)
## Ubicar proyecto oficial local
```bash
find "$HOME" -name '*.kicad_pro' 2>/dev/null
```

## Iteración mínima
1. Abrir esquema y PCB.
2. Verificar librerías y footprints.
3. Ejecutar ERC y DRC.
4. Exportar PDF de esquema y Gerber de prueba.
5. Registrar cambios del día.

## Snapshot diario recomendado
- Formato: `YYYYMMDD_softbot_pcb_snapshot.zip`.

## 6) Checklist de handover (resumen)
- El integrante ejecuta `doctor`, identifica puerto y entiende secuencia completa.
- El integrante corre diagnóstico hardware y cierre seguro.
- El integrante abre KiCad, corre ERC/DRC y exporta evidencia.
- Se asignan responsables y fecha compromiso semanal.

## 7) Plan mensual resumido (4 semanas)
1. Semana 1: baseline PCB + backlog priorizado + handover inicial.
2. Semana 2: cambios eléctricos de mayor impacto.
3. Semana 3: prototipo/banco y validación con firmware/diagnóstico.
4. Semana 4: cierre de transferencia + demo interna.

## 8) Plantilla de mensaje de convocatoria
“Confirmo la capacitación mañana a las 10:30 AM. Sesión práctica de 90 minutos con flujo Linux, diagnóstico de hardware (MOSFET/válvulas/bombas/mux) y arranque de trabajo en KiCad. Cerraremos con asignaciones iniciales y checklist de operación para que el equipo trabaje de forma autónoma desde el primer día.”

## 9) Cierre seguro obligatorio
```bash
./scripts/labctl hardware off
./scripts/labctl stop
```
