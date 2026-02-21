# Locomoción — Estrategias y Script Maestro

## 1. Script principal (competencia)
Archivo:
```
software/locomotion/x_crabs.py
```

Incluye un motor de fases con transición por tolerancia y tiempos mínimos/máximos.

## 2. Estrategias actuales
- **SYNC AB (Saltar)**: succión + inflado sincronizado de ambas cámaras.
- **CAMINATA (Alternada)**: A y B alternan tracción/recuperación con comandos mixtos.
- **GIRO IZQUIERDA / DERECHA**: pivote usando una sola cámara.
- **RANDOM**: secuencia pseudoaleatoria para desatasque.

## 3. Parámetros críticos
- `P_HIGH`, `P_LOW` (kPa)
- `min_time`, `max_time`
- `tol` (tolerancia de llegada)
- `settle_time` (estabilidad antes de transición)
- `snap_ms` (freno breve entre fases)

## 4. Notas de operación
- El modo salto es foco principal de competencia.
- La selección de cámara ahora usa bitmask A/B/C (`/active_chamber` 0..7).
- `x_crabs.py` opera en modos PID/PWM/VENT sin rutas turbo/tank-fill.
- Se recomienda registrar datos durante cada cambio de estrategia.

## 5. Script alterno
```
software/locomotion/locomocion_ab.py
```
Loop automático sincronizado (A+B) con asentamiento.

## 6. Flujo recomendado para experimentar secuencias
Opción recomendada (GUI dedicada):
1. `./scripts/labctl agent start --profile default --port <PORT> --baud 115200`
2. `./scripts/labctl gui locomotion --foreground`
3. Cargar preset base (`AB settle`, `x_crabs sync AB`, `3-chamber wave`) y ajustar fases.
4. Ejecutar, validar y exportar JSON/CSV para reproducibilidad.

Opción legacy (`x_crabs.py` + GUI principal):
1. `./scripts/labctl agent start --profile default --port <PORT> --baud 115200`
2. `./scripts/labctl gui start --foreground` (telemetría/log y benchmark)
3. `python3 software/locomotion/x_crabs.py`

Notas:
- Si `x_crabs.py` está controlando, evita mandar comandos simultáneos desde GUI.
- Para comparar bombas (actuales vs nuevas), corre `./scripts/labctl benchmark pumps ...`
  antes y después del cambio.
