# Locomoción — Estrategias y Script Maestro

## 1. Script principal (competencia)
Archivo:
```
software/locomotion/x_crabs.py
```

Incluye un **motor de fases** con transición por tolerancia y tiempos mínimos/máximos.

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
- El **modo salto** es el foco principal de la competencia.
- Existe un **límite físico de brusquedad**; por eso se planea integrar tanque + válvula 2/2 como “turbo”.
- Se recomienda registrar datos durante cada cambio de estrategia.

## 6. BOOST (tanque)
En `software/locomotion/x_crabs.py` puedes activar el boost:
- `BOOST_ENABLE = True`
- `BOOST_PULSE_MS = 150.0`

## 5. Script alterno
```
software/locomotion/locomocion_ab.py
```
Loop automático sincronizado (A+B) con asentamiento.
