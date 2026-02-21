# GUIs de Escritorio

## 1) GUI principal de telemetría y control
Script:
```text
software/gui/softbot_gui.py
```

Incluye:
- Telemetría en tiempo real.
- Modos de control PID/PWM/venteo.
- Selector de cámaras A/B/C por bitmask (`/active_chamber`).
- Panel de hardware test por componente.
- Panel `Benchmark bombas (competencia)` para lanzar pruebas de tiempo a target
  (CSV en `experiments/` y registro acumulado en `experiments/pump_benchmark_registry.csv`).

## 2) GUI dedicada de diagnóstico MOSFET
Script:
```text
software/gui/hardware_mosfet_gui.py
```

Incluye:
- Válvulas independientes (`inflate`, `suction`, `chamber_c`).
- Bombas agrupadas por `presión` y `vacío`.
- MUX independientes (`mux_a`, `mux_b`).
- Indicadores tipo LED ON/OFF por actuador.
- Auto-OFF de seguridad por inactividad (10 s).
- Logging CSV automático en `experiments/logs/hardware_diag/`.
- Escenario guiado `Pre-Competition MOSFET Check v1` con marcado PASS/FAIL.

## 3) GUI dedicada de selección de bomba (presión + vacío)
Script:
```text
software/gui/pump_eval_gui.py
```

Incluye:
- Protocolo dual por corrida: capacidad y tiempo a target en presión y vacío.
- Selector de cámaras A/B/C (bitmask `1..7`, default `ABC=7`).
- Ajuste de ganancias PID (`Kp+`, `Ki+`, `Kp-`, `Ki-`) antes de correr evaluación.
- Safety break configurable por umbrales `Safety +max` / `Safety -min` (corte inmediato).
- Curva en vivo de `kPa` cruda y filtrada (mediana deslizante configurable).
- Comparativo histórico visual seleccionando una o varias filas de registro.
- Guía explícita de líneas verticales de eventos (inicio/fin fase, target, tope).
- Métricas en vivo: topes, tiempos de tope, tiempos a target.
- Criterio `APTA/NO_APTA` (debe cumplir ambos targets en todas las corridas).
- Score balanceado con penalización por variabilidad entre corridas.
- Histórico por etiqueta (`pump_label`) en `experiments/pump_eval_registry.csv`.
- Exportación de CSV raw/summary por sesión.

## 4) GUI dedicada de secuencias de locomoción (editor + runner)
Script:
```text
software/gui/locomotion_gui.py
```

Incluye:
- Editor de fases con columnas de tiempo, tolerancia y transición por `snap`.
- Presets basados en secuencias funcionales actuales (`locomocion_ab`, `x_crabs`) y presets nuevos para 3 cámaras.
- Soporte completo de bitmask A/B/C (`1..7`) y combinaciones (`AB`, `AC`, `BC`, `ABC`).
- Ejecución en loop con `Start/Pause/Resume/Stop/E-STOP`.
- Acciones por fase: `PID inflate`, `PID suction`, `PWM inflate`, `PWM suction`, `vent`, `stop`.
- Telemetría en vivo (presión vs target), log de eventos y exportación de CSV.
- Importación/exportación JSON para compartir secuencias entre equipos.
- Panel de tuning para mantener PID embebido en firmware y ajustar ganancias sin salir de la GUI.

## Dependencias
- `pyqtgraph` (para `softbot_gui.py` y `pump_eval_gui.py`)
- `PySide6` (o `PyQt5` como fallback)

## Ejecución
```bash
source /opt/ros/humble/setup.bash
python3 -m pip install pyqtgraph PySide6
python3 software/gui/softbot_gui.py
python3 software/gui/hardware_mosfet_gui.py
python3 software/gui/pump_eval_gui.py
python3 software/gui/locomotion_gui.py
```

También disponible por CLI:
```bash
./scripts/labctl gui start --foreground
./scripts/labctl gui pump-eval --foreground
./scripts/labctl gui locomotion --foreground
./scripts/labctl hardware gui --foreground
./scripts/labctl benchmark pumps --pump-label actuales --chamber 7 --target-kpa 35 --runs 5
```
