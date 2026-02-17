# GUIs de Escritorio

## 1) GUI principal de telemetría y control
Script:
```text
software/gui/softbot_gui.py
```

Incluye:
- Telemetría en tiempo real.
- Modos de control PID/PWM/turbo/tanque/venteo.
- Panel de hardware test por componente.

## 2) GUI dedicada de diagnóstico MOSFET
Script:
```text
software/gui/hardware_mosfet_gui.py
```

Incluye:
- Válvulas independientes (`inflate`, `suction`, `boost`).
- Bombas agrupadas por `presión` y `vacío`.
- MUX independientes (`mux_a`, `mux_b`).
- Indicadores tipo LED ON/OFF por actuador.
- Auto-OFF de seguridad por inactividad (10 s).
- Logging CSV automático en `experiments/logs/hardware_diag/`.
- Escenario guiado `Pre-Competition MOSFET Check v1` con marcado PASS/FAIL.

## Dependencias
- `pyqtgraph` (solo para `softbot_gui.py`)
- `PySide6` (o `PyQt5` como fallback)

## Ejecución
```bash
source /opt/ros/humble/setup.bash
python3 -m pip install pyqtgraph PySide6
python3 software/gui/softbot_gui.py
python3 software/gui/hardware_mosfet_gui.py
```

También disponible por CLI:
```bash
./scripts/labctl gui start
./scripts/labctl hardware gui --foreground
```
