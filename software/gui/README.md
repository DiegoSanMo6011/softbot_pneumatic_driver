# GUI de Escritorio

Script principal:
```
software/gui/softbot_gui.py
```

Dependencias:
- pyqtgraph
- PySide6 (o PyQt5 si PySide6 no está disponible)

Ejemplo:
```bash
source /opt/ros/humble/setup.bash
python3 -m pip install pyqtgraph PySide6
python3 software/gui/softbot_gui.py
```

Incluye controles de **BOOST** (tanque) para abrir/cerrar la válvula o enviar un pulso.
Incluye controles de **llenado de tanque** (modo 3) con setpoint configurable.
Incluye **venteo** para liberar presión a atmósfera.
