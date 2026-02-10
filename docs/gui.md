# GUI de Escritorio (Tiempo Real)

## 1. Objetivo
Monitorear en tiempo real la presión y señales de depuración, además de enviar comandos
básicos de operación (modo, cámara, setpoint) y registrar datos.

## 2. Ubicación
```
software/gui/softbot_gui.py
```

## 3. Dependencias
Recomendado:
- **PySide6** (Qt)
- **pyqtgraph** (gráficas rápidas)

Instalación sugerida:
```bash
python3 -m pip install pyqtgraph PySide6
```
> Si PySide6 no está disponible, puedes usar `PyQt5`.

Archivo de dependencias:
```
software/gui/requirements.txt
```

## 4. Ejecución
```bash
source /opt/ros/humble/setup.bash
python3 software/gui/softbot_gui.py
```

## 5. Señales mostradas
- Presión (kPa) vs tiempo
- Setpoint (kPa o PWM según modo)
- PWM principal y auxiliar
- Modo actual y error estimado
- Estado de BOOST (tanque)
- Estado del tanque (idle/llenando/lleno/timeout)

## 5.1 Flujo recomendado: Turbo + PID (un clic)
- **Inflar Turbo+PID** ejecuta el modo `5` del firmware:
  - Prefase turbo de 150 ms (boost + inflado agresivo).
  - Transición automática a PID inflado (modo `1`).
- Requiere:
  - Cámara activa en `1`, `2` o `3`.
  - Setpoint positivo (> 0 kPa).

## 5.2 Controles de BOOST manual (diagnóstico)
- **BOOST ON/OFF** abre o cierra manualmente la válvula del tanque.
- **Pulso BOOST** abre por un tiempo configurable (ms).

## 5.3 Controles de llenado de tanque
- **Llenar tanque**: inicia modo 3 con setpoint (kPa).
- **Detener llenado**: corta el modo y apaga actuadores.

## 5.4 Controles de venteo
- **Ventear**: abre A->R para liberar presion durante un tiempo configurable.

## 6. Logging
Los logs de GUI se guardan en:
```
experiments/logs/
```
