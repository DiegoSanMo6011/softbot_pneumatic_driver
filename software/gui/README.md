# GUIs

Interfaces graficas operativas para control, locomocion y diagnostico.

## Archivos principales
- `software/gui/softbot_gui.py`: GUI principal de control general.
- `software/gui/locomotion_gui.py`: GUI de secuencias por fases.
- `software/gui/pump_eval_gui.py`: GUI dedicada de evaluacion dual de bombas.
- `software/gui/hardware_mosfet_gui.py`: GUI de diagnostico por componente.
- `software/gui/sensor_diagnostic_gui.py`: GUI de inspeccion de sensores y ADS1115.

## GUI principal (`softbot_gui.py`)
- Publica control por `/pneumatic_command`.
- Lee estado de alto nivel por `/pneumatic_state`.
- Expone selector manual `Direct / Auto / Arm / Fire`.
- Mantiene `stop`, `vent`, tuning PI y benchmark basico.

## GUI de locomocion (`locomotion_gui.py`)
- Editor de fases con columnas de camara, accion, target, tiempo y `behavior`.
- Modo manual con selector `Direct / Auto / Arm / Fire`.
- Presets base ya migrados al comando atomico.
- La logica vigente asume un solo modo neumatico por fase.

## GUI de evaluacion de bombas (`pump_eval_gui.py`)
- Ejecuta protocolo dual de capacidad y target para presion/vacio.
- Usa el SDK ya migrado al contrato atomico.
- Registra historico en `experiments/`.

## GUI de hardware (`hardware_mosfet_gui.py`)
- Usa `/hardware_test` para activar grupos/componentes individuales.
- El firmware entra a `MODE_HARDWARE_DIAGNOSTIC` por la ruta legacy de diagnostico.
- Se usa para validar MOSFET, bombas, valvulas y mux antes de pruebas funcionales.

## GUI de sensores (`sensor_diagnostic_gui.py`)
- Inspecciona telemetria de `sensor/pressure`, `sensor/vacuum` y debug asociado.
- Util para revisar ADS1115, offsets y ruido antes de cerrar tuning.

## Contrato y referencias
- `docs/protocolo_neumatico_atomico.md`
- `docs/playbook_operacion_equipo_es.md`
- `software/sdk/README.md`
