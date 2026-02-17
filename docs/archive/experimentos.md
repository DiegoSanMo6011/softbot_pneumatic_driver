# Experimentos y Datos

## 1. Estructura
Los datos experimentales se almacenan en carpetas por mes:
```
experiments/YYYY-MM/
```

Logs generados por la GUI:
```
experiments/logs/
```

## 2. Convenciones sugeridas
- `calibration_YYYYMMDD_HHMMSS.csv`
- `sweep_kp_XX.csv`
- `sweep_ki_XX.csv`
- `vacuum_sweep_YYYYMMDD_HHMM.csv`
- `matrix_sweep_phaseX_YYYYMMDD.csv`

## 3. Campos recomendados
- `Timestamp_s`
- `Setpoint`
- `Feedback_kPa`
- `PWM_Main`
- `PWM_Aux`
- `Mode`
- `Error`

## 4. Notas
- Mantener el CSV con separador coma.
- Guardar metadatos en un `.md` si el experimento cambia topología o hardware.
