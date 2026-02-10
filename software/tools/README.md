# Tools / Calibración

Scripts utilitarios para barridos y calibración.

- `calibrador.py`: barrido de Kp/Ki en inflado y guardado de resultados.
- `bench_turbo_validation.py`: rutina de banco para comparar `mode=1` (PID) vs
  `mode=5` (Turbo+PID) con métricas de tiempo de subida y overshoot.

Los resultados se guardan automáticamente en `experiments/YYYY-MM/`.
