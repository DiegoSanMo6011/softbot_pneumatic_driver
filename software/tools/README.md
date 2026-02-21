# Tools / Calibración

Scripts utilitarios para barridos y calibración.

- `calibrador.py`: barrido de Kp/Ki en inflado y guardado de resultados.
- `bench_turbo_validation.py`: rutina de banco en modo PID para medir tiempo de
  subida y overshoot con distintas máscaras de cámara.
- `pump_swap_validation.py`: benchmark para comparar configuraciones de bombas
  (ej. actuales vs nuevas) por tiempo a presión objetivo y registro histórico.
- `pump_eval_core.py`: núcleo de evaluación dual para selección de bomba (capacidad
  y tiempos en presión/vacío), con score balanceado y registro histórico.
- `controller_id_capture.py`: captura de datos de identificación en lazo abierto
  (PWM directo) para inflado/succión.
- `root_locus_discrete_tuner.py`: ajuste de modelo ARX discreto + búsqueda de
  ganancias PI con root locus.
- `controller_validation_report.py`: comparación baseline vs candidato con métricas
  de tiempo a target, overshoot y settling (PASS/FAIL).
- `smoke_lab.py`: secuencia segura de verificación operacional para `labctl smoke`.
- `hardware_component_tester.py`: diagnóstico por componente (bombas, válvulas y mux)
  para validar hardware de forma independiente.

Los resultados se guardan automáticamente en `experiments/YYYY-MM/`.
