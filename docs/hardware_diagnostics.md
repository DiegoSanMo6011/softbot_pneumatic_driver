# Diagnóstico de Hardware (bombas, válvulas y mux)

## Objetivo
Probar componentes individuales para descartar fallas eléctricas/neumáticas sin depender
completamente de una rutina de locomoción.

## Requisitos
- Firmware actualizado con modo `9` (hardware diagnostic).
- micro-ROS agent corriendo.
- ESP32 conectado por USB.

## Opción A: CLI rápida con `labctl`
Prueba puntual por componente:
```bash
./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0 --repeat 2
./scripts/labctl hardware test --component valve_boost --duration-s 0.8
./scripts/labctl hardware test --component mux_a --duration-s 0.8
```

Apagar todo:
```bash
./scripts/labctl hardware off
```

Panel interactivo en terminal:
```bash
./scripts/labctl hardware panel --pwm 120
```

## Opción B: GUI (botones)
En `software/gui/softbot_gui.py`, usa el panel:
- **Hardware test (componentes)**
- Marca/desmarca salidas.
- Ajusta PWM.
- Presiona **Aplicar HW Test**.
- Presiona **HW OFF** al terminar.

## Componentes disponibles
- `inflate_main`
- `inflate_aux`
- `suction_main`
- `suction_aux`
- `valve_inflate`
- `valve_suction`
- `valve_boost`
- `mux_a`
- `mux_b`

## Secuencia recomendada de validación
1. `valve_inflate`, `valve_suction`, `valve_boost` (clic/sonido de válvula).
2. `mux_a`, `mux_b` (confirmar conmutación).
3. Bombas una por una (`inflate_main`, `inflate_aux`, `suction_main`, `suction_aux`).
4. Combinaciones mínimas (ejemplo: `valve_inflate + inflate_main`).
5. Terminar con `hardware off` o `HW OFF`.

## Seguridad
- Usa PWM bajo al inicio (80-120).
- No mantener actuadores energizados sin supervisión.
- Confirmar límites de seguridad en firmware (`max_safe`, `min_safe`).
