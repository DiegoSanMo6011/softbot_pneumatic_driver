# Locomoción — secuencias sobre el protocolo atómico

## 1. Principio operativo
La locomoción vigente trabaja con **un solo modo neumático activo por fase**.

Eso significa:

- una fase selecciona `presión` **o** `vacío`
- una fase selecciona una máscara de cámaras (`A/B/C`)
- una fase elige un comportamiento:
  - `Direct`
  - `Auto`
  - `Arm`
  - `Fire`

Con el manifold actual ya no se documenta como válido el patrón
“A infla mientras B succiona simultáneamente” dentro de una misma fase.

## 2. Script principal
Archivo:

```text
software/locomotion/x_crabs.py
```

Puntos clave:

- usa el SDK con `/pneumatic_command`
- cada fase manda un solo comando atómico
- los targets de presión/vacío siguen cerrando por tolerancia y tiempos mínimos/máximos

## 3. GUI principal de secuencias
Archivo:

```text
software/gui/locomotion_gui.py
```

Incluye:

- editor de fases con columnas de cámara, acción y `behavior`
- presets base para A/B/C
- runner con `Start / Pause / Resume / Stop / E-STOP`
- comando manual con selector de `Direct / Auto / Arm / Fire`

## 4. Comportamiento recomendado por tipo de fase
- **PID inflate / PID suction**: usar `Auto` por defecto.
- **PWM inflate / PWM suction**: usar `Direct`.
- **Vent / Stop**: siempre `Direct`.
- **Arm / Fire**: reservar para secuencias avanzadas donde se quiera cargar línea y disparar después.

## 5. Script alterno
Archivo:

```text
software/locomotion/locomocion_ab.py
```

Secuencia AB sincronizada con transición por tolerancia, ya migrada al comando atómico.

## 6. Flujo recomendado para iterar locomoción
1. `./scripts/labctl agent start --profile default --port <PORT> --baud 115200`
2. `./scripts/labctl gui locomotion --foreground`
3. Cargar un preset.
4. Ajustar tiempos, tolerancia y `behavior`.
5. Ejecutar y revisar `/pneumatic_state` y la telemetría exportada.

## 7. Documentos relacionados
- `docs/protocolo_neumatico_atomico.md`
- `docs/neumatica.md`
- `software/gui/README.md`
