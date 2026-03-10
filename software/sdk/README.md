# SoftBot Python SDK

## Archivo principal
```text
software/sdk/softbot_interface.py
```

## Capacidades
- Publicar comandos atómicos por `/pneumatic_command`
- Leer estado de alto nivel por `/pneumatic_state`
- Mantener wrappers compatibles (`set_chamber`, `inflate`, `suction`, `set_pwm`, `vent`, `stop`)
- Ajustar tuning PI
- Ejecutar diagnóstico hardware por bitmask

## Métodos nuevos recomendados
- `send_pneumatic_command(mode, chamber_mask, target, behavior="auto", token=0)`
- `direct_command(...)`
- `arm_inflate(...)`
- `arm_suction(...)`
- `fire(...)`

## Métodos legacy conservados
- `set_chamber(...)`
- `inflate(...)`
- `suction(...)`
- `set_pwm(...)`
- `vent(...)`
- `stop()`

Los wrappers legacy ya publican internamente el comando atómico.

## `get_state()`
Expone, además del debug clásico:

- `sensor_pressure_kpa`
- `sensor_vacuum_kpa`
- `control_pressure_kpa`
- `source_pressure_kpa`
- `pneumatic_state`
- `pneumatic_state_label`
- `pneumatic_behavior`
- `pneumatic_behavior_label`
- `requested_chamber_mask`
- `applied_chamber_mask`
- `pneumatic_flags`
- `pneumatic_target`
- `command_token`

## Referencia del contrato
```text
docs/protocolo_neumatico_atomico.md
```
