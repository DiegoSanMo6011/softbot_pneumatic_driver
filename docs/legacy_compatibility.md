# Legacy Compatibility and Deprecation Notes

## Compatibility contract (v1.0)
- SDK public API in `software/sdk/softbot_interface.py` remains stable.
- ROS topics and mode semantics remain stable as documented in `docs/arquitectura.md`.
- Existing scripts in `software/ejemplos/` are preserved for backward compatibility.

## Current legacy status
- `software/ejemplos/04_demo_completa.py`: supported, fixed for current `set_pwm` signature.
- `software/ejemplos/02_Identificacion_Sistema.py`: supported with PID kPa setpoints.
- Other scripts under `software/ejemplos/`: supported as legacy operational references.

## Deprecation policy
- Any future deprecation must include:
  1. Changelog entry.
  2. Replacement path in docs.
  3. At least one minor release overlap where both paths work.
