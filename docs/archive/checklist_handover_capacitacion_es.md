# Checklist de handover - Capacitacion equipo

Fecha: ______________________

Facilitador: ______________________

Participantes:
- Mariano [ ] Presente
- Daniel [ ] Presente
- Sebastian [ ] Presente

## A) Operacion Linux/ROS2
- [ ] Ejecuta `source /opt/ros/humble/setup.bash`.
- [ ] Ejecuta `./scripts/labctl doctor --profile default`.
- [ ] Identifica puerto serial (`/dev/ttyUSB*` o `/dev/ttyACM*`).
- [ ] Entiende secuencia build -> flash -> agent -> gui.

## B) Diagnostico de hardware
- [ ] Abre `./scripts/labctl hardware gui --foreground`.
- [ ] Activa y valida valvulas independientes.
- [ ] Activa y valida bombas por grupo (presion/vacio).
- [ ] Activa y valida mux_a/mux_b.
- [ ] Ejecuta cierre seguro (`hardware off`, `stop`).

## C) KiCad / PCB
- [ ] Ubica proyecto oficial local (`*.kicad_pro`).
- [ ] Verifica librerias y footprints.
- [ ] Ejecuta ERC.
- [ ] Ejecuta DRC.
- [ ] Exporta evidencia minima (PDF/Gerber).
- [ ] Registra cambios en bitacora diaria.

## D) Asignacion de trabajo
- [ ] Mariano: librerias y footprint table.
- [ ] Daniel: revision electrica y protecciones.
- [ ] Sebastian: layout/ruteo/DRC.
- [ ] Diego: integracion y validacion final en hardware.

## E) Criterio de salida
- [ ] Los 3 participantes ejecutan el flujo base con minima ayuda.
- [ ] Se definen tareas semanales con fecha compromiso.
- [ ] Se acuerda horario de sync diario (15 min).

## Firmas
Facilitador: ______________________

Mariano: ______________________

Daniel: ______________________

Sebastian: ______________________
