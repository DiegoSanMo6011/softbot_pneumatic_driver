# Plan mensual de experimentacion (18 feb 2026 - 18 mar 2026)

## Objetivo general
- Dejar la plataforma estable y transferida al equipo para mejorar robot y circuito
  sin dependencia operativa en una sola persona.

## Metas de cierre
- Al menos 2 integrantes adicionales operan la plataforma de forma autonoma.
- Existe backlog priorizado de mejoras PCB con estimacion y riesgos.
- Se valida pipeline completo Linux + firmware + diagnostico hardware.

## Semana 1 (18-24 feb) - Foco electronica y baseline
- Consolidar baseline PCB actual del laboratorio.
- Ejecutar sesion de arranque y checklist de handover.
- Crear `PCB Improvement Backlog v1`:
  - lista priorizada,
  - impacto esperado,
  - esfuerzo estimado,
  - riesgo tecnico.
- Sync diario de 15 minutos para bloqueos.

## Semana 2 (25 feb-3 mar) - Cambios de alto impacto
- Implementar cambios electricos prioritarios.
- Verificar consistencia de librerias/footprints.
- Correr ERC/DRC por cada iteracion.
- Documentar decisiones tecnicas.

## Semana 3 (4-10 mar) - Prototipo y pruebas de banco
- Validar compatibilidad con firmware/diagnostico.
- Ejecutar pruebas fisicas por componente.
- Registrar evidencia:
  - fotos y video,
  - logs operativos,
  - resultados de verificacion.

## Semana 4 (11-18 mar) - Cierre y transferencia
- Demo interna final:
  - arranque completo,
  - diagnostico de actuadores,
  - presentacion de estado PCB.
- Cerrar handover:
  - checklist firmado,
  - responsabilidades claras,
  - siguientes pasos.

## Riesgos y mitigacion
- Riesgo: Docker/serial no listo el dia de prueba.
  - Mitigacion: pre-check tecnico previo y plan B de demo.
- Riesgo: cambios PCB sin trazabilidad.
  - Mitigacion: snapshot diario + bitacora obligatoria.
- Riesgo: cuello de botella en una persona.
  - Mitigacion: practica guiada individual y validacion de autonomia.
