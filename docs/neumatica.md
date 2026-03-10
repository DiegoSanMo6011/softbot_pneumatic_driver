# Neumática — conexión física y lógica de precarga

## 1. Arquitectura vigente
Sistema bidireccional con **4 bombas**:

- **Presión**: 2 bombas en paralelo.
- **Vacío**: 2 bombas en paralelo.

Cada modo tiene su propia línea upstream:

- bombas -> sensor -> válvula de modo

Ambas válvulas de modo descargan hacia el mismo **manifold**.

Desde el manifold salen tres rutas de cámara:

- Cámara A por `MUX A`
- Cámara B por `MUX B`
- Cámara C por `Valve Chamber C` (pin legacy BOOST)

## 2. Qué hace la precarga
El firmware nuevo ya no necesita tanque para “almacenar” el target.
La precarga usa el volumen de línea entre bombas/sensor y la válvula de modo:

1. selecciona el modo (`presión` o `vacío`)
2. mantiene cerrada la válvula de modo
3. regula la línea upstream hasta el target
4. cuando está lista:
   - `AUTO`: abre sola al manifold
   - `ARM`: espera un `FIRE`

Esto evita que la presión objetivo se construya con la cámara conectada desde el inicio.

## 3. Selección de cámaras
La selección sigue siendo por bitmask:

- `1` => A
- `2` => B
- `4` => C
- combinaciones: `3`, `5`, `6`, `7`

Reglas:

- `0` bloquea cámaras en comandos normales.
- En `VENT`, `0` se interpreta como `A+B+C`.
- Durante `PRECHARGE` y `READY_HOLD`, la máscara aplicada al manifold es `0`.

## 4. Estados útiles para interpretar la neumática
- `DIRECT`: la válvula de modo abre inmediatamente; es el comportamiento antiguo.
- `PRECHARGE`: la línea fuente se llena upstream del manifold.
- `READY_HOLD`: la fuente está lista pero aún no entrega.
- `DELIVER`: la fuente se conecta al manifold y a la(s) cámara(s) seleccionadas.
- `VENT`: descarga a atmósfera.
- `FAULT`: timeout o safety break.

## 5. Sensores
- `/sensor/pressure` mide la línea de presión (`ADS1115 Ch0`).
- `/sensor/vacuum` mide la línea de vacío (`ADS1115 Ch1`).

Ambos sensores son de **línea fuente**, no sensores individuales por cámara.

## 6. Referencias activas
- Diagrama vigente:
  `hardware/pneumatica/diagrama.md`
- Protocolo y estados:
  `docs/protocolo_neumatico_atomico.md`
- Arquitectura global:
  `docs/arquitectura.md`
