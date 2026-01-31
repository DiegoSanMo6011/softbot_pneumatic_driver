# Neumática — Esquema y Conexiones

## 1. Descripción actual (segun la ultima confirmacion)
Sistema bidireccional con **4 bombas**:

- **Presion**: 2 bombas en paralelo.
- **Vacio**: 2 bombas en paralelo.
- Ambas lineas entran al **HEADER / manifold**.
- **Derivaciones del header**:
  - **BOOST**: tanque con dos lineas (carga y descarga) con check valves.
  - **SENSOR**: transductor de presion.
  - **TEE_AT** y **TEE_AD**: valvulas 3/2 NC para inflar/ventear camara trasera (AT) y delantera (AD).
  - **R -> atmosfera** (desfogue).

## 2. Notas confirmadas
- Todas las valvulas son **NC por defecto**.
- Se usan las mismas **3/2**; cuando se requiere 2/2 se coloca **tapon** para anular un puerto.
- **Check valves**: por ahora **solo en el tanque**.
- Tanques disponibles:
  - **Plastico**: 2 entradas + 2 salidas, sin etiqueta.
  - **Metal**: 2 puertos (carga/descarga), marcado **1.25 MPa** (se usara este).

## 3. Tanque (detalle confirmado)
El tanque tiene **dos puertos**:
- **Carga**: permite flujo del sistema hacia el tanque.
- **Descarga**: permite flujo del tanque hacia el sistema.
Cada puerto tiene **check valve** orientada segun el sentido de flujo.

## 4. Terminologia
**Manifold / header / interseccion** = el bloque o union donde confluyen las lineas y desde el cual salen las derivaciones a sensor, boost y camaras.

## 5. Diagrama actualizado
Ver el esquema en:
```
hardware/pneumatica/diagrama.md
```

## 6. Detalles pendientes para cerrar el diagrama
Por favor confirmar:
1. **Posicion de reposo exacta** en las 3/2: **A -> R abierto** y **P cerrado**.
2. **Tanque metal**: volumen aprox **0.3 L**.
3. **Sensor**: conectado **directo** al manifold.
4. **Check valves**: solo en el tanque (carga/descarga).

## 7. Estado final
Con esta informacion el diagrama queda completo y consistente con la topologia actual.
