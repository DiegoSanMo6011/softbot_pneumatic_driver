# Neumática — Esquema y Conexiones

## 1. Descripción actual
Sistema bidireccional con **4 bombas**:

- **Presión**: 2 bombas en paralelo.
- **Vacío**: 2 bombas en paralelo.
- Ambas líneas llegan al **header/manifold** principal.

Desde el header se enrutan tres caminos de cámara:
- Cámara A (vía `MUX A`)
- Cámara B (vía `MUX B`)
- Cámara C (vía válvula en pin legacy BOOST)

## 2. Selección de cámaras (lógica)
La selección se controla por software con `/active_chamber` (bitmask):
- `1` => A
- `2` => B
- `4` => C
- combinaciones por OR (`3`, `5`, `6`, `7`)

Estado `0` bloquea cámaras, excepto en `VENT`, donde el firmware aplica venteo a `A+B+C`.

## 3. Notas de implementación
- No se cambiaron conexiones eléctricas.
- El pin antes etiquetado como BOOST ahora habilita la **ruta neumática de Cámara C**.
- El sistema ya no depende de tanque ni de modos turbo/tank-fill.

## 4. Diagrama
Ver esquema base en:
```
hardware/pneumatica/diagrama.md
```

> Nota: si el diagrama histórico aún menciona tanque/boost, tomar `docs/arquitectura.md`
> y este documento como referencia operativa vigente.
