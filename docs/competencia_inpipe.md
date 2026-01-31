# RoboSoft — inPipe Locomotion (Competencia)

## 1. Objetivo
Preparar el sistema de locomoción neumática para demostrar movimiento robusto en entornos
confinados (tuberías), priorizando **salto** y **estabilidad**.

## 2. Estrategia actual
- Locomoción principal: **salto sincronizado A+B**
- Límite físico identificado: brusquedad máxima con neumática directa
- Solución propuesta: **tanque + válvula 2/2** como “turbo”

## 3. Checklist de preparación
- [ ] Validar modo salto con tanque (impulso bruto)
- [ ] Medir tiempos de respuesta con y sin turbo
- [ ] Ajustar límites de seguridad (max_safe/min_safe)
- [ ] Documentar diagrama neumático definitivo
- [ ] Loggear datos con GUI en tiempo real
