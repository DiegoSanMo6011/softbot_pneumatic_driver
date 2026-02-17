# Hoja rapida - Flujo KiCad (PC laboratorio)

## Objetivo
- Trabajar diseño PCB en equipo con una fuente local controlada.
- Mantener trazabilidad diaria durante el mes de experimentacion.

## 1) Encontrar proyecto KiCad oficial local
En la PC del laboratorio:
```bash
find "$HOME" -name '*.kicad_pro' 2>/dev/null
```

Seleccionar la carpeta oficial y documentarla en bitacora interna.

## 2) Convencion de snapshots diarios
- Formato:
  - `YYYYMMDD_softbot_pcb_snapshot.zip`
- Ejemplo:
  - `20260218_softbot_pcb_snapshot.zip`

Comando sugerido:
```bash
cd /ruta/al/proyecto_kicad
zip -r "20260218_softbot_pcb_snapshot.zip" .
```

## 3) Estructura de trabajo por persona
Crear carpetas de trabajo en laboratorio:
```bash
mkdir -p "$HOME/kicad_team/Mariano" "$HOME/kicad_team/Daniel" "$HOME/kicad_team/Sebastian"
```

Cada integrante guarda:
- notas de cambios,
- capturas de ERC/DRC,
- exportes intermedios.

## 4) Flujo minimo por iteracion
1. Abrir esquema y PCB.
2. Verificar librerias y `footprint table`.
3. Correr ERC.
4. Correr DRC.
5. Exportar evidencia minima:
   - PDF de esquema,
   - Gerber de prueba.
6. Registrar cambios del dia en bitacora.

## 5) Bitacora diaria recomendada
Campos minimos:
- fecha/hora,
- autor,
- cambio realizado,
- resultado ERC/DRC,
- riesgo detectado,
- proximo paso.

## 6) Reglas del equipo
- No sobrescribir sin snapshot previo.
- Toda modificacion relevante debe quedar en bitacora.
- Cerrar dia con snapshot y respaldo.
