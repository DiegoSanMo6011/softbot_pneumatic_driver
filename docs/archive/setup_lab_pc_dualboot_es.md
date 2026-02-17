# Setup PC laboratorio (Dual Boot) - ES

## Objetivo
Instalar Ubuntu 22.04 LTS junto a Windows sin comprometer el uso principal de
ANSYS/FEM en Windows.

## Guía detallada
Para instalación completa con checklist de ejecución y troubleshooting:
- `docs/dualboot_step_by_step_es.md`

## Recomendacion de particionado
- Espacio total Linux recomendado: **80-120 GB**
- `root` (`/`, ext4): 55 GB
- `swap`: 8 GB (16 GB si RAM <= 16 GB)
- `home` (`/home`, ext4): resto

## Pre-checklist en Windows
1. Respaldar archivos criticos del laboratorio.
2. Confirmar BitLocker y politicas institucionales antes de tocar particiones.
3. Verificar espacio libre suficiente en disco.
4. Crear USB booteable con Ubuntu 22.04 LTS.

## Flujo de instalacion
1. En Windows, reducir particion principal con "Administracion de discos".
2. Boot desde USB Ubuntu (UEFI).
3. Seleccionar "Install Ubuntu" -> "Something else".
4. Crear particiones Linux solo dentro del espacio no asignado.
5. Instalar GRUB en el disco principal UEFI.
6. Reiniciar y validar que GRUB ofrezca Ubuntu y Windows.

## Post-instalacion minima
1. Actualizar sistema:
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
2. Clonar repositorio en Linux.
3. Ejecutar instalador de plataforma:
```bash
./scripts/install_lab.sh --online
```
4. Validar con:
```bash
./scripts/labctl doctor --profile default
```

## Notas operativas
- Windows queda como sistema complementario para simulacion/analisis.
- El control del robot y operacion de campo se estandarizan en Linux.
