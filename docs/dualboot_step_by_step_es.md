# Dual Boot Windows + Ubuntu 22.04 (Paso a Paso)

Guía operativa para preparar una PC de laboratorio con Windows + Ubuntu 22.04
sin comprometer el uso principal de ANSYS/FEM en Windows.

## Perfil objetivo
- Ubuntu 22.04 LTS como runtime oficial de control.
- Windows como entorno complementario para análisis/simulación.
- Instalación en modo UEFI (sin legacy/CSM).

## Requisitos previos
- USB de 16 GB o mayor.
- ISO oficial de Ubuntu 22.04 LTS descargada.
- Acceso administrador en Windows y BIOS/UEFI.
- Respaldo de archivos críticos del laboratorio.

## Tamaño recomendado de partición Linux
- Mínimo funcional: `120 GB`.
- Recomendado para ROS + Docker + datos experimentales: `160-220 GB`.
- Sugerencia práctica:
- `/` (ext4): `80-120 GB`
- `/home` (ext4): resto
- `swap`: opcional si tienes RAM alta (Ubuntu usa swapfile por defecto)

## Checklist imprimible (antes de instalar)
- [ ] Backup completo de datos críticos.
- [ ] Confirmar políticas institucionales (BitLocker, inventario, seguridad).
- [ ] Verificar espacio libre real en disco.
- [ ] BitLocker suspendido temporalmente.
- [ ] Inicio rápido/hibernación desactivados en Windows.
- [ ] USB booteable creado en modo GPT + UEFI.
- [ ] BIOS en modo UEFI.

## Paso 1: preparar Windows
1. Actualiza Windows y reinicia una vez.
2. Suspende BitLocker (si está activo) antes de tocar particiones.
3. Desactiva hibernación e inicio rápido:
```bash
powercfg /h off
```
4. Abre `diskmgmt.msc`.
5. Reduce la partición `C:` con `Shrink Volume`.
6. Deja el espacio como `Unallocated` (no formatear en Windows).

## Paso 2: crear USB booteable Ubuntu
1. Usa Rufus en Windows.
2. Selecciona ISO Ubuntu 22.04.
3. Configura `Partition scheme = GPT`, `Target system = UEFI (non-CSM)`, `File system = FAT32`.
4. Crea el USB y expúlsalo de forma segura.

## Paso 3: instalar Ubuntu (dual boot)
1. Reinicia e inicia desde USB (`F12`, `F9` o `Esc`, según equipo).
2. En el instalador, si aparece `Install Ubuntu alongside Windows Boot Manager`, úsalo.
3. Si no aparece la opción anterior, usa `Something else`.
4. Si usas `Something else`, crea particiones solo dentro del espacio libre:
- Partición ext4 montada en `/`.
- Opcional partición ext4 montada en `/home`.
5. Instala el bootloader en el disco principal UEFI (ejemplo: `/dev/nvme0n1`).
6. Finaliza instalación y reinicia.

## Paso 4: validación de arranque
1. Verifica que GRUB muestre Ubuntu y Windows.
2. Inicia Ubuntu y actualiza:
```bash
sudo apt update && sudo apt full-upgrade -y
```
3. Reinicia y valida que ambos sistemas sigan arrancando.

## Paso 5: instalar plataforma SoftBot
1. Clona el repo:
```bash
git clone https://github.com/DiegoSanMo6011/softbot_pneumatic_driver.git
cd softbot_pneumatic_driver
```
2. Instala dependencias:
```bash
./scripts/install_lab.sh --online
```
3. Abre nueva terminal y valida entorno:
```bash
source /opt/ros/humble/setup.bash
./scripts/labctl doctor --profile default
./scripts/test_no_hw.sh
```

## Paso 6: validación posterior con ESP32
1. Compilar firmware:
```bash
./scripts/labctl firmware build --profile default
```
2. Flashear:
```bash
./scripts/labctl firmware flash --profile default --port /dev/ttyUSB0
```
3. Levantar agent:
```bash
./scripts/labctl agent start --profile default --port /dev/ttyUSB0 --baud 115200
```
4. Prueba rápida:
```bash
./scripts/labctl smoke --profile default
```

## Troubleshooting rápido
1. Ubuntu no detecta SSD/NVMe:
   Revisar BIOS: si está en Intel RST/RAID, migrar a AHCI con procedimiento seguro.
2. Windows no aparece en GRUB:
```bash
sudo apt install os-prober -y
sudo sed -i 's/^#\\?GRUB_DISABLE_OS_PROBER=.*/GRUB_DISABLE_OS_PROBER=false/' /etc/default/grub
sudo update-grub
```
3. `docker` sin permisos:
   Cierra sesión y vuelve a entrar después de `install_lab.sh`.
4. `doctor` marca `Serial devices: none`:
   Esperado cuando no hay ESP32 conectada.

## Cierre de instalación
- [ ] Ubuntu y Windows arrancan desde GRUB.
- [ ] `labctl doctor` sin errores críticos.
- [ ] `test_no_hw.sh` en PASS.
- [ ] Build firmware en PASS.
- [ ] Checklist archivado con fecha y responsable en bitácora del laboratorio.
