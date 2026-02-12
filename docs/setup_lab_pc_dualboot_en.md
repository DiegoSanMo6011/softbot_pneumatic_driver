# Lab PC Setup (Dual Boot) - EN

## Goal
Install Ubuntu 22.04 LTS alongside Windows while preserving the main
Windows workflows (ANSYS/FEM/mechanics).

## Recommended partition layout
- Total Linux budget: **80-120 GB**
- `root` (`/`, ext4): 55 GB
- `swap`: 8 GB (16 GB if RAM <= 16 GB)
- `home` (`/home`, ext4): remaining space

## Windows pre-checklist
1. Back up critical lab files.
2. Confirm BitLocker/institutional policy before partitioning.
3. Verify enough free disk space.
4. Create Ubuntu 22.04 LTS bootable USB.

## Installation flow
1. Shrink Windows partition using Disk Management.
2. Boot from Ubuntu USB (UEFI mode).
3. Select "Install Ubuntu" -> "Something else".
4. Create Linux partitions only in unallocated space.
5. Install GRUB to main UEFI disk.
6. Reboot and confirm both Ubuntu and Windows entries.

## Minimum post-install
1. Update system:
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
2. Clone repository in Linux.
3. Run platform installer:
```bash
./scripts/install_lab.sh --online
```
4. Validate environment:
```bash
./scripts/labctl doctor --profile default
```

## Operational notes
- Windows remains complementary for simulation/analysis.
- Robot control/runtime is standardized on Linux.
