# Hoja rapida - Fallas comunes y recuperacion

## 1) `Docker daemon: not reachable`
Sintoma:
- `./scripts/labctl doctor --profile default` marca Docker FAIL.

Accion:
```bash
sudo systemctl enable --now docker
docker info
```

Si sigue fallando:
```bash
sudo usermod -aG docker $USER
exec su -l $USER
docker info
```

## 2) No aparece `/dev/ttyUSB*` o `/dev/ttyACM*`
Sintoma:
- `ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null` no muestra puertos.

Accion:
- revisar cable USB (datos, no solo carga),
- reconectar ESP32,
- cambiar puerto fisico.

Verificacion:
```bash
dmesg | tail -n 50
```

## 3) `Permission denied` al flashear serial
Accion:
```bash
sudo usermod -aG dialout $USER
sudo usermod -aG uucp $USER 2>/dev/null || true
exec su -l $USER
```

## 4) GUI no abre
Accion:
```bash
./scripts/labctl hardware gui --foreground
echo "$XDG_SESSION_TYPE $DISPLAY"
```

Debe existir sesion grafica y `DISPLAY` no vacio.

## 5) Puerto ocupado
Sintoma:
- flash/agent falla por recurso ocupado.

Accion:
```bash
lsof /dev/ttyUSB0
./scripts/labctl stop
```

## 6) Cierre seguro obligatorio
```bash
./scripts/labctl hardware off
./scripts/labctl stop
```
