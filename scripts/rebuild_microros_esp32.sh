#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/rebuild_microros_esp32.sh [--max-subscriptions N] [--skip-pull]

Description:
  Rebuild micro_ros_arduino static library for ESP32 with a custom
  RMW_UXRCE_MAX_SUBSCRIPTIONS value.

Options:
  --max-subscriptions N  Set RMW_UXRCE_MAX_SUBSCRIPTIONS (default: 8)
  --skip-pull            Do not run docker pull before rebuild
  -h, --help             Show this help
EOF
}

MAX_SUBSCRIPTIONS="8"
SKIP_PULL="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --max-subscriptions)
      MAX_SUBSCRIPTIONS="${2:-}"
      shift 2
      ;;
    --skip-pull)
      SKIP_PULL="1"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if ! [[ "$MAX_SUBSCRIPTIONS" =~ ^[0-9]+$ ]] || [[ "$MAX_SUBSCRIPTIONS" -lt 1 ]]; then
  echo "Invalid --max-subscriptions value: $MAX_SUBSCRIPTIONS" >&2
  exit 1
fi

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing command: $1" >&2
    exit 1
  fi
}

require_cmd docker
require_cmd sed
require_cmd rg

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LABCTL="$REPO_ROOT/scripts/labctl"
MICROROS_DIR="$REPO_ROOT/firmware/.pio/libdeps/esp32dev/micro_ros_arduino"
META_FILE="$MICROROS_DIR/extras/library_generation/colcon.meta"
CONFIG_FILE="$MICROROS_DIR/src/rmw_microxrcedds_c/config.h"
LIB_FILE="$MICROROS_DIR/src/esp32/libmicroros.a"

if [[ ! -d "$MICROROS_DIR" ]]; then
  echo "micro_ros_arduino not found in .pio cache. Running initial firmware build..."
  "$LABCTL" firmware build --profile default
fi

if [[ ! -f "$META_FILE" ]]; then
  echo "Missing meta file: $META_FILE" >&2
  exit 1
fi

if ! rg -q -- "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=[0-9]+" "$META_FILE"; then
  echo "Could not find RMW_UXRCE_MAX_SUBSCRIPTIONS in $META_FILE" >&2
  exit 1
fi

echo "Setting RMW_UXRCE_MAX_SUBSCRIPTIONS=$MAX_SUBSCRIPTIONS in colcon.meta..."
sed -E -i \
  "s/-DRMW_UXRCE_MAX_SUBSCRIPTIONS=[0-9]+/-DRMW_UXRCE_MAX_SUBSCRIPTIONS=$MAX_SUBSCRIPTIONS/" \
  "$META_FILE"

pushd "$MICROROS_DIR" >/dev/null
if [[ "$SKIP_PULL" == "0" ]]; then
  docker pull microros/micro_ros_static_library_builder:humble
fi

docker run --rm \
  -v "$PWD:/project" \
  --env MICROROS_LIBRARY_FOLDER=extras \
  microros/micro_ros_static_library_builder:humble \
  -p esp32
popd >/dev/null

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Missing generated config: $CONFIG_FILE" >&2
  exit 1
fi

CURRENT_SUBSCRIPTIONS="$(
  rg --no-filename -o "RMW_UXRCE_MAX_SUBSCRIPTIONS [0-9]+" "$CONFIG_FILE" \
    | awk '{print $2}' \
    | head -n 1
)"

if [[ "$CURRENT_SUBSCRIPTIONS" != "$MAX_SUBSCRIPTIONS" ]]; then
  echo "Rebuild finished but config mismatch: expected=$MAX_SUBSCRIPTIONS got=${CURRENT_SUBSCRIPTIONS:-none}" >&2
  exit 1
fi

echo "micro-ROS rebuild OK: RMW_UXRCE_MAX_SUBSCRIPTIONS=$CURRENT_SUBSCRIPTIONS"

if [[ -f "$LIB_FILE" ]]; then
  OWNER="$(stat -c '%U' "$LIB_FILE" 2>/dev/null || true)"
  if [[ -n "$OWNER" && "$OWNER" != "$USER" ]]; then
    echo
    echo "Note: generated files are owned by '$OWNER'."
    echo "If future builds fail by permissions, run:"
    echo "  sudo chown -R $USER:$USER \"$MICROROS_DIR\""
  fi
fi

cat <<'EOF'

Next steps:
  ./scripts/labctl firmware build --profile default
  PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)
  ./scripts/labctl firmware flash --profile default --port "$PORT"
  ./scripts/labctl agent start --profile default --port "$PORT" --baud 115200
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=0
  ./scripts/labctl hardware verify --timeout-s 12 --sample-timeout-s 5
EOF
