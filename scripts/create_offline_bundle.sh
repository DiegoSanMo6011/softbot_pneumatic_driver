#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REQ_FILE="$SCRIPT_DIR/requirements_lab.txt"

OUTPUT_DIR=""
SKIP_DOCKER="false"

usage() {
  cat <<USAGE
Usage:
  ./scripts/create_offline_bundle.sh [--output <dir>] [--skip-docker]

Creates an offline bundle containing:
  - Python wheels for platform tools
  - Optional micro-ROS agent Docker image tar
  - Platform profiles and metadata
USAGE
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --output)
        OUTPUT_DIR="${2:-}"
        [[ -n "$OUTPUT_DIR" ]] || { echo "Missing value for --output" >&2; exit 1; }
        shift 2
        ;;
      --skip-docker)
        SKIP_DOCKER="true"
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown argument: $1" >&2
        exit 1
        ;;
    esac
  done
}

require_cmd() {
  local cmd="$1"
  command -v "$cmd" >/dev/null 2>&1 || { echo "Missing command: $cmd" >&2; exit 1; }
}

main() {
  parse_args "$@"

  require_cmd python3
  require_cmd tar
  require_cmd date

  if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="$REPO_ROOT/offline_bundle_$(date +%Y%m%d_%H%M%S)"
  fi

  mkdir -p "$OUTPUT_DIR/wheels" "$OUTPUT_DIR/docker" "$OUTPUT_DIR/profiles"

  echo "[offline_bundle] Downloading Python wheels"
  python3 -m pip download -r "$REQ_FILE" -d "$OUTPUT_DIR/wheels"

  if [[ "$SKIP_DOCKER" == "false" ]]; then
    require_cmd docker
    echo "[offline_bundle] Pulling micro-ROS agent Docker image"
    docker pull microros/micro-ros-agent:humble
    echo "[offline_bundle] Saving Docker image tar"
    docker save -o "$OUTPUT_DIR/docker/micro-ros-agent_humble.tar" microros/micro-ros-agent:humble
  fi

  cp "$REPO_ROOT/config/profiles"/*.json "$OUTPUT_DIR/profiles/"
  cp "$SCRIPT_DIR/install_lab.sh" "$OUTPUT_DIR/"
  cp "$REQ_FILE" "$OUTPUT_DIR/"

  {
    echo "created_at=$(date --iso-8601=seconds)"
    echo "host=$(uname -a)"
    echo "python=$(python3 --version 2>&1)"
  } > "$OUTPUT_DIR/manifest.txt"

  local tarball
  tarball="${OUTPUT_DIR}.tar.gz"
  echo "[offline_bundle] Creating archive: $tarball"
  tar -czf "$tarball" -C "$(dirname "$OUTPUT_DIR")" "$(basename "$OUTPUT_DIR")"

  echo "[offline_bundle] Done"
  echo "Bundle directory: $OUTPUT_DIR"
  echo "Bundle archive:   $tarball"
}

main "$@"
