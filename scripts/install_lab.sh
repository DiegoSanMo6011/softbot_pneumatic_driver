#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REQ_FILE="$SCRIPT_DIR/requirements_lab.txt"

MODE="online"
OFFLINE_BUNDLE=""
SUDO_CMD=""

usage() {
  cat <<USAGE
Usage:
  ./scripts/install_lab.sh --online
  ./scripts/install_lab.sh --offline <bundle_dir_or_tar.gz>

Options:
  --online              Install using internet (default).
  --offline <bundle>    Install from a previously generated offline bundle.
  -h, --help            Show this help message.
USAGE
}

log() {
  printf '[install_lab] %s\n' "$1"
}

fail() {
  printf '[install_lab] ERROR: %s\n' "$1" >&2
  exit 1
}

require_cmd() {
  local cmd="$1"
  command -v "$cmd" >/dev/null 2>&1 || fail "Missing command: $cmd"
}

apt_pkg_available() {
  local pkg="$1"
  apt-cache show "$pkg" >/dev/null 2>&1
}

run_apt() {
  if [[ -n "$SUDO_CMD" ]]; then
    $SUDO_CMD apt-get "$@"
  else
    apt-get "$@"
  fi
}

set_sudo_cmd() {
  if [[ "$EUID" -ne 0 ]]; then
    if command -v sudo >/dev/null 2>&1; then
      SUDO_CMD="sudo"
    else
      fail "sudo is required when running as non-root user"
    fi
  fi
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --online)
        MODE="online"
        shift
        ;;
      --offline)
        MODE="offline"
        OFFLINE_BUNDLE="${2:-}"
        [[ -n "$OFFLINE_BUNDLE" ]] || fail "--offline requires a bundle path"
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        fail "Unknown argument: $1"
        ;;
    esac
  done
}

check_ubuntu() {
  if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${ID:-}" != "ubuntu" ]]; then
      log "Warning: this installer is optimized for Ubuntu 22.04; detected ID=${ID:-unknown}"
      return
    fi
    if [[ "${VERSION_ID:-}" != "22.04" ]]; then
      log "Warning: target is Ubuntu 22.04 LTS; detected VERSION_ID=${VERSION_ID:-unknown}"
    fi
  fi
}

install_base_packages_online() {
  log "Installing base system packages"
  run_apt update
  run_apt install -y \
    build-essential \
    ca-certificates \
    curl \
    git \
    gnupg \
    jq \
    lsb-release \
    python3 \
    python3-pip \
    python3-venv \
    software-properties-common
}

install_docker_online() {
  if command -v docker >/dev/null 2>&1; then
    log "Docker already installed"
  else
    log "Installing Docker from Ubuntu repository"
    run_apt install -y docker.io
  fi

  if docker compose version >/dev/null 2>&1; then
    log "Docker Compose plugin already available"
  else
    if apt_pkg_available docker-compose-plugin; then
      log "Installing docker-compose-plugin"
      run_apt install -y docker-compose-plugin
    elif apt_pkg_available docker-compose-v2; then
      log "Installing docker-compose-v2"
      run_apt install -y docker-compose-v2
    elif apt_pkg_available docker-compose; then
      log "Installing legacy docker-compose"
      run_apt install -y docker-compose
    else
      log "Warning: no docker compose package found in current apt repos"
    fi
  fi

  local target_user="${SUDO_USER:-$USER}"
  if getent group docker >/dev/null 2>&1; then
    if id -nG "$target_user" | grep -qw docker; then
      log "User $target_user already in docker group"
    else
      log "Adding $target_user to docker group"
      if [[ -n "$SUDO_CMD" ]]; then
        $SUDO_CMD usermod -aG docker "$target_user"
      else
        usermod -aG docker "$target_user"
      fi
      log "Re-login required for docker group changes"
    fi
  fi
}

install_ros2_humble_online() {
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    log "ROS 2 Humble already installed"
    return
  fi

  log "Installing ROS 2 Humble (ros-base)"
  if [[ -n "$SUDO_CMD" ]]; then
    $SUDO_CMD add-apt-repository -y universe
    $SUDO_CMD curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
      $SUDO_CMD gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
      $SUDO_CMD tee /etc/apt/sources.list.d/ros2.list >/dev/null
  else
    add-apt-repository -y universe
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
      gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list
  fi

  run_apt update
  run_apt install -y ros-humble-ros-base python3-colcon-common-extensions
}

resolve_offline_bundle() {
  local bundle_path="$1"
  if [[ -d "$bundle_path" ]]; then
    printf '%s\n' "$bundle_path"
    return
  fi

  if [[ -f "$bundle_path" ]]; then
    local tmp_dir
    tmp_dir="$(mktemp -d /tmp/softbot_bundle.XXXXXX)"
    tar -xzf "$bundle_path" -C "$tmp_dir"
    local inner
    inner="$(find "$tmp_dir" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
    [[ -n "$inner" ]] || fail "Offline bundle archive is empty"
    printf '%s\n' "$inner"
    return
  fi

  fail "Offline bundle not found: $bundle_path"
}

install_from_offline_bundle() {
  local bundle_dir="$1"
  [[ -d "$bundle_dir" ]] || fail "Invalid offline bundle directory"

  log "Offline mode: validating prerequisites"
  require_cmd python3
  require_cmd git
  require_cmd docker

  if [[ -f "$bundle_dir/docker/micro-ros-agent_humble.tar" ]]; then
    log "Loading micro-ROS agent Docker image from offline bundle"
    docker load -i "$bundle_dir/docker/micro-ros-agent_humble.tar"
  else
    log "Warning: docker image tar not found in bundle"
  fi

  if [[ ! -d "$bundle_dir/wheels" ]]; then
    fail "Offline bundle missing wheels/ directory"
  fi

  setup_python_env "$bundle_dir/wheels"
}

setup_python_env() {
  local wheels_dir="${1:-}"

  log "Preparing Python virtual environment (.venv)"
  python3 -m venv "$REPO_ROOT/.venv"
  # shellcheck disable=SC1091
  source "$REPO_ROOT/.venv/bin/activate"

  if [[ -n "$wheels_dir" ]]; then
    python -m pip install --upgrade pip wheel
    python -m pip install --no-index --find-links "$wheels_dir" -r "$REQ_FILE"
  else
    python -m pip install --upgrade pip wheel
    python -m pip install -r "$REQ_FILE"
  fi

  deactivate
}

post_install_checks() {
  log "Running post-install checks"
  if [[ -x "$REPO_ROOT/scripts/labctl" ]]; then
    "$REPO_ROOT/scripts/labctl" doctor --profile default || true
  else
    log "labctl not found yet; skipping doctor"
  fi
}

main() {
  parse_args "$@"
  set_sudo_cmd
  check_ubuntu

  if [[ "$MODE" == "online" ]]; then
    install_base_packages_online
    install_docker_online
    install_ros2_humble_online
    setup_python_env
  else
    local bundle_dir
    bundle_dir="$(resolve_offline_bundle "$OFFLINE_BUNDLE")"
    install_from_offline_bundle "$bundle_dir"
  fi

  post_install_checks

  cat <<DONE

[install_lab] Completed.
Next steps:
  1) Open a new shell (required if docker group changed).
  2) source /opt/ros/humble/setup.bash
  3) ./scripts/labctl doctor --profile default
DONE
}

main "$@"
