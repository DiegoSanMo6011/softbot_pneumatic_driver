#!/usr/bin/env bash
set -u -o pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPS_DIR="$REPO_ROOT/experiments/logs/ops"

WITH_HARDWARE="false"
PORT="/dev/ttyUSB0"
BAUD="115200"
SKIP_FLASH="false"

usage() {
  cat <<USAGE
Usage:
  ./scripts/tester_report.sh [--with-hardware] [--port /dev/ttyUSB0] [--baud 115200] [--skip-flash]

Modes:
  default          Software-only validation + doctor
  --with-hardware  Adds firmware/agent/smoke/component checks
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-hardware)
      WITH_HARDWARE="true"
      shift
      ;;
    --port)
      PORT="${2:-}"
      [[ -n "$PORT" ]] || { echo "Missing value for --port" >&2; exit 2; }
      shift 2
      ;;
    --baud)
      BAUD="${2:-}"
      [[ -n "$BAUD" ]] || { echo "Missing value for --baud" >&2; exit 2; }
      shift 2
      ;;
    --skip-flash)
      SKIP_FLASH="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

mkdir -p "$OPS_DIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OPS_DIR/tester_run_${STAMP}"
REPORT="$OPS_DIR/tester_report_${STAMP}.md"
mkdir -p "$RUN_DIR"

PASS_COUNT=0
FAIL_COUNT=0
STEP_INDEX=0

step() {
  local title="$1"
  local command="$2"
  STEP_INDEX=$((STEP_INDEX + 1))

  local slug
  slug="$(echo "$title" | tr '[:upper:]' '[:lower:]' | tr ' /' '__' | tr -cd 'a-z0-9_')"
  local log_file="$RUN_DIR/$(printf '%02d' "$STEP_INDEX")_${slug}.log"

  echo "[tester] Step $STEP_INDEX: $title"
  bash -lc "$command" >"$log_file" 2>&1
  local rc=$?

  if [[ $rc -eq 0 ]]; then
    PASS_COUNT=$((PASS_COUNT + 1))
    status="PASS"
  else
    FAIL_COUNT=$((FAIL_COUNT + 1))
    status="FAIL"
  fi

  {
    echo "## Step $STEP_INDEX - $title"
    echo "- Command: \`$command\`"
    echo "- Status: **$status** (exit=$rc)"
    echo "- Log: \`${log_file#$REPO_ROOT/}\`"
    echo
  } >> "$REPORT"
}

{
  echo "# SoftBot Tester Report"
  echo
  echo "- Timestamp: $(date --iso-8601=seconds)"
  echo "- Repo: $REPO_ROOT"
  echo "- Head: $(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo N/A)"
  echo "- Host: $(uname -a)"
  echo "- Mode: $([[ "$WITH_HARDWARE" == "true" ]] && echo 'with-hardware' || echo 'software-only')"
  echo "- Port/Baud: $PORT / $BAUD"
  echo
} > "$REPORT"

step "Repo status" "cd '$REPO_ROOT' && git status -sb"
step "No-HW suite" "cd '$REPO_ROOT' && ./scripts/test_no_hw.sh"
step "Doctor" "cd '$REPO_ROOT' && ./scripts/labctl doctor --profile default"

if [[ "$WITH_HARDWARE" == "true" ]]; then
  step "Firmware build" "cd '$REPO_ROOT' && ./scripts/labctl firmware build --profile default"

  if [[ "$SKIP_FLASH" == "false" ]]; then
    step "Firmware flash" "cd '$REPO_ROOT' && ./scripts/labctl firmware flash --profile default --port '$PORT'"
  fi

  step "Agent start" "cd '$REPO_ROOT' && ./scripts/labctl agent start --profile default --port '$PORT' --baud '$BAUD'"
  step "Smoke test" "cd '$REPO_ROOT' && ./scripts/labctl smoke --profile default"

  step "HW valve_inflate" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component valve_inflate --duration-s 0.8"
  step "HW valve_suction" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component valve_suction --duration-s 0.8"
  step "HW valve_boost" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component valve_boost --duration-s 0.8"
  step "HW mux_a" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component mux_a --duration-s 0.8"
  step "HW mux_b" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component mux_b --duration-s 0.8"
  step "HW inflate_main" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component inflate_main --pwm 120 --duration-s 1.0"
  step "HW inflate_aux" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component inflate_aux --pwm 120 --duration-s 1.0"
  step "HW suction_main" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component suction_main --pwm 120 --duration-s 1.0"
  step "HW suction_aux" "cd '$REPO_ROOT' && ./scripts/labctl hardware test --component suction_aux --pwm 120 --duration-s 1.0"

  step "HW off" "cd '$REPO_ROOT' && ./scripts/labctl hardware off"
  step "Stop services" "cd '$REPO_ROOT' && ./scripts/labctl stop"
fi

{
  echo "## Summary"
  echo "- PASS: $PASS_COUNT"
  echo "- FAIL: $FAIL_COUNT"
  echo
  if [[ $FAIL_COUNT -eq 0 ]]; then
    echo "Overall: ✅ PASS"
  else
    echo "Overall: ❌ FAIL"
  fi
} >> "$REPORT"

echo "[tester] Report generated: ${REPORT#$REPO_ROOT/}"
echo "[tester] Logs directory: ${RUN_DIR#$REPO_ROOT/}"

if [[ $FAIL_COUNT -eq 0 ]]; then
  exit 0
fi
exit 1
