#!/usr/bin/env python3
"""Dual-phase pump evaluation core for pressure and vacuum selection."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import re
import statistics
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Any

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_ROOT = os.path.join(REPO_ROOT, "software")
if SOFTWARE_ROOT not in sys.path:
    sys.path.append(SOFTWARE_ROOT)

from sdk.protocol import CHAMBER_ABC, MODE_PWM_INFLATE, MODE_PWM_SUCTION  # noqa: E402

EVENT_PREFIX = "[pump_eval_event]"
PHASE_CAP_PRESSURE = "cap_pressure"
PHASE_TARGET_PRESSURE = "target_pressure"
PHASE_CAP_VACUUM = "cap_vacuum"
PHASE_TARGET_VACUUM = "target_vacuum"
PHASES = (
    PHASE_CAP_PRESSURE,
    PHASE_TARGET_PRESSURE,
    PHASE_CAP_VACUUM,
    PHASE_TARGET_VACUUM,
)


class SafetyBreakError(RuntimeError):
    """Raised when pressure exits configured safety envelope."""


@dataclass
class SamplePoint:
    run_idx: int
    phase: str
    t_session_s: float
    t_phase_s: float
    pressure_raw_kpa: float
    pressure_filtered_kpa: float
    pwm_main: int
    pwm_aux: int


@dataclass
class PhaseMetrics:
    run_idx: int
    phase: str
    start_session_s: float
    end_session_s: float
    top_kpa: float
    time_to_top_s: float
    time_to_top_session_s: float
    time_to_target_s: float | None
    time_to_target_session_s: float | None
    target_hit: bool
    overshoot_kpa: float | None
    settling_time_s: float | None


@dataclass
class RunMetrics:
    run_idx: int
    top_pressure_kpa: float
    time_to_top_pressure_s: float
    top_vacuum_kpa: float
    time_to_top_vacuum_s: float
    time_to_target_pressure_s: float | None
    time_to_target_vacuum_s: float | None
    overshoot_pressure_kpa: float
    overshoot_vacuum_kpa: float
    settling_pressure_s: float | None
    settling_vacuum_s: float | None
    stable_pressure: bool
    stable_vacuum: bool
    hit_target_pressure: bool
    hit_target_vacuum: bool
    valid: bool


@dataclass
class AggregateMetrics:
    runs: int
    chamber: int
    target_pressure_kpa: float
    target_vacuum_kpa: float
    safety_max_kpa: float
    safety_min_kpa: float
    timeout_phase_s: float
    pwm_capacity: int
    filter_window: int
    kp_pos: float
    ki_pos: float
    kp_neg: float
    ki_neg: float
    apta: bool
    top_pressure_mean_kpa: float
    top_pressure_std_kpa: float
    top_vacuum_mean_kpa: float
    top_vacuum_std_kpa: float
    time_to_top_pressure_mean_s: float
    time_to_top_vacuum_mean_s: float
    time_to_target_pressure_mean_s: float | None
    time_to_target_vacuum_mean_s: float | None
    time_to_target_pressure_eff_mean_s: float
    time_to_target_vacuum_eff_mean_s: float
    overshoot_pressure_mean_kpa: float
    overshoot_vacuum_mean_kpa: float
    settling_pressure_mean_s: float | None
    settling_vacuum_mean_s: float | None
    settling_pressure_eff_mean_s: float
    settling_vacuum_eff_mean_s: float
    cap_presion: float
    cap_vacio: float
    vel_presion: float
    vel_vacio: float
    score_base: float
    penalty_variability: float
    penalty_stability: float
    penalty: float
    score_final: float
    cv_t_target_pressure: float
    cv_t_target_vacuum: float
    cv_top_pressure: float
    cv_top_vacuum: float


class SlidingMedianFilter:
    def __init__(self, window_size: int):
        self.window_size = max(1, int(window_size))
        self.values: deque[float] = deque(maxlen=self.window_size)

    def add(self, value: float) -> float:
        self.values.append(float(value))
        return float(statistics.median(self.values))


def emit_event(event: str, **payload: Any) -> None:
    message = {
        "event": event,
        "ts": datetime.now().isoformat(timespec="milliseconds"),
        **payload,
    }
    print(f"{EVENT_PREFIX} {json.dumps(message, ensure_ascii=True)}", flush=True)


def sanitize_label(label: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9_-]+", "_", str(label).strip())
    return cleaned.strip("_") or "pump"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def safe_mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(statistics.mean(values))


def safe_std(values: list[float]) -> float:
    if len(values) <= 1:
        return 0.0
    return float(statistics.pstdev(values))


def coeff_var(values: list[float]) -> float:
    if len(values) <= 1:
        return 0.0
    mean = safe_mean(values)
    if math.isclose(mean, 0.0, abs_tol=1e-9):
        return 0.0
    return abs(safe_std(values) / mean)


def _target_crossed(value: float, target: float, phase: str) -> bool:
    if phase == PHASE_TARGET_PRESSURE:
        return value >= target
    if phase == PHASE_TARGET_VACUUM:
        return value <= target
    return False


def overshoot_kpa(values: list[float], target: float, phase: str) -> float:
    if not values:
        return 0.0
    if phase == PHASE_TARGET_PRESSURE:
        return max(0.0, max(values) - target)
    if phase == PHASE_TARGET_VACUUM:
        return max(0.0, target - min(values))
    return 0.0


def settling_time(
    t_values: list[float],
    y_values: list[float],
    *,
    target: float,
    phase: str,
    band_kpa: float,
    hold_s: float,
) -> float | None:
    if not t_values or not y_values or len(t_values) != len(y_values):
        return None
    band = max(0.0, float(band_kpa))
    hold = max(0.0, float(hold_s))

    start_idx = None
    for idx, value in enumerate(y_values):
        if _target_crossed(value, target, phase):
            start_idx = idx
            break
    if start_idx is None:
        return None

    n = len(t_values)
    for i in range(start_idx, n):
        if abs(y_values[i] - target) > band:
            continue
        t0 = t_values[i]
        j = i
        stable = True
        while j < n and (t_values[j] - t0) < hold:
            if abs(y_values[j] - target) > band:
                stable = False
                break
            j += 1
        if stable and (hold <= 0.0 or (t_values[min(j, n - 1)] - t0) >= hold):
            return t0
    return None


def month_output_dir() -> str:
    dirname = time.strftime("%Y-%m")
    path = os.path.join(REPO_ROOT, "experiments", dirname)
    os.makedirs(path, exist_ok=True)
    return path


def registry_path(path_arg: str) -> str:
    candidate = str(path_arg or "").strip() or "experiments/pump_eval_registry.csv"
    if os.path.isabs(candidate):
        return candidate
    return os.path.join(REPO_ROOT, candidate)


class DemoPump:
    """Synthetic pressure source to test GUI/core without hardware."""

    def __init__(self, pump_label: str):
        self.rng = random.Random((hash(pump_label) & 0xFFFFFFFF) ^ 0x5D1F)
        self.pressure = 0.0
        self.mode = "stop"
        self.target = 0.0
        self.pwm = 0
        self.last_t = time.monotonic()

    def _step(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self.last_t)
        self.last_t = now

        pump_quality = 0.92 + self.rng.random() * 0.16  # 0.92..1.08
        if self.mode == PHASE_CAP_PRESSURE:
            asym = max(5.0, self.target) * (1.15 * pump_quality)
            alpha = 1.9
        elif self.mode == PHASE_TARGET_PRESSURE:
            asym = max(5.0, self.target) * (1.02 * pump_quality)
            alpha = 1.6
        elif self.mode == PHASE_CAP_VACUUM:
            asym = min(-4.0, self.target) * (1.15 * pump_quality)
            alpha = 1.9
        elif self.mode == PHASE_TARGET_VACUUM:
            asym = min(-4.0, self.target) * (1.02 * pump_quality)
            alpha = 1.6
        else:
            asym = 0.0
            alpha = 2.2

        response = 1.0 - math.exp(-alpha * dt)
        self.pressure += (asym - self.pressure) * response
        noise = self.rng.gauss(0.0, 0.12)
        self.pressure += noise

    def stop(self) -> None:
        self.mode = "stop"
        self.pwm = 0

    def set_chamber(self, _chamber: int) -> None:
        return None

    def vent(self, chamber_id: int = CHAMBER_ABC, duration_s: float | None = None) -> None:
        del chamber_id
        self.mode = "stop"
        self.pressure *= 0.2
        if duration_s:
            time.sleep(max(0.0, duration_s))

    def set_pwm(self, pwm: int, mode: int) -> None:
        self.pwm = int(clamp(pwm, 0, 255))
        if mode == MODE_PWM_INFLATE:
            self.mode = PHASE_CAP_PRESSURE
            self.target = max(8.0, self.pwm / 7.0)
        elif mode == MODE_PWM_SUCTION:
            self.mode = PHASE_CAP_VACUUM
            self.target = min(-8.0, -self.pwm / 9.0)

    def inflate(self, pressure_kpa: float) -> None:
        self.mode = PHASE_TARGET_PRESSURE
        self.target = float(pressure_kpa)

    def suction(self, pressure_kpa: float) -> None:
        self.mode = PHASE_TARGET_VACUUM
        self.target = float(pressure_kpa)

    def get_state(self) -> dict[str, float | int]:
        self._step()
        return {
            "pressure": float(self.pressure),
            "pwm_main": int(self.pwm),
            "pwm_aux": int(self.pwm),
            "error": float(self.target - self.pressure),
        }

    def close(self) -> None:
        return None


class PumpEvalRunner:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.sample_period_s = max(0.001, float(args.sample_ms) / 1000.0)
        self.filter = SlidingMedianFilter(args.filter_window)
        self.t0_session = time.monotonic()

        self.samples: list[SamplePoint] = []
        self.phase_metrics: list[PhaseMetrics] = []
        self.run_metrics: list[RunMetrics] = []

        if args.demo:
            self.bot = DemoPump(args.pump_label)
            self.rclpy = None
        else:
            import rclpy  # imported lazily to keep --help and --demo lightweight

            from sdk.softbot_interface import SoftBot

            self.rclpy = rclpy
            self.rclpy.init()
            self.bot = SoftBot()

        self._apply_tuning()

    def _apply_tuning(self) -> None:
        tuning = {
            "kp_pos": float(self.args.kp_pos),
            "ki_pos": float(self.args.ki_pos),
            "kp_neg": float(self.args.kp_neg),
            "ki_neg": float(self.args.ki_neg),
            "max_safe": float(self.args.safety_max_kpa),
            "min_safe": float(self.args.safety_min_kpa),
        }

        if self.args.demo:
            emit_event("tuning_applied", demo=True, **tuning)
            return

        self.bot.update_tuning(**tuning)
        emit_event("tuning_applied", demo=False, **tuning)

    def _trigger_safety_break(
        self,
        *,
        run_idx: int,
        phase: str,
        t_phase_s: float,
        pressure_raw_kpa: float,
        pressure_filtered_kpa: float,
    ) -> None:
        t_session_s = self._session_elapsed()
        max_kpa = float(self.args.safety_max_kpa)
        min_kpa = float(self.args.safety_min_kpa)

        emit_event(
            "safety_break",
            run_idx=run_idx,
            phase=phase,
            t_session_s=t_session_s,
            t_phase_s=t_phase_s,
            pressure_raw_kpa=pressure_raw_kpa,
            pressure_filtered_kpa=pressure_filtered_kpa,
            safety_max_kpa=max_kpa,
            safety_min_kpa=min_kpa,
        )

        try:
            self.bot.stop()
            self.bot.vent(chamber_id=self.args.chamber, duration_s=self.args.vent_s)
            self.bot.stop()
        except Exception:
            pass

        raise SafetyBreakError(
            "Safety break activado: "
            f"presion={pressure_filtered_kpa:.3f} kPa (raw={pressure_raw_kpa:.3f}) "
            f"fuera de rango [{min_kpa:.3f}, {max_kpa:.3f}] kPa"
        )

    def close(self) -> None:
        try:
            self.bot.stop()
            self.bot.vent(chamber_id=self.args.chamber, duration_s=self.args.vent_s)
            self.bot.stop()
        except Exception:
            pass
        try:
            self.bot.close()
        except Exception:
            pass
        if self.rclpy is not None and self.rclpy.ok():
            try:
                self.rclpy.shutdown()
            except Exception:
                pass

    def _session_elapsed(self) -> float:
        return max(0.0, time.monotonic() - self.t0_session)

    def _emit_sample(
        self,
        run_idx: int,
        phase: str,
        t_phase_s: float,
        pressure_raw_kpa: float,
        pressure_filtered_kpa: float,
        pwm_main: int,
        pwm_aux: int,
    ) -> None:
        t_session_s = self._session_elapsed()
        sample = SamplePoint(
            run_idx=run_idx,
            phase=phase,
            t_session_s=t_session_s,
            t_phase_s=t_phase_s,
            pressure_raw_kpa=pressure_raw_kpa,
            pressure_filtered_kpa=pressure_filtered_kpa,
            pwm_main=pwm_main,
            pwm_aux=pwm_aux,
        )
        self.samples.append(sample)
        emit_event(
            "sample",
            run_idx=run_idx,
            phase=phase,
            t_session_s=t_session_s,
            t_phase_s=t_phase_s,
            pressure_raw_kpa=pressure_raw_kpa,
            pressure_filtered_kpa=pressure_filtered_kpa,
            pwm_main=pwm_main,
            pwm_aux=pwm_aux,
        )

    def _prepare_for_phase(self) -> None:
        self.bot.stop()
        self.bot.set_chamber(self.args.chamber)
        self.bot.vent(chamber_id=self.args.chamber, duration_s=self.args.vent_s)
        time.sleep(max(0.0, float(self.args.rest_s)))

    def _apply_phase_command(self, phase: str) -> None:
        if phase == PHASE_CAP_PRESSURE:
            self.bot.set_pwm(self.args.pwm_capacity, MODE_PWM_INFLATE)
            return
        if phase == PHASE_TARGET_PRESSURE:
            self.bot.inflate(self.args.target_pressure_kpa)
            return
        if phase == PHASE_CAP_VACUUM:
            self.bot.set_pwm(self.args.pwm_capacity, MODE_PWM_SUCTION)
            return
        if phase == PHASE_TARGET_VACUUM:
            self.bot.suction(self.args.target_vacuum_kpa)
            return
        raise ValueError(f"Unknown phase: {phase}")

    def _target_for_phase(self, phase: str) -> float | None:
        if phase == PHASE_TARGET_PRESSURE:
            return float(self.args.target_pressure_kpa)
        if phase == PHASE_TARGET_VACUUM:
            return float(self.args.target_vacuum_kpa)
        return None

    def _collect_phase(self, run_idx: int, phase: str) -> PhaseMetrics:
        self._prepare_for_phase()
        # Avoid cross-phase contamination in filtered metrics.
        self.filter = SlidingMedianFilter(self.args.filter_window)

        t_start_session = self._session_elapsed()
        emit_event(
            "phase_start",
            run_idx=run_idx,
            phase=phase,
            t_session_s=t_start_session,
        )

        self._apply_phase_command(phase)

        target = self._target_for_phase(phase)
        target_hit = False
        t_target_s: float | None = None
        t_target_session_s: float | None = None
        phase_t_values: list[float] = []
        phase_filtered_values: list[float] = []

        best_value = -1e9 if phase in (PHASE_CAP_PRESSURE, PHASE_TARGET_PRESSURE) else 1e9
        t_best_s = 0.0
        t_best_session_s = t_start_session

        phase_t0 = time.monotonic()
        while True:
            now = time.monotonic()
            t_phase_s = now - phase_t0
            state = self.bot.get_state()
            pressure_raw = float(state["pressure"])
            pressure_filtered = self.filter.add(pressure_raw)
            pwm_main = int(state.get("pwm_main", 0))
            pwm_aux = int(state.get("pwm_aux", 0))
            phase_t_values.append(t_phase_s)
            phase_filtered_values.append(pressure_filtered)

            self._emit_sample(
                run_idx=run_idx,
                phase=phase,
                t_phase_s=t_phase_s,
                pressure_raw_kpa=pressure_raw,
                pressure_filtered_kpa=pressure_filtered,
                pwm_main=pwm_main,
                pwm_aux=pwm_aux,
            )

            max_kpa = float(self.args.safety_max_kpa)
            min_kpa = float(self.args.safety_min_kpa)
            if (
                pressure_raw >= max_kpa
                or pressure_filtered >= max_kpa
                or pressure_raw <= min_kpa
                or pressure_filtered <= min_kpa
            ):
                self._trigger_safety_break(
                    run_idx=run_idx,
                    phase=phase,
                    t_phase_s=t_phase_s,
                    pressure_raw_kpa=pressure_raw,
                    pressure_filtered_kpa=pressure_filtered,
                )

            if phase in (PHASE_CAP_PRESSURE, PHASE_TARGET_PRESSURE):
                if pressure_filtered > best_value:
                    best_value = pressure_filtered
                    t_best_s = t_phase_s
                    t_best_session_s = self._session_elapsed()
            else:
                if pressure_filtered < best_value:
                    best_value = pressure_filtered
                    t_best_s = t_phase_s
                    t_best_session_s = self._session_elapsed()

            if not target_hit and target is not None:
                if phase == PHASE_TARGET_PRESSURE and pressure_filtered >= target:
                    target_hit = True
                elif phase == PHASE_TARGET_VACUUM and pressure_filtered <= target:
                    target_hit = True
                if target_hit:
                    t_target_s = t_phase_s
                    t_target_session_s = self._session_elapsed()
                    emit_event(
                        "target_cross",
                        run_idx=run_idx,
                        phase=phase,
                        t_session_s=t_target_session_s,
                        t_phase_s=t_target_s,
                        target_kpa=target,
                        pressure_filtered_kpa=pressure_filtered,
                    )

            if t_phase_s >= float(self.args.timeout_phase_s):
                break
            time.sleep(self.sample_period_s)

        self.bot.stop()
        self.bot.vent(chamber_id=self.args.chamber, duration_s=self.args.vent_s)
        time.sleep(max(0.0, float(self.args.rest_s)))

        phase_overshoot: float | None = None
        phase_settling: float | None = None
        if target is not None:
            phase_overshoot = overshoot_kpa(phase_filtered_values, target, phase)
            phase_settling = settling_time(
                phase_t_values,
                phase_filtered_values,
                target=target,
                phase=phase,
                band_kpa=float(self.args.settle_band_kpa),
                hold_s=float(self.args.settle_hold_s),
            )

        t_end_session = self._session_elapsed()
        metrics = PhaseMetrics(
            run_idx=run_idx,
            phase=phase,
            start_session_s=t_start_session,
            end_session_s=t_end_session,
            top_kpa=best_value,
            time_to_top_s=t_best_s,
            time_to_top_session_s=t_best_session_s,
            time_to_target_s=t_target_s,
            time_to_target_session_s=t_target_session_s,
            target_hit=target_hit,
            overshoot_kpa=phase_overshoot,
            settling_time_s=phase_settling,
        )
        self.phase_metrics.append(metrics)
        emit_event(
            "phase_end",
            **asdict(metrics),
            t_session_s=t_end_session,
        )
        return metrics

    def _run_once(self, run_idx: int) -> RunMetrics:
        emit_event("run_start", run_idx=run_idx, runs=self.args.runs)

        phase_map: dict[str, PhaseMetrics] = {}
        for phase in PHASES:
            phase_map[phase] = self._collect_phase(run_idx=run_idx, phase=phase)

        target_pressure = phase_map[PHASE_TARGET_PRESSURE]
        target_vacuum = phase_map[PHASE_TARGET_VACUUM]
        overshoot_pressure = float(target_pressure.overshoot_kpa or 0.0)
        overshoot_vacuum = float(target_vacuum.overshoot_kpa or 0.0)
        settling_pressure = target_pressure.settling_time_s
        settling_vacuum = target_vacuum.settling_time_s

        stable_pressure = bool(
            target_pressure.target_hit
            and settling_pressure is not None
            and overshoot_pressure <= float(self.args.max_overshoot_pressure_kpa)
        )
        stable_vacuum = bool(
            target_vacuum.target_hit
            and settling_vacuum is not None
            and overshoot_vacuum <= float(self.args.max_overshoot_vacuum_kpa)
        )
        targets_ok = bool(target_pressure.target_hit and target_vacuum.target_hit)
        if bool(self.args.require_stability):
            valid = bool(targets_ok and stable_pressure and stable_vacuum)
        else:
            valid = targets_ok

        run = RunMetrics(
            run_idx=run_idx,
            top_pressure_kpa=float(phase_map[PHASE_CAP_PRESSURE].top_kpa),
            time_to_top_pressure_s=float(phase_map[PHASE_CAP_PRESSURE].time_to_top_s),
            top_vacuum_kpa=float(phase_map[PHASE_CAP_VACUUM].top_kpa),
            time_to_top_vacuum_s=float(phase_map[PHASE_CAP_VACUUM].time_to_top_s),
            time_to_target_pressure_s=target_pressure.time_to_target_s,
            time_to_target_vacuum_s=target_vacuum.time_to_target_s,
            overshoot_pressure_kpa=overshoot_pressure,
            overshoot_vacuum_kpa=overshoot_vacuum,
            settling_pressure_s=settling_pressure,
            settling_vacuum_s=settling_vacuum,
            stable_pressure=stable_pressure,
            stable_vacuum=stable_vacuum,
            hit_target_pressure=bool(target_pressure.target_hit),
            hit_target_vacuum=bool(target_vacuum.target_hit),
            valid=valid,
        )
        self.run_metrics.append(run)
        emit_event("run_end", **asdict(run))
        return run

    def compute_aggregate(self) -> AggregateMetrics:
        tops_pressure = [float(row.top_pressure_kpa) for row in self.run_metrics]
        tops_vacuum = [float(row.top_vacuum_kpa) for row in self.run_metrics]
        tops_vacuum_abs = [abs(value) for value in tops_vacuum]

        ttop_pressure = [float(row.time_to_top_pressure_s) for row in self.run_metrics]
        ttop_vacuum = [float(row.time_to_top_vacuum_s) for row in self.run_metrics]

        ttarget_pressure = [
            float(row.time_to_target_pressure_s)
            for row in self.run_metrics
            if row.time_to_target_pressure_s is not None
        ]
        ttarget_vacuum = [
            float(row.time_to_target_vacuum_s)
            for row in self.run_metrics
            if row.time_to_target_vacuum_s is not None
        ]

        ttarget_pressure_eff = [
            float(row.time_to_target_pressure_s)
            if row.time_to_target_pressure_s is not None
            else float(self.args.timeout_phase_s)
            for row in self.run_metrics
        ]
        ttarget_vacuum_eff = [
            float(row.time_to_target_vacuum_s)
            if row.time_to_target_vacuum_s is not None
            else float(self.args.timeout_phase_s)
            for row in self.run_metrics
        ]
        overshoot_pressure = [float(row.overshoot_pressure_kpa) for row in self.run_metrics]
        overshoot_vacuum = [float(row.overshoot_vacuum_kpa) for row in self.run_metrics]
        settling_pressure = [
            float(row.settling_pressure_s)
            for row in self.run_metrics
            if row.settling_pressure_s is not None
        ]
        settling_vacuum = [
            float(row.settling_vacuum_s)
            for row in self.run_metrics
            if row.settling_vacuum_s is not None
        ]
        settling_pressure_eff = [
            float(row.settling_pressure_s)
            if row.settling_pressure_s is not None
            else float(self.args.timeout_phase_s)
            for row in self.run_metrics
        ]
        settling_vacuum_eff = [
            float(row.settling_vacuum_s)
            if row.settling_vacuum_s is not None
            else float(self.args.timeout_phase_s)
            for row in self.run_metrics
        ]

        apta = all(row.valid for row in self.run_metrics)

        top_pressure_mean = safe_mean(tops_pressure)
        top_pressure_std = safe_std(tops_pressure)
        top_vacuum_mean = safe_mean(tops_vacuum)
        top_vacuum_std = safe_std(tops_vacuum)
        ttop_pressure_mean = safe_mean(ttop_pressure)
        ttop_vacuum_mean = safe_mean(ttop_vacuum)

        ttarget_pressure_mean = safe_mean(ttarget_pressure) if ttarget_pressure else None
        ttarget_vacuum_mean = safe_mean(ttarget_vacuum) if ttarget_vacuum else None

        ttarget_pressure_eff_mean = safe_mean(ttarget_pressure_eff)
        ttarget_vacuum_eff_mean = safe_mean(ttarget_vacuum_eff)
        overshoot_pressure_mean = safe_mean(overshoot_pressure)
        overshoot_vacuum_mean = safe_mean(overshoot_vacuum)
        settling_pressure_mean = safe_mean(settling_pressure) if settling_pressure else None
        settling_vacuum_mean = safe_mean(settling_vacuum) if settling_vacuum else None
        settling_pressure_eff_mean = safe_mean(settling_pressure_eff)
        settling_vacuum_eff_mean = safe_mean(settling_vacuum_eff)

        cap_presion = (
            clamp(top_pressure_mean / float(self.args.target_pressure_kpa), 0.0, 1.5) / 1.5
        )
        cap_vacio = (
            clamp(abs(top_vacuum_mean) / abs(float(self.args.target_vacuum_kpa)), 0.0, 1.5) / 1.5
        )
        vel_presion = clamp(
            (float(self.args.timeout_phase_s) - ttarget_pressure_eff_mean)
            / float(self.args.timeout_phase_s),
            0.0,
            1.0,
        )
        vel_vacio = clamp(
            (float(self.args.timeout_phase_s) - ttarget_vacuum_eff_mean)
            / float(self.args.timeout_phase_s),
            0.0,
            1.0,
        )

        score_base = 0.25 * cap_presion + 0.25 * cap_vacio + 0.25 * vel_presion + 0.25 * vel_vacio

        cv_t_target_pressure = coeff_var(ttarget_pressure_eff)
        cv_t_target_vacuum = coeff_var(ttarget_vacuum_eff)
        cv_top_pressure = coeff_var(tops_pressure)
        cv_top_vacuum = coeff_var(tops_vacuum_abs)

        penalty_variability = min(
            0.30,
            0.10 * cv_t_target_pressure
            + 0.10 * cv_t_target_vacuum
            + 0.05 * cv_top_pressure
            + 0.05 * cv_top_vacuum,
        )

        overshoot_pressure_norm = clamp(
            overshoot_pressure_mean / max(1e-9, float(self.args.max_overshoot_pressure_kpa)),
            0.0,
            3.0,
        )
        overshoot_vacuum_norm = clamp(
            overshoot_vacuum_mean / max(1e-9, float(self.args.max_overshoot_vacuum_kpa)),
            0.0,
            3.0,
        )
        settling_pressure_norm = clamp(
            settling_pressure_eff_mean / float(self.args.timeout_phase_s),
            0.0,
            1.5,
        )
        settling_vacuum_norm = clamp(
            settling_vacuum_eff_mean / float(self.args.timeout_phase_s),
            0.0,
            1.5,
        )
        penalty_stability = min(
            0.40,
            0.10 * overshoot_pressure_norm
            + 0.10 * overshoot_vacuum_norm
            + 0.10 * settling_pressure_norm
            + 0.10 * settling_vacuum_norm,
        )
        penalty = min(0.60, penalty_variability + penalty_stability)
        score_final = max(0.0, score_base - penalty) * 100.0

        return AggregateMetrics(
            runs=int(self.args.runs),
            chamber=int(self.args.chamber),
            target_pressure_kpa=float(self.args.target_pressure_kpa),
            target_vacuum_kpa=float(self.args.target_vacuum_kpa),
            safety_max_kpa=float(self.args.safety_max_kpa),
            safety_min_kpa=float(self.args.safety_min_kpa),
            timeout_phase_s=float(self.args.timeout_phase_s),
            pwm_capacity=int(self.args.pwm_capacity),
            filter_window=int(self.args.filter_window),
            kp_pos=float(self.args.kp_pos),
            ki_pos=float(self.args.ki_pos),
            kp_neg=float(self.args.kp_neg),
            ki_neg=float(self.args.ki_neg),
            apta=bool(apta),
            top_pressure_mean_kpa=top_pressure_mean,
            top_pressure_std_kpa=top_pressure_std,
            top_vacuum_mean_kpa=top_vacuum_mean,
            top_vacuum_std_kpa=top_vacuum_std,
            time_to_top_pressure_mean_s=ttop_pressure_mean,
            time_to_top_vacuum_mean_s=ttop_vacuum_mean,
            time_to_target_pressure_mean_s=ttarget_pressure_mean,
            time_to_target_vacuum_mean_s=ttarget_vacuum_mean,
            time_to_target_pressure_eff_mean_s=ttarget_pressure_eff_mean,
            time_to_target_vacuum_eff_mean_s=ttarget_vacuum_eff_mean,
            overshoot_pressure_mean_kpa=overshoot_pressure_mean,
            overshoot_vacuum_mean_kpa=overshoot_vacuum_mean,
            settling_pressure_mean_s=settling_pressure_mean,
            settling_vacuum_mean_s=settling_vacuum_mean,
            settling_pressure_eff_mean_s=settling_pressure_eff_mean,
            settling_vacuum_eff_mean_s=settling_vacuum_eff_mean,
            cap_presion=cap_presion,
            cap_vacio=cap_vacio,
            vel_presion=vel_presion,
            vel_vacio=vel_vacio,
            score_base=score_base,
            penalty_variability=penalty_variability,
            penalty_stability=penalty_stability,
            penalty=penalty,
            score_final=score_final,
            cv_t_target_pressure=cv_t_target_pressure,
            cv_t_target_vacuum=cv_t_target_vacuum,
            cv_top_pressure=cv_top_pressure,
            cv_top_vacuum=cv_top_vacuum,
        )

    def run(self) -> tuple[AggregateMetrics, str, str]:
        emit_event(
            "session_start",
            pump_label=self.args.pump_label,
            chamber=self.args.chamber,
            runs=self.args.runs,
            target_pressure_kpa=self.args.target_pressure_kpa,
            target_vacuum_kpa=self.args.target_vacuum_kpa,
            safety_max_kpa=self.args.safety_max_kpa,
            safety_min_kpa=self.args.safety_min_kpa,
            pwm_capacity=self.args.pwm_capacity,
            timeout_phase_s=self.args.timeout_phase_s,
            sample_ms=self.args.sample_ms,
            vent_s=self.args.vent_s,
            rest_s=self.args.rest_s,
            filter_window=self.args.filter_window,
            settle_band_kpa=self.args.settle_band_kpa,
            settle_hold_s=self.args.settle_hold_s,
            max_overshoot_pressure_kpa=self.args.max_overshoot_pressure_kpa,
            max_overshoot_vacuum_kpa=self.args.max_overshoot_vacuum_kpa,
            require_stability=bool(self.args.require_stability),
            kp_pos=self.args.kp_pos,
            ki_pos=self.args.ki_pos,
            kp_neg=self.args.kp_neg,
            ki_neg=self.args.ki_neg,
            demo=bool(self.args.demo),
        )

        for run_idx in range(1, int(self.args.runs) + 1):
            self._run_once(run_idx)

        aggregate = self.compute_aggregate()

        out_dir = month_output_dir()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = sanitize_label(self.args.pump_label)
        safe_tag = sanitize_label(self.args.tag) if self.args.tag else ""
        suffix = f"_{safe_tag}" if safe_tag else ""
        raw_path = os.path.join(out_dir, f"pump_eval_raw_{stamp}_{safe_label}{suffix}.csv")
        summary_path = os.path.join(out_dir, f"pump_eval_summary_{stamp}_{safe_label}{suffix}.csv")

        write_raw_csv(raw_path, self.samples)
        write_summary_csv(summary_path, self.run_metrics, aggregate)

        emit_event(
            "session_end",
            pump_label=self.args.pump_label,
            apta=aggregate.apta,
            score_final=aggregate.score_final,
            score_base=aggregate.score_base,
            penalty=aggregate.penalty,
            top_pressure_mean_kpa=aggregate.top_pressure_mean_kpa,
            top_vacuum_mean_kpa=aggregate.top_vacuum_mean_kpa,
            time_to_top_pressure_mean_s=aggregate.time_to_top_pressure_mean_s,
            time_to_top_vacuum_mean_s=aggregate.time_to_top_vacuum_mean_s,
            time_to_target_pressure_mean_s=aggregate.time_to_target_pressure_mean_s,
            time_to_target_vacuum_mean_s=aggregate.time_to_target_vacuum_mean_s,
            overshoot_pressure_mean_kpa=aggregate.overshoot_pressure_mean_kpa,
            overshoot_vacuum_mean_kpa=aggregate.overshoot_vacuum_mean_kpa,
            settling_pressure_mean_s=aggregate.settling_pressure_mean_s,
            settling_vacuum_mean_s=aggregate.settling_vacuum_mean_s,
            raw_csv=os.path.relpath(raw_path, REPO_ROOT),
            summary_csv=os.path.relpath(summary_path, REPO_ROOT),
        )

        return aggregate, raw_path, summary_path


def write_raw_csv(path: str, rows: list[SamplePoint]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "run_idx",
                "phase",
                "t_session_s",
                "t_phase_s",
                "pressure_raw_kpa",
                "pressure_filtered_kpa",
                "pwm_main",
                "pwm_aux",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.run_idx,
                    row.phase,
                    f"{row.t_session_s:.6f}",
                    f"{row.t_phase_s:.6f}",
                    f"{row.pressure_raw_kpa:.6f}",
                    f"{row.pressure_filtered_kpa:.6f}",
                    row.pwm_main,
                    row.pwm_aux,
                ]
            )


def _num(value: float | None) -> str:
    if value is None:
        return ""
    return f"{float(value):.6f}"


def write_summary_csv(path: str, run_rows: list[RunMetrics], aggregate: AggregateMetrics) -> None:
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "run_idx",
                "top_pressure_kpa",
                "time_to_top_pressure_s",
                "top_vacuum_kpa",
                "time_to_top_vacuum_s",
                "time_to_target_pressure_s",
                "time_to_target_vacuum_s",
                "overshoot_pressure_kpa",
                "overshoot_vacuum_kpa",
                "settling_pressure_s",
                "settling_vacuum_s",
                "stable_pressure",
                "stable_vacuum",
                "hit_target_pressure",
                "hit_target_vacuum",
                "valid",
            ]
        )
        for row in run_rows:
            writer.writerow(
                [
                    row.run_idx,
                    _num(row.top_pressure_kpa),
                    _num(row.time_to_top_pressure_s),
                    _num(row.top_vacuum_kpa),
                    _num(row.time_to_top_vacuum_s),
                    _num(row.time_to_target_pressure_s),
                    _num(row.time_to_target_vacuum_s),
                    _num(row.overshoot_pressure_kpa),
                    _num(row.overshoot_vacuum_kpa),
                    _num(row.settling_pressure_s),
                    _num(row.settling_vacuum_s),
                    int(row.stable_pressure),
                    int(row.stable_vacuum),
                    int(row.hit_target_pressure),
                    int(row.hit_target_vacuum),
                    int(row.valid),
                ]
            )

        writer.writerow([])
        writer.writerow(["field", "value"])
        for key, value in asdict(aggregate).items():
            if isinstance(value, bool):
                writer.writerow([key, int(value)])
            elif isinstance(value, float):
                writer.writerow([key, _num(value)])
            elif value is None:
                writer.writerow([key, ""])
            else:
                writer.writerow([key, value])


def append_registry(
    path: str,
    args: argparse.Namespace,
    aggregate: AggregateMetrics,
    summary_path: str,
) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    exists = os.path.exists(path)

    with open(path, "a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if not exists:
            writer.writerow(
                [
                    "timestamp",
                    "pump_label",
                    "apta",
                    "score_final",
                    "score_base",
                    "penalty",
                    "chamber",
                    "target_pressure_kpa",
                    "target_vacuum_kpa",
                    "pwm_capacity",
                    "timeout_phase_s",
                    "runs",
                    "sample_ms",
                    "vent_s",
                    "rest_s",
                    "filter_window",
                    "settle_band_kpa",
                    "settle_hold_s",
                    "max_overshoot_pressure_kpa",
                    "max_overshoot_vacuum_kpa",
                    "require_stability",
                    "top_pressure_mean_kpa",
                    "top_pressure_std_kpa",
                    "top_vacuum_mean_kpa",
                    "top_vacuum_std_kpa",
                    "time_to_top_pressure_mean_s",
                    "time_to_top_vacuum_mean_s",
                    "time_to_target_pressure_mean_s",
                    "time_to_target_vacuum_mean_s",
                    "time_to_target_pressure_eff_mean_s",
                    "time_to_target_vacuum_eff_mean_s",
                    "overshoot_pressure_mean_kpa",
                    "overshoot_vacuum_mean_kpa",
                    "settling_pressure_mean_s",
                    "settling_vacuum_mean_s",
                    "settling_pressure_eff_mean_s",
                    "settling_vacuum_eff_mean_s",
                    "cv_t_target_pressure",
                    "cv_t_target_vacuum",
                    "cv_top_pressure",
                    "cv_top_vacuum",
                    "summary_csv",
                ]
            )

        writer.writerow(
            [
                datetime.now().isoformat(timespec="seconds"),
                args.pump_label,
                int(aggregate.apta),
                _num(aggregate.score_final),
                _num(aggregate.score_base),
                _num(aggregate.penalty),
                aggregate.chamber,
                _num(aggregate.target_pressure_kpa),
                _num(aggregate.target_vacuum_kpa),
                aggregate.pwm_capacity,
                _num(aggregate.timeout_phase_s),
                aggregate.runs,
                args.sample_ms,
                _num(args.vent_s),
                _num(args.rest_s),
                aggregate.filter_window,
                _num(args.settle_band_kpa),
                _num(args.settle_hold_s),
                _num(args.max_overshoot_pressure_kpa),
                _num(args.max_overshoot_vacuum_kpa),
                int(bool(args.require_stability)),
                _num(aggregate.top_pressure_mean_kpa),
                _num(aggregate.top_pressure_std_kpa),
                _num(aggregate.top_vacuum_mean_kpa),
                _num(aggregate.top_vacuum_std_kpa),
                _num(aggregate.time_to_top_pressure_mean_s),
                _num(aggregate.time_to_top_vacuum_mean_s),
                _num(aggregate.time_to_target_pressure_mean_s),
                _num(aggregate.time_to_target_vacuum_mean_s),
                _num(aggregate.time_to_target_pressure_eff_mean_s),
                _num(aggregate.time_to_target_vacuum_eff_mean_s),
                _num(aggregate.overshoot_pressure_mean_kpa),
                _num(aggregate.overshoot_vacuum_mean_kpa),
                _num(aggregate.settling_pressure_mean_s),
                _num(aggregate.settling_vacuum_mean_s),
                _num(aggregate.settling_pressure_eff_mean_s),
                _num(aggregate.settling_vacuum_eff_mean_s),
                _num(aggregate.cv_t_target_pressure),
                _num(aggregate.cv_t_target_vacuum),
                _num(aggregate.cv_top_pressure),
                _num(aggregate.cv_top_vacuum),
                os.path.relpath(summary_path, REPO_ROOT),
            ]
        )


def _row_float(row: dict[str, str], key: str) -> float | None:
    value = (row.get(key, "") or "").strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _same_float(a: float | None, b: float, eps: float = 1e-6) -> bool:
    return a is not None and abs(a - float(b)) <= eps


def _matches_or_unknown(
    row: dict[str, str],
    key: str,
    target: float,
    eps: float = 1e-6,
) -> bool:
    value = _row_float(row, key)
    if value is None:
        return True
    return abs(value - float(target)) <= eps


def historical_ranking(path: str, args: argparse.Namespace) -> list[dict[str, str]]:
    if not os.path.exists(path):
        return []

    rows: list[dict[str, str]] = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if (row.get("apta", "") or "0").strip() not in {"1", "true", "True"}:
                continue

            chamber_raw = (row.get("chamber", "") or "").strip()
            if chamber_raw:
                try:
                    chamber = int(chamber_raw)
                except ValueError:
                    continue
                if chamber != int(args.chamber):
                    continue

            if not _matches_or_unknown(row, "target_pressure_kpa", args.target_pressure_kpa):
                continue
            if not _matches_or_unknown(row, "target_vacuum_kpa", args.target_vacuum_kpa):
                continue
            if not _matches_or_unknown(row, "timeout_phase_s", args.timeout_phase_s):
                continue
            if not _matches_or_unknown(row, "pwm_capacity", float(args.pwm_capacity)):
                continue

            rows.append(row)

    rows.sort(
        key=lambda item: (
            _row_float(item, "score_final") is None,
            -(_row_float(item, "score_final") or 0.0),
        )
    )
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dual pump evaluator: capacity and target performance in pressure and vacuum."
    )
    parser.add_argument("--pump-label", required=True, type=str)
    parser.add_argument(
        "--chamber",
        type=int,
        default=CHAMBER_ABC,
        choices=[1, 2, 3, 4, 5, 6, 7],
        help="Bitmask de cámaras a evaluar (A=1, B=2, C=4).",
    )
    parser.add_argument("--target-pressure-kpa", type=float, default=25.0)
    parser.add_argument("--target-vacuum-kpa", type=float, default=-15.0)
    parser.add_argument("--safety-max-kpa", type=float, default=45.0)
    parser.add_argument("--safety-min-kpa", type=float, default=-45.0)
    parser.add_argument("--kp-pos", type=float, default=24.0)
    parser.add_argument("--ki-pos", type=float, default=1500.0)
    parser.add_argument("--kp-neg", type=float, default=-75.0)
    parser.add_argument("--ki-neg", type=float, default=-750.0)
    parser.add_argument("--pwm-capacity", type=int, default=220)
    parser.add_argument("--timeout-phase-s", type=float, default=3.0)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--sample-ms", type=int, default=20)
    parser.add_argument("--vent-s", type=float, default=0.6)
    parser.add_argument("--rest-s", type=float, default=0.4)
    parser.add_argument("--filter-window", type=int, default=5)
    parser.add_argument("--settle-band-kpa", type=float, default=1.0)
    parser.add_argument("--settle-hold-s", type=float, default=0.4)
    parser.add_argument("--max-overshoot-pressure-kpa", type=float, default=2.0)
    parser.add_argument("--max-overshoot-vacuum-kpa", type=float, default=2.0)
    parser.add_argument(
        "--require-stability",
        dest="require_stability",
        action="store_true",
        default=True,
    )
    parser.add_argument(
        "--no-require-stability",
        dest="require_stability",
        action="store_false",
    )
    parser.add_argument("--tag", type=str, default="")
    parser.add_argument("--registry-csv", type=str, default="experiments/pump_eval_registry.csv")
    parser.add_argument("--no-registry", action="store_true")
    parser.add_argument("--demo", action="store_true", help="Run with synthetic pressure model")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.pump_label or not str(args.pump_label).strip():
        raise SystemExit("--pump-label es obligatorio")
    if int(args.chamber) < 1 or int(args.chamber) > 7:
        raise SystemExit("--chamber debe estar en rango 1..7")
    if float(args.target_pressure_kpa) <= 0:
        raise SystemExit("--target-pressure-kpa debe ser > 0")
    if float(args.target_vacuum_kpa) >= 0:
        raise SystemExit("--target-vacuum-kpa debe ser < 0")
    if not math.isfinite(float(args.safety_max_kpa)):
        raise SystemExit("--safety-max-kpa debe ser finito")
    if not math.isfinite(float(args.safety_min_kpa)):
        raise SystemExit("--safety-min-kpa debe ser finito")
    if float(args.safety_max_kpa) <= 0:
        raise SystemExit("--safety-max-kpa debe ser > 0")
    if float(args.safety_min_kpa) >= 0:
        raise SystemExit("--safety-min-kpa debe ser < 0")
    if float(args.safety_min_kpa) >= float(args.safety_max_kpa):
        raise SystemExit("--safety-min-kpa debe ser menor que --safety-max-kpa")
    for key in ("kp_pos", "ki_pos", "kp_neg", "ki_neg"):
        value = float(getattr(args, key))
        if not math.isfinite(value):
            raise SystemExit(f"--{key.replace('_', '-')} debe ser finito")
    if int(args.pwm_capacity) < 0 or int(args.pwm_capacity) > 255:
        raise SystemExit("--pwm-capacity debe estar en rango 0..255")
    if float(args.timeout_phase_s) <= 0:
        raise SystemExit("--timeout-phase-s debe ser > 0")
    if int(args.runs) < 1:
        raise SystemExit("--runs debe ser >= 1")
    if int(args.sample_ms) < 1:
        raise SystemExit("--sample-ms debe ser >= 1")
    if int(args.filter_window) < 1:
        raise SystemExit("--filter-window debe ser >= 1")
    if float(args.settle_band_kpa) < 0:
        raise SystemExit("--settle-band-kpa debe ser >= 0")
    if float(args.settle_hold_s) < 0:
        raise SystemExit("--settle-hold-s debe ser >= 0")
    if float(args.max_overshoot_pressure_kpa) <= 0:
        raise SystemExit("--max-overshoot-pressure-kpa debe ser > 0")
    if float(args.max_overshoot_vacuum_kpa) <= 0:
        raise SystemExit("--max-overshoot-vacuum-kpa debe ser > 0")


def main() -> int:
    args = parse_args()
    validate_args(args)

    runner = PumpEvalRunner(args)
    aggregate: AggregateMetrics | None = None
    raw_path = ""
    summary_path = ""
    try:
        aggregate, raw_path, summary_path = runner.run()
        print("Pump evaluation completed")
        print(f"- APTA: {'YES' if aggregate.apta else 'NO'}")
        print(f"- Score final: {aggregate.score_final:.3f}")
        print(f"- Top pressure mean: {aggregate.top_pressure_mean_kpa:.3f} kPa")
        print(f"- Top vacuum mean: {aggregate.top_vacuum_mean_kpa:.3f} kPa")
        print(f"- Overshoot pressure mean: {aggregate.overshoot_pressure_mean_kpa:.3f} kPa")
        print(f"- Overshoot vacuum mean: {aggregate.overshoot_vacuum_mean_kpa:.3f} kPa")
        print(
            "- Settling pressure mean: "
            f"{aggregate.settling_pressure_mean_s:.3f} s"
            if aggregate.settling_pressure_mean_s is not None
            else "- Settling pressure mean: N/A"
        )
        print(
            "- Settling vacuum mean: "
            f"{aggregate.settling_vacuum_mean_s:.3f} s"
            if aggregate.settling_vacuum_mean_s is not None
            else "- Settling vacuum mean: N/A"
        )
        print(f"- Raw CSV: {raw_path}")
        print(f"- Summary CSV: {summary_path}")

        registry = registry_path(args.registry_csv)
        if not args.no_registry:
            append_registry(
                path=registry,
                args=args,
                aggregate=aggregate,
                summary_path=summary_path,
            )
            print(f"- Registry: {registry}")

            ranking = historical_ranking(registry, args)
            emit_event("ranking", rows=ranking[:20])
            if ranking:
                print("\nRanking historico (APTA only):")
                print("pump_label | score_final | top_pressure_mean_kpa | top_vacuum_mean_kpa")
                for row in ranking[:10]:
                    print(
                        f"{row.get('pump_label', 'N/A')} | "
                        f"{row.get('score_final', 'N/A')} | "
                        f"{row.get('top_pressure_mean_kpa', 'N/A')} | "
                        f"{row.get('top_vacuum_mean_kpa', 'N/A')}"
                    )
    except KeyboardInterrupt:
        emit_event("cancelled", message="Interrupted by user")
        return 130
    except SafetyBreakError as exc:
        emit_event("error", kind="safety_break", message=str(exc))
        print(f"SAFETY_BREAK: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        emit_event("error", message=str(exc))
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        runner.close()

    if aggregate is not None:
        emit_event(
            "finished",
            apta=aggregate.apta,
            score_final=aggregate.score_final,
            raw_csv=os.path.relpath(raw_path, REPO_ROOT),
            summary_csv=os.path.relpath(summary_path, REPO_ROOT),
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
