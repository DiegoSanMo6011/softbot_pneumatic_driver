#!/usr/bin/env python3
"""Compare baseline vs candidate controller runs with strict acceptance criteria."""

from __future__ import annotations

import argparse
import csv
import os
import time
from dataclasses import dataclass
from datetime import datetime

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


@dataclass
class EvalMetrics:
    label: str
    target_pressure_kpa: float
    target_vacuum_kpa: float
    time_target_pressure_mean_s: float | None
    time_target_vacuum_mean_s: float | None
    overshoot_pressure_mean_kpa: float | None
    overshoot_vacuum_mean_kpa: float | None
    settling_pressure_mean_s: float | None
    settling_vacuum_mean_s: float | None


def parse_float(value: str | None) -> float | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def safe_mean(values: list[float]) -> float | None:
    if not values:
        return None
    return float(sum(values) / len(values))


def parse_summary(path: str) -> tuple[list[dict[str, str]], dict[str, str]]:
    run_rows: list[dict[str, str]] = []
    fields: dict[str, str] = {}
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))

    header: list[str] | None = None
    mode = "runs"
    for row in rows:
        if not row or all(not str(item).strip() for item in row):
            continue
        if row[:2] == ["field", "value"]:
            mode = "fields"
            continue
        if mode == "runs":
            if row[0].strip() == "run_idx":
                header = [str(item).strip() for item in row]
                continue
            if header is not None and row[0].strip().isdigit():
                data = {
                    key: (row[idx].strip() if idx < len(row) else "")
                    for idx, key in enumerate(header)
                }
                run_rows.append(data)
        else:
            if len(row) >= 2:
                fields[row[0].strip()] = row[1].strip()
    return run_rows, fields


def infer_raw_from_summary(summary_path: str) -> str | None:
    name = os.path.basename(summary_path)
    if not name.startswith("pump_eval_summary_"):
        return None
    raw_name = name.replace("pump_eval_summary_", "pump_eval_raw_", 1)
    raw_path = os.path.join(os.path.dirname(summary_path), raw_name)
    return raw_path if os.path.exists(raw_path) else None


def phase_metrics(
    t_values: list[float],
    y_values: list[float],
    *,
    target: float,
    phase: str,
    band_kpa: float,
    hold_s: float,
) -> tuple[float | None, float, float | None]:
    if not t_values or not y_values:
        return None, 0.0, None

    crossing_idx: int | None = None
    for idx, value in enumerate(y_values):
        if phase == "target_pressure" and value >= target:
            crossing_idx = idx
            break
        if phase == "target_vacuum" and value <= target:
            crossing_idx = idx
            break

    t_target = t_values[crossing_idx] if crossing_idx is not None else None

    if phase == "target_pressure":
        overshoot = max(0.0, max(y_values) - target)
    else:
        overshoot = max(0.0, target - min(y_values))

    settling: float | None = None
    if crossing_idx is not None:
        for i in range(crossing_idx, len(t_values)):
            if abs(y_values[i] - target) > band_kpa:
                continue
            t0 = t_values[i]
            j = i
            stable = True
            while j < len(t_values) and (t_values[j] - t0) < hold_s:
                if abs(y_values[j] - target) > band_kpa:
                    stable = False
                    break
                j += 1
            if stable and (hold_s <= 0.0 or (t_values[min(j, len(t_values) - 1)] - t0) >= hold_s):
                settling = t0
                break

    return t_target, float(overshoot), settling


def metrics_from_raw(
    raw_path: str,
    *,
    target_pressure_kpa: float,
    target_vacuum_kpa: float,
    settle_band_kpa: float,
    settle_hold_s: float,
) -> EvalMetrics:
    by_phase: dict[tuple[int, str], list[tuple[float, float]]] = {}
    with open(raw_path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                run_idx = int(float(str(row.get("run_idx", "0")).strip()))
                phase = str(row.get("phase", "")).strip()
                t_phase_s = float(str(row.get("t_phase_s", "0")).strip())
                value = float(
                    str(row.get("pressure_filtered_kpa", row.get("pressure_raw_kpa", "0"))).strip()
                )
            except ValueError:
                continue
            key = (run_idx, phase)
            by_phase.setdefault(key, []).append((t_phase_s, value))

    tp: list[float] = []
    tv: list[float] = []
    op: list[float] = []
    ov: list[float] = []
    sp: list[float] = []
    sv: list[float] = []

    runs = sorted({key[0] for key in by_phase})
    for run_idx in runs:
        pressure_data = by_phase.get((run_idx, "target_pressure"), [])
        vacuum_data = by_phase.get((run_idx, "target_vacuum"), [])

        if pressure_data:
            t_values = [item[0] for item in pressure_data]
            y_values = [item[1] for item in pressure_data]
            t_target, overshoot, settle = phase_metrics(
                t_values,
                y_values,
                target=float(target_pressure_kpa),
                phase="target_pressure",
                band_kpa=float(settle_band_kpa),
                hold_s=float(settle_hold_s),
            )
            if t_target is not None:
                tp.append(float(t_target))
            op.append(float(overshoot))
            if settle is not None:
                sp.append(float(settle))

        if vacuum_data:
            t_values = [item[0] for item in vacuum_data]
            y_values = [item[1] for item in vacuum_data]
            t_target, overshoot, settle = phase_metrics(
                t_values,
                y_values,
                target=float(target_vacuum_kpa),
                phase="target_vacuum",
                band_kpa=float(settle_band_kpa),
                hold_s=float(settle_hold_s),
            )
            if t_target is not None:
                tv.append(float(t_target))
            ov.append(float(overshoot))
            if settle is not None:
                sv.append(float(settle))

    return EvalMetrics(
        label=os.path.basename(raw_path),
        target_pressure_kpa=float(target_pressure_kpa),
        target_vacuum_kpa=float(target_vacuum_kpa),
        time_target_pressure_mean_s=safe_mean(tp),
        time_target_vacuum_mean_s=safe_mean(tv),
        overshoot_pressure_mean_kpa=safe_mean(op),
        overshoot_vacuum_mean_kpa=safe_mean(ov),
        settling_pressure_mean_s=safe_mean(sp),
        settling_vacuum_mean_s=safe_mean(sv),
    )


def metrics_from_summary(summary_path: str, label: str) -> EvalMetrics:
    _, fields = parse_summary(summary_path)
    target_pressure = parse_float(fields.get("target_pressure_kpa")) or 0.0
    target_vacuum = parse_float(fields.get("target_vacuum_kpa")) or 0.0
    return EvalMetrics(
        label=label,
        target_pressure_kpa=float(target_pressure),
        target_vacuum_kpa=float(target_vacuum),
        time_target_pressure_mean_s=parse_float(fields.get("time_to_target_pressure_mean_s")),
        time_target_vacuum_mean_s=parse_float(fields.get("time_to_target_vacuum_mean_s")),
        overshoot_pressure_mean_kpa=parse_float(fields.get("overshoot_pressure_mean_kpa")),
        overshoot_vacuum_mean_kpa=parse_float(fields.get("overshoot_vacuum_mean_kpa")),
        settling_pressure_mean_s=parse_float(fields.get("settling_pressure_mean_s")),
        settling_vacuum_mean_s=parse_float(fields.get("settling_vacuum_mean_s")),
    )


def merge_metrics(preferred: EvalMetrics, fallback: EvalMetrics) -> EvalMetrics:
    return EvalMetrics(
        label=preferred.label,
        target_pressure_kpa=preferred.target_pressure_kpa or fallback.target_pressure_kpa,
        target_vacuum_kpa=preferred.target_vacuum_kpa or fallback.target_vacuum_kpa,
        time_target_pressure_mean_s=(
            preferred.time_target_pressure_mean_s
            if preferred.time_target_pressure_mean_s is not None
            else fallback.time_target_pressure_mean_s
        ),
        time_target_vacuum_mean_s=(
            preferred.time_target_vacuum_mean_s
            if preferred.time_target_vacuum_mean_s is not None
            else fallback.time_target_vacuum_mean_s
        ),
        overshoot_pressure_mean_kpa=(
            preferred.overshoot_pressure_mean_kpa
            if preferred.overshoot_pressure_mean_kpa is not None
            else fallback.overshoot_pressure_mean_kpa
        ),
        overshoot_vacuum_mean_kpa=(
            preferred.overshoot_vacuum_mean_kpa
            if preferred.overshoot_vacuum_mean_kpa is not None
            else fallback.overshoot_vacuum_mean_kpa
        ),
        settling_pressure_mean_s=(
            preferred.settling_pressure_mean_s
            if preferred.settling_pressure_mean_s is not None
            else fallback.settling_pressure_mean_s
        ),
        settling_vacuum_mean_s=(
            preferred.settling_vacuum_mean_s
            if preferred.settling_vacuum_mean_s is not None
            else fallback.settling_vacuum_mean_s
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate controller validation comparison report")
    parser.add_argument("--baseline-summary", required=True)
    parser.add_argument("--candidate-summary", required=True)
    parser.add_argument("--baseline-raw", default="")
    parser.add_argument("--candidate-raw", default="")
    parser.add_argument("--settle-band-kpa", type=float, default=1.0)
    parser.add_argument("--settle-hold-s", type=float, default=0.4)
    parser.add_argument("--max-time-target-s", type=float, default=0.35)
    parser.add_argument("--max-overshoot-kpa", type=float, default=2.0)
    parser.add_argument("--max-settling-s", type=float, default=1.0)
    parser.add_argument("--tag", type=str, default="ab")
    return parser.parse_args()


def write_csv(path: str, rows: list[EvalMetrics]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "label",
                "target_pressure_kpa",
                "target_vacuum_kpa",
                "time_target_pressure_mean_s",
                "time_target_vacuum_mean_s",
                "overshoot_pressure_mean_kpa",
                "overshoot_vacuum_mean_kpa",
                "settling_pressure_mean_s",
                "settling_vacuum_mean_s",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.label,
                    f"{row.target_pressure_kpa:.6f}",
                    f"{row.target_vacuum_kpa:.6f}",
                    ""
                    if row.time_target_pressure_mean_s is None
                    else f"{row.time_target_pressure_mean_s:.6f}",
                    ""
                    if row.time_target_vacuum_mean_s is None
                    else f"{row.time_target_vacuum_mean_s:.6f}",
                    ""
                    if row.overshoot_pressure_mean_kpa is None
                    else f"{row.overshoot_pressure_mean_kpa:.6f}",
                    ""
                    if row.overshoot_vacuum_mean_kpa is None
                    else f"{row.overshoot_vacuum_mean_kpa:.6f}",
                    ""
                    if row.settling_pressure_mean_s is None
                    else f"{row.settling_pressure_mean_s:.6f}",
                    ""
                    if row.settling_vacuum_mean_s is None
                    else f"{row.settling_vacuum_mean_s:.6f}",
                ]
            )


def pass_fail(
    value: float | None, threshold: float, lower_is_better: bool = True
) -> tuple[str, str]:
    if value is None:
        return "FAIL", "N/A"
    if lower_is_better:
        return (
            ("PASS", f"{value:.3f} <= {threshold:.3f}")
            if value <= threshold
            else (
                "FAIL",
                f"{value:.3f} > {threshold:.3f}",
            )
        )
    return (
        ("PASS", f"{value:.3f} >= {threshold:.3f}")
        if value >= threshold
        else (
            "FAIL",
            f"{value:.3f} < {threshold:.3f}",
        )
    )


def write_md(
    path: str, baseline: EvalMetrics, candidate: EvalMetrics, args: argparse.Namespace
) -> bool:
    checks = [
        (
            "time_target_pressure_mean_s",
            candidate.time_target_pressure_mean_s,
            args.max_time_target_s,
        ),
        ("time_target_vacuum_mean_s", candidate.time_target_vacuum_mean_s, args.max_time_target_s),
        (
            "overshoot_pressure_mean_kpa",
            candidate.overshoot_pressure_mean_kpa,
            args.max_overshoot_kpa,
        ),
        ("overshoot_vacuum_mean_kpa", candidate.overshoot_vacuum_mean_kpa, args.max_overshoot_kpa),
        ("settling_pressure_mean_s", candidate.settling_pressure_mean_s, args.max_settling_s),
        ("settling_vacuum_mean_s", candidate.settling_vacuum_mean_s, args.max_settling_s),
    ]
    results: list[tuple[str, str, str]] = []
    for name, value, threshold in checks:
        status, detail = pass_fail(value, float(threshold), lower_is_better=True)
        results.append((name, status, detail))

    all_pass = all(status == "PASS" for _, status, _ in results)

    lines: list[str] = []
    lines.append("# Controller Validation Comparison")
    lines.append("")
    lines.append(f"- Baseline: {baseline.label}")
    lines.append(f"- Candidate: {candidate.label}")
    lines.append(
        "- Targets: "
        f"pressure={candidate.target_pressure_kpa:.2f} kPa, "
        f"vacuum={candidate.target_vacuum_kpa:.2f} kPa"
    )
    lines.append("")
    lines.append("## Metrics")
    lines.append("")
    lines.append("| Metric | Baseline | Candidate | Threshold | Status |")
    lines.append("| --- | ---: | ---: | ---: | --- |")

    metric_map = {
        "time_target_pressure_mean_s": (
            baseline.time_target_pressure_mean_s,
            candidate.time_target_pressure_mean_s,
            args.max_time_target_s,
        ),
        "time_target_vacuum_mean_s": (
            baseline.time_target_vacuum_mean_s,
            candidate.time_target_vacuum_mean_s,
            args.max_time_target_s,
        ),
        "overshoot_pressure_mean_kpa": (
            baseline.overshoot_pressure_mean_kpa,
            candidate.overshoot_pressure_mean_kpa,
            args.max_overshoot_kpa,
        ),
        "overshoot_vacuum_mean_kpa": (
            baseline.overshoot_vacuum_mean_kpa,
            candidate.overshoot_vacuum_mean_kpa,
            args.max_overshoot_kpa,
        ),
        "settling_pressure_mean_s": (
            baseline.settling_pressure_mean_s,
            candidate.settling_pressure_mean_s,
            args.max_settling_s,
        ),
        "settling_vacuum_mean_s": (
            baseline.settling_vacuum_mean_s,
            candidate.settling_vacuum_mean_s,
            args.max_settling_s,
        ),
    }

    for name, status, _detail in results:
        base_val, cand_val, threshold = metric_map[name]
        base_txt = "N/A" if base_val is None else f"{base_val:.3f}"
        cand_txt = "N/A" if cand_val is None else f"{cand_val:.3f}"
        lines.append(f"| {name} | {base_txt} | {cand_txt} | <= {float(threshold):.3f} | {status} |")

    lines.append("")
    lines.append(f"## Verdict: {'PASS' if all_pass else 'FAIL'}")
    lines.append("")

    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")

    return all_pass


def main() -> int:
    args = parse_args()

    for required in (args.baseline_summary, args.candidate_summary):
        if not os.path.exists(required):
            raise SystemExit(f"Summary file not found: {required}")

    baseline_from_summary = metrics_from_summary(args.baseline_summary, label="baseline")
    candidate_from_summary = metrics_from_summary(args.candidate_summary, label="candidate")

    baseline_raw = args.baseline_raw or (infer_raw_from_summary(args.baseline_summary) or "")
    candidate_raw = args.candidate_raw or (infer_raw_from_summary(args.candidate_summary) or "")

    if baseline_raw and os.path.exists(baseline_raw):
        baseline_from_raw = metrics_from_raw(
            baseline_raw,
            target_pressure_kpa=baseline_from_summary.target_pressure_kpa,
            target_vacuum_kpa=baseline_from_summary.target_vacuum_kpa,
            settle_band_kpa=float(args.settle_band_kpa),
            settle_hold_s=float(args.settle_hold_s),
        )
        baseline = merge_metrics(baseline_from_raw, baseline_from_summary)
        baseline.label = "baseline"
    else:
        baseline = baseline_from_summary

    if candidate_raw and os.path.exists(candidate_raw):
        candidate_from_raw = metrics_from_raw(
            candidate_raw,
            target_pressure_kpa=candidate_from_summary.target_pressure_kpa,
            target_vacuum_kpa=candidate_from_summary.target_vacuum_kpa,
            settle_band_kpa=float(args.settle_band_kpa),
            settle_hold_s=float(args.settle_hold_s),
        )
        candidate = merge_metrics(candidate_from_raw, candidate_from_summary)
        candidate.label = "candidate"
    else:
        candidate = candidate_from_summary

    out_dir = os.path.join(REPO_ROOT, "experiments", time.strftime("%Y-%m"))
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = "".join(ch if (ch.isalnum() or ch in {"_", "-"}) else "_" for ch in str(args.tag).strip())
    tag = tag.strip("_") or "ab"

    csv_path = os.path.join(out_dir, f"comparison_{stamp}_{tag}.csv")
    md_path = os.path.join(out_dir, f"comparison_{stamp}_{tag}.md")

    write_csv(csv_path, [baseline, candidate])
    all_pass = write_md(md_path, baseline, candidate, args)

    print("Controller validation report completed")
    print(f"- CSV: {csv_path}")
    print(f"- MD:  {md_path}")
    print(f"- Verdict: {'PASS' if all_pass else 'FAIL'}")

    return 0 if all_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
