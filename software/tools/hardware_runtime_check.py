#!/usr/bin/env python3
"""Runtime verification for hardware ROS graph and firmware telemetry topics."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

EXPECTED_FIRMWARE_NODE = "soft_robot_node"

EXPECTED_TELEMETRY_TOPICS = {
    "/sensor/pressure": "std_msgs/msg/Float32",
    "/sensor/vacuum": "std_msgs/msg/Float32",
    "/system_debug": "std_msgs/msg/Int16MultiArray",
    "/pneumatic_state": "std_msgs/msg/Int16MultiArray",
}

EXPECTED_COMMAND_TOPICS = {
    "/pneumatic_command": "std_msgs/msg/Int16MultiArray",
    "/pressure_mode": "std_msgs/msg/Int8",
    "/pressure_setpoint": "std_msgs/msg/Float32",
    "/active_chamber": "std_msgs/msg/Int8",
    "/tuning_params": "std_msgs/msg/Float32MultiArray",
    "/hardware_test": "std_msgs/msg/Int16",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify SoftBot runtime ROS graph")
    parser.add_argument("--timeout-s", type=float, default=8.0, help="Graph discovery timeout")
    parser.add_argument(
        "--sample-timeout-s",
        type=float,
        default=3.0,
        help="Timeout waiting for /system_debug sample",
    )
    parser.add_argument(
        "--node-name",
        type=str,
        default=EXPECTED_FIRMWARE_NODE,
        help="Expected firmware ROS node name",
    )
    parser.add_argument(
        "--no-system-debug-sample",
        action="store_true",
        help="Skip waiting for runtime sample payloads (/system_debug and /pneumatic_state)",
    )
    return parser.parse_args()


def topic_types_map(node: Node) -> dict[str, set[str]]:
    return {name: set(types) for name, types in node.get_topic_names_and_types()}


def topic_type_ok(topic_types: dict[str, set[str]], topic: str, expected_type: str) -> bool:
    return expected_type in topic_types.get(topic, set())


def list_node_names(node: Node) -> set[str]:
    return {name for name, _namespace in node.get_node_names_and_namespaces()}


def wait_for_graph(node: Node, args: argparse.Namespace) -> tuple[bool, dict[str, list[str]]]:
    deadline = time.monotonic() + max(0.1, float(args.timeout_s))

    while True:
        topic_types = topic_types_map(node)
        node_names = list_node_names(node)

        missing: dict[str, list[str]] = {
            "nodes": [],
            "telemetry_types": [],
            "telemetry_publishers": [],
            "command_types": [],
            "command_subscribers": [],
        }

        if args.node_name and args.node_name not in node_names:
            missing["nodes"].append(args.node_name)

        for topic, msg_type in EXPECTED_TELEMETRY_TOPICS.items():
            if not topic_type_ok(topic_types, topic, msg_type):
                missing["telemetry_types"].append(topic)
            if node.count_publishers(topic) < 1:
                missing["telemetry_publishers"].append(topic)

        for topic, msg_type in EXPECTED_COMMAND_TOPICS.items():
            if not topic_type_ok(topic_types, topic, msg_type):
                missing["command_types"].append(topic)
            if node.count_subscribers(topic) < 1:
                missing["command_subscribers"].append(topic)

        if all(len(items) == 0 for items in missing.values()):
            return True, missing

        if time.monotonic() >= deadline:
            return False, missing

        rclpy.spin_once(node, timeout_sec=0.2)


def wait_for_runtime_samples(node: Node, timeout_s: float) -> tuple[bool, list[int], list[int]]:
    debug_sample: list[int] = []
    pneumatic_sample: list[int] = []

    def _cb_debug(msg: Int16MultiArray) -> None:
        nonlocal debug_sample
        debug_sample = list(msg.data)

    def _cb_pneumatic(msg: Int16MultiArray) -> None:
        nonlocal pneumatic_sample
        pneumatic_sample = list(msg.data)

    debug_subscription = node.create_subscription(Int16MultiArray, "/system_debug", _cb_debug, 10)
    pneumatic_subscription = node.create_subscription(
        Int16MultiArray,
        "/pneumatic_state",
        _cb_pneumatic,
        10,
    )
    try:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while time.monotonic() < deadline:
            if debug_sample and pneumatic_sample:
                return True, debug_sample, pneumatic_sample
            rclpy.spin_once(node, timeout_sec=0.1)
        return False, debug_sample, pneumatic_sample
    finally:
        node.destroy_subscription(debug_subscription)
        node.destroy_subscription(pneumatic_subscription)


def system_debug_schema_ok(sample: list[int]) -> tuple[bool, str]:
    if len(sample) != 6:
        return False, f"expected 6 items, got {len(sample)}"
    pwm_main, pwm_aux, _ch0_x10, _ch1_x10, mode, flags = sample
    if pwm_main < -1 or pwm_main > 255:
        return False, f"pwm_main fuera de rango: {pwm_main}"
    if pwm_aux < -1 or pwm_aux > 255:
        return False, f"pwm_aux fuera de rango: {pwm_aux}"
    if mode not in (-2, -1, 0, 1, 2, 4, 9):
        return False, f"mode inesperado: {mode}"
    if flags < 0 or flags > 1024:
        return False, f"flags fuera de rango: {flags}"
    return True, "ok"


def pneumatic_state_schema_ok(sample: list[int]) -> tuple[bool, str]:
    if len(sample) != 10:
        return False, f"expected 10 items, got {len(sample)}"
    version, state, mode, behavior, requested, applied, flags, _target, _source, _token = sample
    if version != 1:
        return False, f"version inesperada: {version}"
    if state not in range(0, 7):
        return False, f"state inesperado: {state}"
    if mode not in (-2, -1, 0, 1, 2, 4, 9):
        return False, f"mode inesperado: {mode}"
    if behavior not in (0, 1, 2, 3):
        return False, f"behavior inesperado: {behavior}"
    if requested < 0 or requested > 7:
        return False, f"requested_chamber_mask fuera de rango: {requested}"
    if applied < 0 or applied > 7:
        return False, f"applied_chamber_mask fuera de rango: {applied}"
    if flags < 0 or flags > 4096:
        return False, f"flags fuera de rango: {flags}"
    return True, "ok"


def print_missing(missing: dict[str, list[str]]) -> None:
    for key, items in missing.items():
        if items:
            print(f"[FAIL] {key}: {', '.join(items)}")
        else:
            print(f"[OK] {key}")


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = Node("softbot_runtime_check")
    try:
        print("SoftBot runtime check")
        print(f"[info] expected firmware node: {args.node_name}")
        print(f"[info] graph timeout: {args.timeout_s:.1f}s")

        graph_ok, missing = wait_for_graph(node, args)
        print_missing(missing)

        if not graph_ok:
            print("[hint] Verifica agent start, puerto serial, y reset físico de la ESP32.")
            return 1

        if args.no_system_debug_sample:
            print("[OK] runtime sample checks: skipped")
            return 0

        sample_ok, debug_sample, pneumatic_sample = wait_for_runtime_samples(
            node,
            timeout_s=args.sample_timeout_s,
        )
        if not sample_ok:
            print(f"[FAIL] runtime samples: timeout ({args.sample_timeout_s:.1f}s)")
            print(f"[info] /system_debug recibido: {debug_sample}")
            print(f"[info] /pneumatic_state recibido: {pneumatic_sample}")
            print(
                "[hint] Los tópicos pueden existir sin tráfico si el firmware "
                "no está ejecutando controlLoop."
            )
            return 1

        debug_schema_ok, debug_schema_msg = system_debug_schema_ok(debug_sample)
        if not debug_schema_ok:
            print(f"[FAIL] /system_debug schema: {debug_schema_msg}")
            print(f"[info] payload recibido: {debug_sample}")
            return 1

        pneumatic_schema_ok, pneumatic_schema_msg = pneumatic_state_schema_ok(pneumatic_sample)
        if not pneumatic_schema_ok:
            print(f"[FAIL] /pneumatic_state schema: {pneumatic_schema_msg}")
            print(f"[info] payload recibido: {pneumatic_sample}")
            return 1

        print(f"[OK] /system_debug sample: {debug_sample}")
        print("[OK] /system_debug schema: [pwm_main,pwm_aux,ch0_x10,ch1_x10,mode,flags]")
        print(f"[OK] /pneumatic_state sample: {pneumatic_sample}")
        print(
            "[OK] /pneumatic_state schema: "
            "[version,state,mode,behavior,requested,applied,flags,target,source,token]"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
