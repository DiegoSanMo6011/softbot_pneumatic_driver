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
    "/pressure_feedback": "std_msgs/msg/Float32",
    "/system_debug": "std_msgs/msg/Int16MultiArray",
    "/tank_state": "std_msgs/msg/Int8",
}

EXPECTED_COMMAND_TOPICS = {
    "/pressure_mode": "std_msgs/msg/Int8",
    "/pressure_setpoint": "std_msgs/msg/Float32",
    "/active_chamber": "std_msgs/msg/Int8",
    "/tuning_params": "std_msgs/msg/Float32MultiArray",
    "/boost_valve": "std_msgs/msg/Int8",
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
        help="Skip waiting for /system_debug sample payload",
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


def wait_for_system_debug_sample(node: Node, timeout_s: float) -> tuple[bool, list[int]]:
    received: list[int] = []

    def _cb(msg: Int16MultiArray) -> None:
        nonlocal received
        received = list(msg.data)

    subscription = node.create_subscription(Int16MultiArray, "/system_debug", _cb, 10)
    try:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while time.monotonic() < deadline:
            if received:
                return True, received
            rclpy.spin_once(node, timeout_sec=0.1)
        return False, received
    finally:
        node.destroy_subscription(subscription)


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
            print("[OK] /system_debug sample check: skipped")
            return 0

        sample_ok, sample = wait_for_system_debug_sample(node, timeout_s=args.sample_timeout_s)
        if not sample_ok:
            print(f"[FAIL] /system_debug sample: timeout ({args.sample_timeout_s:.1f}s)")
            print(
                "[hint] El tópico puede existir sin tráfico si el firmware "
                "no está ejecutando controlLoop."
            )
            return 1

        print(f"[OK] /system_debug sample: {sample}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
