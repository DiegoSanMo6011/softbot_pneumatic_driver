"""Shared control protocol constants and helpers for SoftBot ROS topics."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

# Chamber selection.
CHAMBER_BLOCKED = 0
CHAMBER_A = 1
CHAMBER_B = 2
CHAMBER_C = 4
CHAMBER_AB = CHAMBER_A | CHAMBER_B
CHAMBER_AC = CHAMBER_A | CHAMBER_C
CHAMBER_BC = CHAMBER_B | CHAMBER_C
CHAMBER_ABC = CHAMBER_A | CHAMBER_B | CHAMBER_C
VALID_CHAMBERS = (
    CHAMBER_BLOCKED,
    CHAMBER_A,
    CHAMBER_B,
    CHAMBER_C,
    CHAMBER_AB,
    CHAMBER_AC,
    CHAMBER_BC,
    CHAMBER_ABC,
)

# Pressure control modes.
MODE_STOP = 0
MODE_PID_INFLATE = 1
MODE_PID_SUCTION = -1
MODE_PWM_INFLATE = 2
MODE_PWM_SUCTION = -2
MODE_VENT = 4
MODE_HARDWARE_DIAGNOSTIC = 9

# Hardware diagnostic bitmask for /hardware_test.
HW_PUMP_INFLATE_MAIN = 1 << 0
HW_PUMP_INFLATE_AUX = 1 << 1
HW_PUMP_SUCTION_MAIN = 1 << 2
HW_PUMP_SUCTION_AUX = 1 << 3
HW_VALVE_INFLATE = 1 << 4
HW_VALVE_SUCTION = 1 << 5
HW_VALVE_CHAMBER_C = 1 << 6
HW_MUX_CHAMBER_A = 1 << 7
HW_MUX_CHAMBER_B = 1 << 8

HARDWARE_TEST_MAX_BIT = 15
HARDWARE_TEST_MAX_MASK = (1 << (HARDWARE_TEST_MAX_BIT + 1)) - 1


@dataclass(frozen=True)
class HardwareComponent:
    id: str
    label: str
    bit: int
    kind: str
    group: str
    ui_order: int

    @property
    def mask(self) -> int:
        return 1 << self.bit


HARDWARE_COMPONENTS = (
    HardwareComponent(
        id="inflate_main",
        label="Pump Inflate Main",
        bit=0,
        kind="pump",
        group="pressure",
        ui_order=10,
    ),
    HardwareComponent(
        id="inflate_aux",
        label="Pump Inflate Aux",
        bit=1,
        kind="pump",
        group="pressure",
        ui_order=11,
    ),
    HardwareComponent(
        id="suction_main",
        label="Pump Suction Main",
        bit=2,
        kind="pump",
        group="vacuum",
        ui_order=20,
    ),
    HardwareComponent(
        id="suction_aux",
        label="Pump Suction Aux",
        bit=3,
        kind="pump",
        group="vacuum",
        ui_order=21,
    ),
    HardwareComponent(
        id="valve_inflate",
        label="Valve Inflate",
        bit=4,
        kind="valve",
        group="valve",
        ui_order=30,
    ),
    HardwareComponent(
        id="valve_suction",
        label="Valve Suction",
        bit=5,
        kind="valve",
        group="valve",
        ui_order=31,
    ),
    HardwareComponent(
        id="valve_chamber_c",
        label="Valve Chamber C",
        bit=6,
        kind="valve",
        group="valve",
        ui_order=32,
    ),
    HardwareComponent(
        id="mux_a",
        label="Mux Chamber A",
        bit=7,
        kind="mux",
        group="mux",
        ui_order=40,
    ),
    HardwareComponent(
        id="mux_b",
        label="Mux Chamber B",
        bit=8,
        kind="mux",
        group="mux",
        ui_order=41,
    ),
)

HARDWARE_COMPONENT_BY_ID = {component.id: component for component in HARDWARE_COMPONENTS}
HARDWARE_COMPONENT_IDS = tuple(component.id for component in HARDWARE_COMPONENTS)
HARDWARE_COMPONENT_BITS = {component.id: component.mask for component in HARDWARE_COMPONENTS}

HARDWARE_PUMP_GROUPS = {
    "pressure": ("inflate_main", "inflate_aux"),
    "vacuum": ("suction_main", "suction_aux"),
}


def validate_hardware_components_fit_int16() -> None:
    """Validate software component catalog against /hardware_test Int16 constraints."""
    seen_ids: set[str] = set()
    seen_bits: set[int] = set()
    for component in HARDWARE_COMPONENTS:
        if component.id in seen_ids:
            raise ValueError(f"Duplicate hardware component id: {component.id}")
        if component.bit in seen_bits:
            raise ValueError(f"Duplicate hardware component bit: {component.bit}")
        if component.bit > HARDWARE_TEST_MAX_BIT:
            raise ValueError(
                f"Component {component.id} uses bit {component.bit}, exceeds Int16 limit"
            )
        seen_ids.add(component.id)
        seen_bits.add(component.bit)


def component_ids_by_kind(kind: str) -> tuple[str, ...]:
    kind = str(kind).strip().lower()
    return tuple(component.id for component in HARDWARE_COMPONENTS if component.kind == kind)


def component_id_to_mask(component_id: str) -> int:
    key = str(component_id).strip().lower()
    if key not in HARDWARE_COMPONENT_BITS:
        valid = ", ".join(HARDWARE_COMPONENT_IDS)
        raise ValueError(f"Unknown hardware component '{component_id}'. Valid: {valid}")
    return HARDWARE_COMPONENT_BITS[key]


def build_hardware_mask(
    component_ids: Iterable[str] | None = None,
    groups: Iterable[str] | None = None,
) -> int:
    mask = 0

    if groups:
        for raw_group in groups:
            group = str(raw_group).strip().lower()
            if group not in HARDWARE_PUMP_GROUPS:
                valid = ", ".join(sorted(HARDWARE_PUMP_GROUPS))
                raise ValueError(f"Unknown hardware group '{raw_group}'. Valid: {valid}")
            for component_id in HARDWARE_PUMP_GROUPS[group]:
                mask |= component_id_to_mask(component_id)

    if component_ids:
        for component_id in component_ids:
            mask |= component_id_to_mask(component_id)

    if mask < 0 or mask > HARDWARE_TEST_MAX_MASK:
        raise ValueError(
            f"Hardware mask out of Int16 range: {mask}. Max allowed: {HARDWARE_TEST_MAX_MASK}"
        )
    return mask


def decode_hardware_mask(mask: int) -> list[str]:
    mask = int(mask)
    return [component.id for component in HARDWARE_COMPONENTS if (mask & component.mask) != 0]


validate_hardware_components_fit_int16()

MODE_LABELS = {
    MODE_STOP: "STOP",
    MODE_PID_INFLATE: "PID_INFLATE",
    MODE_PID_SUCTION: "PID_SUCTION",
    MODE_PWM_INFLATE: "PWM_INFLATE",
    MODE_PWM_SUCTION: "PWM_SUCTION",
    MODE_VENT: "VENT",
    MODE_HARDWARE_DIAGNOSTIC: "HARDWARE_DIAGNOSTIC",
}
