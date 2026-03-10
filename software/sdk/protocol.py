"""Shared control protocol constants and helpers for SoftBot ROS topics."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
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

PID_CONTROL_MODES = (MODE_PID_INFLATE, MODE_PID_SUCTION)
PWM_CONTROL_MODES = (MODE_PWM_INFLATE, MODE_PWM_SUCTION)
PRESSURE_CONTROL_MODES = PID_CONTROL_MODES + PWM_CONTROL_MODES

# Atomic pneumatic command protocol.
PNEUMATIC_PROTOCOL_VERSION = 1
PNEUMATIC_COMMAND_LENGTH = 6
PNEUMATIC_STATE_LENGTH = 10

PNEUMATIC_BEHAVIOR_DIRECT = 0
PNEUMATIC_BEHAVIOR_AUTO = 1
PNEUMATIC_BEHAVIOR_ARM = 2
PNEUMATIC_BEHAVIOR_FIRE = 3
VALID_PNEUMATIC_BEHAVIORS = (
    PNEUMATIC_BEHAVIOR_DIRECT,
    PNEUMATIC_BEHAVIOR_AUTO,
    PNEUMATIC_BEHAVIOR_ARM,
    PNEUMATIC_BEHAVIOR_FIRE,
)

PNEUMATIC_STATE_IDLE = 0
PNEUMATIC_STATE_DIRECT = 1
PNEUMATIC_STATE_PRECHARGE = 2
PNEUMATIC_STATE_READY_HOLD = 3
PNEUMATIC_STATE_DELIVER = 4
PNEUMATIC_STATE_VENT = 5
PNEUMATIC_STATE_FAULT = 6

PNEUMATIC_FLAG_READY = 1 << 0
PNEUMATIC_FLAG_ARMED = 1 << 1
PNEUMATIC_FLAG_DELIVERING = 1 << 2
PNEUMATIC_FLAG_TIMEOUT = 1 << 3
PNEUMATIC_FLAG_COMMAND_REJECTED = 1 << 4
PNEUMATIC_FLAG_EMERGENCY_STOP = 1 << 5
PNEUMATIC_FLAG_LEGACY_INPUT = 1 << 6

PNEUMATIC_BEHAVIOR_LABELS = {
    PNEUMATIC_BEHAVIOR_DIRECT: "DIRECT",
    PNEUMATIC_BEHAVIOR_AUTO: "AUTO",
    PNEUMATIC_BEHAVIOR_ARM: "ARM",
    PNEUMATIC_BEHAVIOR_FIRE: "FIRE",
}

PNEUMATIC_STATE_LABELS = {
    PNEUMATIC_STATE_IDLE: "IDLE",
    PNEUMATIC_STATE_DIRECT: "DIRECT",
    PNEUMATIC_STATE_PRECHARGE: "PRECHARGE",
    PNEUMATIC_STATE_READY_HOLD: "READY_HOLD",
    PNEUMATIC_STATE_DELIVER: "DELIVER",
    PNEUMATIC_STATE_VENT: "VENT",
    PNEUMATIC_STATE_FAULT: "FAULT",
}

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


@dataclass(frozen=True)
class PneumaticStateFrame:
    version: int
    state: int
    mode: int
    behavior: int
    requested_chamber_mask: int
    applied_chamber_mask: int
    flags: int
    target_raw: int
    source_pressure_raw: int
    command_token: int

    @property
    def state_label(self) -> str:
        return PNEUMATIC_STATE_LABELS.get(self.state, str(self.state))

    @property
    def behavior_label(self) -> str:
        return PNEUMATIC_BEHAVIOR_LABELS.get(self.behavior, str(self.behavior))

    @property
    def target_value(self) -> float:
        return decode_pneumatic_target(self.mode, self.target_raw)

    @property
    def source_pressure_kpa(self) -> float:
        return deci_kpa_to_float(self.source_pressure_raw)


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


def validate_chamber_mask(chamber_mask: int) -> int:
    chamber_mask = int(chamber_mask)
    if chamber_mask not in VALID_CHAMBERS:
        raise ValueError(f"Invalid chamber {chamber_mask}. Valid values: {VALID_CHAMBERS}")
    return chamber_mask


def validate_pneumatic_behavior(mode: int, behavior: int) -> int:
    mode = int(mode)
    behavior = int(behavior)
    if behavior not in VALID_PNEUMATIC_BEHAVIORS:
        raise ValueError(
            f"Invalid pneumatic behavior {behavior}. Valid values: {VALID_PNEUMATIC_BEHAVIORS}"
        )
    if mode in PWM_CONTROL_MODES and behavior != PNEUMATIC_BEHAVIOR_DIRECT:
        raise ValueError("PWM modes only support DIRECT behavior.")
    if mode not in PID_CONTROL_MODES and behavior in (
        PNEUMATIC_BEHAVIOR_AUTO,
        PNEUMATIC_BEHAVIOR_ARM,
        PNEUMATIC_BEHAVIOR_FIRE,
    ):
        raise ValueError("AUTO/ARM/FIRE are only valid for PID inflate/suction modes.")
    return behavior


def clamp_int16(value: int | float) -> int:
    value = int(round(float(value)))
    if value > 32767:
        return 32767
    if value < -32768:
        return -32768
    return value


def encode_deci_kpa(value_kpa: int | float) -> int:
    return clamp_int16(float(value_kpa) * 10.0)


def deci_kpa_to_float(value: int | float) -> float:
    return float(int(value)) / 10.0


def encode_pneumatic_target(mode: int, target: int | float) -> int:
    mode = int(mode)
    if mode in PID_CONTROL_MODES:
        return encode_deci_kpa(target)
    if mode in PWM_CONTROL_MODES:
        return clamp_int16(max(0, min(255, int(round(float(target))))))
    return 0


def decode_pneumatic_target(mode: int, raw_target: int | float) -> float:
    mode = int(mode)
    raw_target = int(raw_target)
    if mode in PID_CONTROL_MODES:
        return deci_kpa_to_float(raw_target)
    if mode in PWM_CONTROL_MODES:
        return float(raw_target)
    return 0.0


def build_pneumatic_command_payload(
    *,
    mode: int,
    chamber_mask: int,
    behavior: int,
    target: int | float,
    token: int = 0,
) -> list[int]:
    mode = int(mode)
    chamber_mask = validate_chamber_mask(chamber_mask)
    behavior = validate_pneumatic_behavior(mode, behavior)
    return [
        PNEUMATIC_PROTOCOL_VERSION,
        mode,
        chamber_mask,
        behavior,
        encode_pneumatic_target(mode, target),
        clamp_int16(token),
    ]


def decode_pneumatic_state_payload(payload: Sequence[int | float]) -> PneumaticStateFrame:
    if len(payload) < PNEUMATIC_STATE_LENGTH:
        raise ValueError(
            f"Expected {PNEUMATIC_STATE_LENGTH} pneumatic-state values, got {len(payload)}."
        )
    return PneumaticStateFrame(
        version=int(payload[0]),
        state=int(payload[1]),
        mode=int(payload[2]),
        behavior=int(payload[3]),
        requested_chamber_mask=int(payload[4]),
        applied_chamber_mask=int(payload[5]),
        flags=int(payload[6]),
        target_raw=int(payload[7]),
        source_pressure_raw=int(payload[8]),
        command_token=int(payload[9]),
    )


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
