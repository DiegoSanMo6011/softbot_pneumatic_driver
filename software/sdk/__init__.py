"""SDK de SoftBot: interfaz y constantes de protocolo."""

from .protocol import (  # noqa: F401
    CHAMBER_A,
    CHAMBER_AB,
    CHAMBER_B,
    CHAMBER_BLOCKED,
    MODE_LABELS,
    MODE_PID_INFLATE,
    MODE_PID_INFLATE_TURBO,
    MODE_PID_SUCTION,
    MODE_PWM_INFLATE,
    MODE_PWM_SUCTION,
    MODE_STOP,
    MODE_TANK_FILL,
    MODE_VENT,
)
from .softbot_interface import SoftBot

__all__ = [
    "SoftBot",
    "CHAMBER_BLOCKED",
    "CHAMBER_A",
    "CHAMBER_B",
    "CHAMBER_AB",
    "MODE_STOP",
    "MODE_PID_INFLATE",
    "MODE_PID_SUCTION",
    "MODE_PWM_INFLATE",
    "MODE_PWM_SUCTION",
    "MODE_TANK_FILL",
    "MODE_VENT",
    "MODE_PID_INFLATE_TURBO",
    "MODE_LABELS",
]
