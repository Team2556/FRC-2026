from dataclasses import dataclass, field
from typing import Union

from phoenix6.controls import (
    SolidColor,
    ColorFlowAnimation,
    FireAnimation,
    LarsonAnimation,
    RainbowAnimation,
    RgbFadeAnimation,
    SingleFadeAnimation,
    StrobeAnimation,
    TwinkleAnimation,
    TwinkleOffAnimation,
)

LED_request = Union[
    SolidColor,
    ColorFlowAnimation,
    FireAnimation,
    LarsonAnimation,
    RainbowAnimation,
    RgbFadeAnimation,
    SingleFadeAnimation,
    StrobeAnimation,
    TwinkleAnimation,
    TwinkleOffAnimation,
]

@dataclass
class CANdle_State:
    animation_request: LED_request = None
    priority: int = 0
    enabled: bool = False
