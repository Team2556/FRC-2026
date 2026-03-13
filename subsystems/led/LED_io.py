from dataclasses import dataclass, field
from typing import Union

from phoenix6.controls import (
    ModulateVBatOut,
    SolidColor,
    EmptyAnimation,
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
    ModulateVBatOut,
    SolidColor,
    EmptyAnimation,
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
    animation_request: LED_request = field(default_factory=EmptyAnimation(0))
    priority: int = 0
    enabled: bool = False
    
