from phoenix6.signals import RGBWColor
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
from subsystems.led import _LED_config


class CANdle_Color:
    RED = RGBWColor(255, 0, 0)
    BLUE = RGBWColor(17, 137, 240)
    GREEN = RGBWColor(0, 255, 0)
    PURPLE = RGBWColor(233, 35, 240)
    YELLOW = RGBWColor(240, 240, 0)
    BLACK = RGBWColor(0, 0, 0)
    WHITE = RGBWColor(0, 0, 0, 255)


class ColorFactories:
    """Builders for easy animations addressing entire LED strip."""

    @staticmethod
    def get_LED_interval() -> tuple[int, int]:
        start = _LED_config.LED_INDEX_START
        end = _LED_config.LED_INDEX_START + _LED_config.LED_LENGTH - 1
        return start, end

    @staticmethod
    def solid_color(color: RGBWColor) -> SolidColor:
        start, end = ColorFactories.get_LED_interval()
        return SolidColor(start, end, color)

    @staticmethod
    def color_flow(
        color: RGBWColor, speed: float = 0.5, forward: bool = True
    ) -> ColorFlowAnimation:
        start, end = ColorFactories.get_LED_interval()
        return ColorFlowAnimation(start, end, color, speed, forward)

    @staticmethod
    def fire(
        brightness: float = 0.5,
        speed: float = 0.5,
        sparking: float = 0.5,
        cooling: float = 0.5,
    ) -> FireAnimation:
        start, end = ColorFactories.get_LED_interval()
        return FireAnimation(start, end, brightness, speed, sparking, cooling)

    @staticmethod
    def larson(color: RGBWColor, speed: float = 0.5, size: int = 3) -> LarsonAnimation:
        start, end = ColorFactories.get_LED_interval()
        return LarsonAnimation(start, end, color, speed, size)

    @staticmethod
    def rainbow(brightness: float = 1.0, speed: float = 0.5) -> RainbowAnimation:
        start, end = ColorFactories.get_LED_interval()
        return RainbowAnimation(start, end, brightness, speed)

    @staticmethod
    def rgb_fade(brightness: float = 1.0, speed: float = 0.5) -> RgbFadeAnimation:
        start, end = ColorFactories.get_LED_interval()
        return RgbFadeAnimation(start, end, brightness, speed)

    @staticmethod
    def single_fade(
        color: RGBWColor, brightness: float = 1.0, speed: float = 0.5
    ) -> SingleFadeAnimation:
        start, end = ColorFactories.get_LED_interval()
        return SingleFadeAnimation(start, end, color, brightness, speed)

    @staticmethod
    def strobe(color: RGBWColor, speed: float = 0.5) -> StrobeAnimation:
        start, end = ColorFactories.get_LED_interval()
        return StrobeAnimation(start, end, color, speed)

    @staticmethod
    def twinkle(
        color: RGBWColor, speed: float = 0.5, divider: int = 4
    ) -> TwinkleAnimation:
        start, end = ColorFactories.get_LED_interval()
        return TwinkleAnimation(start, end, color, speed, divider)

    @staticmethod
    def twinkle_off(
        color: RGBWColor, speed: float = 0.5, divider: int = 4
    ) -> TwinkleOffAnimation:
        start, end = ColorFactories.get_LED_interval()
        return TwinkleOffAnimation(start, end, color, speed, divider)
