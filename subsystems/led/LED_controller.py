import commands2
import phoenix6

from util.nt_util import NTTable
from util.flip_util import FlipUtil

from subsystems.led.LED_io import LED_request, CANdle_State
from subsystems.led.LED_helpers import ColorFactories, CANdle_Color

from constants.led import kLED
from constants.canbus import kCANId


class CANdle_StateHandler:
    def __init__(self, state_key: str, controller: "CANdleLEDController"):
        self._state_key = state_key
        self._controller = controller

    def enable(self) -> None:
        self._controller.enable_state(self._state_key)

    def disable(self) -> None:
        self._controller.disable_state(self._state_key)

    def toggle(self, is_enabled: bool) -> None:
        self._controller.toggle_state(self._state_key, is_enabled)


class CANdleLEDController(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self._candle = phoenix6.hardware.CANdle(kCANId.CANDLE)
        self.states: dict[str, CANdle_State] = {}

        # What IS actuall being ran on the LED
        self.current_state_key: str | None = None
        # What NEEDs to be ran on the LED
        self.active_state_key: str | None = None

        self.nt = NTTable("LED")
        self.nt.string("State")
        self.nt.string("Status Description")
        self.nt.bool("Valid")

        default_color = CANdle_Color.RED if FlipUtil.shouldFlip() else CANdle_Color.BLUE
        self.create_state(
            state_key="default",
            animation_request=ColorFactories.solid_color(default_color),
            priority=-1,
            enable=True,
        )

        # Set the 8 onboard LEDs
        self._candle.set_control(ColorFactories.solid_color(default_color))

    def create_state(
        self,
        state_key: str,
        animation_request: LED_request,
        priority: int,
        enable: bool = False,
    ) -> CANdle_StateHandler:
        self.states[state_key] = CANdle_State(
            animation_request=animation_request,
            priority=priority,
            enabled=enable,
        )

        self.update_target_state()
        return CANdle_StateHandler(state_key, self)

    def toggle_state(self, state_key: str, is_enabled: bool) -> None:
        state = self.states.get(state_key)
        if state is None:
            return
        state.enabled = is_enabled

        self.update_target_state()

    def enable_state(self, state_key: str) -> None:
        self.toggle_state(state_key, True)

    def disable_state(self, state_key: str) -> None:
        self.toggle_state(state_key, False)

    def update_target_state(self) -> None:
        enabled_state_keys = [
            state_key for state_key, state in self.states.items() if state.enabled
        ]
        self.active_state_key = max(
            enabled_state_keys, key=lambda k: self.states[k].priority, default="default"
        )

    def get_animation(self, state_key: str) -> LED_request:
        state = self.states.get(state_key)
        if state is None:
            return None

        return state.animation_request

    def periodic(self):
        if self.current_state_key == self.active_state_key:
            return

        animation_request = self.get_animation(self.active_state_key)
        self.current_state_key = self.active_state_key

        self.nt.set("State", self.current_state_key)
        if animation_request is None:
            self.nt.set("Valid", False)
            self.nt.set("Status Description", "Animation Request is None")
            return

        self._candle.set_control(phoenix6.controls.EmptyAnimation(0))
        STATUS_CODE = self._candle.set_control(animation_request)

        self.nt.set("Valid", STATUS_CODE.is_ok())
        self.nt.set("Status Description", STATUS_CODE.description)
