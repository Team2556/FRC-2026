import commands2

from ntcore import NetworkTableInstance, StringPublisher

import phoenix6

from subsystems.led.LED_io import LED_request, CANdle_State
from subsystems.led.colors import CANdle_Color

from constants.led import kLED

from util.flip_util import FlipUtil


class CANdleLEDController(commands2.Subsystem):
    def __init__(self, CAN_id: int):
        super().__init__()
        self._candle = phoenix6.hardware.CANdle(CAN_id)

        self.states: dict[str, CANdle_State] = {}
        
        # What IS actuall being ran on the LED
        self.current_state_key: str | None = None
        # What NEEDs to be ran on the LED
        self.active_state_key: str | None = None

        self.nt = NetworkTableInstance.getDefault().getTable("LED")
        self.nt_sub: dict[str, StringPublisher] = {
            "state": self.nt.getStringTopic("State").publish(),
            "isValid": self.nt.getBooleanTopic("Valid").publish(),
            "description": self.nt.getStringTopic("Status Description").publish(),
        }

        default_color = CANdle_Color.RED if FlipUtil.shouldFlip() else CANdle_Color.BLUE
        self.create_state(
            state_key="default",
            animation_request=phoenix6.controls.SolidColor(
                kLED.LED_STRIP_INDEX, kLED.LED_STRIP_INDEX, default_color
            ),
            priority=-1,
            enable=True,
        )

    def create_state(
        self,
        state_key: str,
        animation_request: LED_request,
        priority: int,
        enable: bool = False,
    ) -> None:
        self.states[state_key] = CANdle_State(
            animation_request=animation_request,
            priority=priority,
            enabled=enable,
        )

        self.update_target_state()

    def toggle_state(self, state_key: str, is_enabled: bool) -> None:
        state = self.states.get(state_key)
        if state is None:
            return
        state.enabled = is_enabled

        self.update_target_state()

    def enable_state(self, state_key: str) -> None:
        self.toggle_state(self, state_key, True)

    def disable_state(self, state_key: str) -> None:
        self.toggle_state(self, state_key, False)

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

        animation_request = self.get_animation()
        STATUS_CODE = self._candle.set_control(animation_request)

        self.nt_sub["state"].set(self.active_state_key)
        self.nt_sub["isValid"].set(STATUS_CODE.is_ok())
        self.nt_sub["description"].set(STATUS_CODE.description)
