import numpy as np

import wpilib
from commands2 import Subsystem
from phoenix6.hardware import TalonFXS
from phoenix6.controls import MotionMagicVoltage, DutyCycleOut
from phoenix6.signals import NeutralModeValue, ReverseLimitValue

from util.editable_pid import EditablePID
from util.nt_util import NTTable
from util.math_helpers import clamp

from constants.shooter import kHoodMotor, kShooterData
from constants.canbus import kCANId


class ShooterHood(Subsystem):
    def __init__(self):
        super().__init__()
        self._motor = TalonFXS(kCANId.shooter.HOOD_CONTROL, "rio")

        if not wpilib.RobotBase.isSimulation():
            self._motor.configurator.apply(kHoodMotor._CONFIG)
            self._motor.setNeutralMode(NeutralModeValue.BRAKE)

        self._pos_req  = MotionMagicVoltage(position=0, enable_foc=False)
        self._duty_req = DutyCycleOut(0, enable_foc=False)

        self._target_angle_deg   = kHoodMotor.HOME_ANGLE_DEG
        self._target_pos_revs    = kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG)
        self._resetting          = False
        self.hard_stopped        = False

        self.editable_pid = EditablePID("Shooter/Hood", self._motor, kHoodMotor._CONFIG)

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.float("Hood Position (deg)", 0.0)
        # self.nt.float("Target Angle (deg)", 0.0)
        # self.nt.bool("NT Override Enabled", False)
        # self.nt.float("NT Override Angle (deg)", 0.0)

    def set_target_angle(self, degrees: float) -> None:
        self._target_angle_deg = clamp(degrees, kHoodMotor.HOME_ANGLE_DEG, kHoodMotor.MAX_ANGLE_DEG)

    def set_speed(self, speed: float) -> None:
        self._motor.set_control(self._duty_req.with_output(speed))

    def zero_encoder(self) -> None:
        self._motor.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))

    def start_reset(self) -> None:
        self._resetting = True

    def end_reset(self) -> None:
        self._resetting = False

    def is_at_angle(self) -> bool:
        current_deg = kHoodMotor.to_deg(self._motor.get_position().value)
        return abs(current_deg - self._target_angle_deg) <= kHoodMotor.REACH_TARGET_ANGLE_ERROR

    def is_hard_stopped(self) -> bool:
        return self._motor.get_reverse_limit().value is ReverseLimitValue.CLOSED_TO_GROUND

    @staticmethod
    def get_angle_by_distance(distance: float) -> float:
        distances, angles = zip(*kShooterData.SHOT_ANGLES)
        return float(np.interp(distance, distances, angles))

    def _apply_position(self) -> None:
        # if self.nt.get("NT Override Enabled"):
        #     target_deg = self.nt.get("NT Override Angle (deg)")
        # else:
        target_deg = self._target_angle_deg + kHoodMotor.TUNER_OFFSET

        self._target_pos_revs = clamp(
            kHoodMotor.to_revs(target_deg),
            kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG),
            kHoodMotor.to_revs(kHoodMotor.MAX_ANGLE_DEG),
        )
        self._motor.set_control(self._pos_req.with_position(self._target_pos_revs))

    def periodic(self) -> None:
        if self.is_hard_stopped() and not self.hard_stopped:
            self._motor.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))
        self.hard_stopped = self.is_hard_stopped()

        if not self._resetting:
            self._apply_position()

        self.nt.set("Hood Position (deg)", round(kHoodMotor.to_deg(self._motor.get_position().value), 1))
        # self.nt.set("Target Angle (deg)", self._target_angle_deg)
        self.editable_pid.periodic()
