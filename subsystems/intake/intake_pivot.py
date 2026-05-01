import wpilib
import commands2
import phoenix6
import phoenix6.controls
from phoenix6 import signals
from phoenix6.signals import MotorAlignmentValue, ReverseLimitValue, ForwardLimitValue

from util.editable_pid import EditablePID
from util.nt_util import NTTable

from constants.intake import kIntakePivot
from constants.canbus import kCANId


class IntakePivot(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.left_pivot_motor  = phoenix6.hardware.TalonFXS(kCANId.intake.LEFT_PIVOT,  "rio")
        self.right_pivot_motor = phoenix6.hardware.TalonFXS(kCANId.intake.RIGHT_PIVOT, "rio")

        self.pivot_cfg = kIntakePivot._CONFIG
        self.left_pivot_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.right_pivot_motor.setNeutralMode(signals.NeutralModeValue.BRAKE)

        if not wpilib.RobotBase.isSimulation():
            self.left_pivot_motor.configurator.apply(self.pivot_cfg)
            self.right_pivot_motor.configurator.apply(self.pivot_cfg)

        self.right_pivot_motor.set_control(
            phoenix6.controls.Follower(kCANId.intake.LEFT_PIVOT, MotorAlignmentValue.OPPOSED)
        )
        self.left_pivot_motor.get_duty_cycle().set_update_frequency(50)

        self._position_request = phoenix6.controls.MotionMagicVoltage(position=0, enable_foc=False, slot=0)

        self._was_at_reverse_limit = False
        self._was_at_foward_limit = False
        self.state = "undeployed"

        self.nt = NTTable("Intake")
        self.nt.float("Pivot Position", 0.0)
        self.nt.float("Target Pivot Position", kIntakePivot.DEPLOYED_POSITION)
        self.nt.float("Ideal Pivot Position", 0.0)
        self.nt.string("State", self.state)

        self.pivot_editable_pid = EditablePID("Intake/PivotPID", self.left_pivot_motor, self.pivot_cfg, use_slot1=True)

    def set_pivot_neutral_mode(self, mode: signals.NeutralModeValue) -> None:
        self.left_pivot_motor.setNeutralMode(mode)
        self.right_pivot_motor.setNeutralMode(mode)

    def set_deployer_position(self, pos: float) -> None:
        self.left_pivot_motor.set_control(self._position_request.with_position(pos))
        # self.nt.set("Ideal Pivot Position", pos)

    def set_deployer_speed(self, speed: float) -> None:
        self.left_pivot_motor.set(speed)

    def set_internal_deployer_position(self, pos: float) -> None:
        self.left_pivot_motor.set_position(pos)
        # self.nt.set("Ideal Pivot Position", 0)

    def is_at_forward_limit(self) -> bool:
        return self.left_pivot_motor.get_forward_limit().value is ForwardLimitValue.CLOSED_TO_GROUND

    def is_at_reverse_limit(self) -> bool:
        return self.left_pivot_motor.get_reverse_limit().value is ReverseLimitValue.CLOSED_TO_GROUND

    def is_any_motor_not_retracted(self) -> bool:
        return (
            self.left_pivot_motor.get_reverse_limit().value  is ReverseLimitValue.OPEN
            or self.right_pivot_motor.get_reverse_limit().value is ReverseLimitValue.OPEN
        )

    def periodic(self) -> None:
        at_reverse_limit = self.is_at_reverse_limit()
        if at_reverse_limit and not self._was_at_reverse_limit:
            self.set_internal_deployer_position(0)
        self._was_at_reverse_limit = at_reverse_limit
        
        at_foward_limmit = self.is_at_forward_limit()
        if at_foward_limmit and not self._was_at_foward_limit:
            self.set_internal_deployer_position(kIntakePivot.DEPLOYED_POSITION)
        self._was_at_foward_limit = at_foward_limmit

        # kIntakePivot.DEPLOYED_POSITION = self.nt.get("Target Pivot Position")

        # self.nt.set("Pivot Position", self.left_pivot_motor.get_position().value)
        # self.nt.set("State", self.state)
        # self.pivot_editable_pid.periodic()
