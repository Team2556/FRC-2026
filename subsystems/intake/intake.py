import wpilib
from wpilib import SmartDashboard, DigitalInput

import commands2

import phoenix6
import phoenix6.controls
from phoenix6 import signals
from phoenix6.signals import MotorAlignmentValue

from util.editable_pid import EditablePID
from util.nt_util import NTTable

from constants.intake import kIntakeRoller, kIntakePivot
from constants.canbus import kCANId


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.left_pivot_motor = phoenix6.hardware.TalonFXS(
            kCANId.intake.LEFT_PIVOT, "rio"
        )
        self.right_pivot_motor = phoenix6.hardware.TalonFXS(
            kCANId.intake.RIGHT_PIVOT, "rio"
        )
        self.roller_motor = phoenix6.hardware.TalonFX(kCANId.intake.ROLLER, "rio")

        self.pivot_cfg = kIntakePivot._CONFIG
        self.roller_cfg = kIntakeRoller._CONFIG
        self.roller_cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

        self.right_pivot_motor.set_control(
            phoenix6.controls.Follower(
                kCANId.intake.LEFT_PIVOT, MotorAlignmentValue.OPPOSED
            )
        )

        if not wpilib.RobotBase.isSimulation():
            self.left_pivot_motor.configurator.apply(self.pivot_cfg)
            self.right_pivot_motor.configurator.apply(self.pivot_cfg)
            self.roller_motor.configurator.apply(self.roller_cfg)

        self.position_request = phoenix6.controls.MotionMagicVoltage(
            position=0, enable_foc=False, slot=0
        )
        self.velocity_request = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self._was_at_reverse_limit = False
        self.state = "undeployed"

        self.nt = NTTable("Intake")
        self.nt.float("Pivot Position", 0.0)
        self.nt.float("Target Pivot Position", kIntakePivot.DEPLOYED_POSITION)
        self.nt.float("Target Pivot Speed", kIntakePivot.DEPLOYED_SPEED) # Temporary hopefully
        self.nt.float("Ideal Pivot Position", 0.0)
        self.nt.int("Current Slot", 0)
        self.nt.float("Roller RPM", 0.0)
        self.nt.float("Target Roller RPM", kIntakeRoller.TARGET_RPM)
        self.nt.float("Ideal Roller RPM", 0.0)
        self.nt.string("State", self.state)

        self.pivot_editable_pid = EditablePID(
            "Intake/PivotPID",
            self.left_pivot_motor,
            self.pivot_cfg,
            use_slot1=True,
        )
        self.roller_editable_pid = EditablePID(
            "Intake/RollerPID", self.roller_motor, self.roller_cfg
        )

    def set_deployer_position(self, pos):
        self.left_pivot_motor.set_control(self.position_request.with_position(pos))
        self.right_pivot_motor.set_control(self.position_request.with_position(-pos))
        self.nt.set("Ideal Pivot Position", pos)
    
    def set_deployer_speed(self, speed):
        self.left_pivot_motor.set(speed)
        self.right_pivot_motor.set_control(speed * -1)

    def set_internal_deployer_position(self, pos):
        self.left_pivot_motor.set_position(pos)
        self.right_pivot_motor.set_position(-pos)
        self.nt.set("Ideal Pivot Position", 0)

    def change_deployer_slot(self, slot=0):
        self.left_pivot_motor.set_control(self.position_request.with_slot(slot))
        self.right_pivot_motor.set_control(self.position_request.with_slot(slot))
        self.nt.set("Current Slot", slot)

    def set_roller_speed(self, rpm):
        self.roller_motor.set_control(self.velocity_request.with_velocity(rpm / 60))
        self.nt.set("Ideal Roller RPM", rpm)

    def stop_roller(self):
        # NeutralOut fully disables the motor; VelocityVoltage(0) leaves the PID integrator running
        # which causes sporadic twitching _FCC_
        self.roller_motor.set_control(phoenix6.controls.NeutralOut())
        self.nt.set("Ideal Roller RPM", 0)

    def periodic(self):
        at_reverse_limit = (
            self.left_pivot_motor.get_reverse_limit().value
            is signals.ReverseLimitValue.CLOSED_TO_GROUND
        )
        if at_reverse_limit and not self._was_at_reverse_limit:
            self.set_internal_deployer_position(0)
            
        self._was_at_reverse_limit = at_reverse_limit

        self.nt.set("Pivot Position", self.left_pivot_motor.get_position().value)
        self.nt.set("Roller RPM", self.roller_motor.get_velocity().value * 60)
        self.nt.set("State", self.state)
        kIntakePivot.DEPLOYED_POSITION = self.nt.get("Target Pivot Position")
        # Temporary hopefully
        kIntakePivot.DEPLOYED_SPEED = self.nt.get("Target Pivot Speed")
        kIntakeRoller.TARGET_RPM = self.nt.get("Target Roller RPM")

        self.pivot_editable_pid.periodic()
        self.roller_editable_pid.periodic()
