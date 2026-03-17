import numpy as np
import wpilib

from commands2 import Subsystem

from wpimath.geometry import Pose2d

from phoenix6.hardware import TalonFXS
from phoenix6.controls import MotionMagicVoltage, DutyCycleOut
from phoenix6.signals import NeutralModeValue, ReverseLimitValue

from util.editable_pid import EditablePID
from util.nt_util import NTTable
from util.math_helpers import distanceFromPose2dtoPose2d

from constants.shooter import kHoodMotor, kShooterData
from constants.canbus import kCANId


class ShooterHood(Subsystem):
    def __init__(self):
        self.hood_motor = TalonFXS(kCANId.shooter.HOOD_CONTROL, "rio")

        if not wpilib.RobotBase.isSimulation():
            self.hood_motor.configurator.apply(kHoodMotor._CONFIG)
            self.hood_motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.position_request = MotionMagicVoltage(position=0, enable_foc=False)
        self.home_request = DutyCycleOut(0)

        self._target_angle_deg = kHoodMotor.HOME_ANGLE_DEG
        self.force_hide = False
        self._was_hard_stopped = False
        self.resetting = False

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.float("Hood Position (deg)", 0.0)
        self.nt.float("Target Angle (deg)", 0.0)
        self.nt.float("Reset Home Speed", kHoodMotor.RESET_HOME_SPEED)
        self.nt.float("Max Hood Position (deg)", kHoodMotor.MAX_ANGLE_DEG)
        self.nt.bool("Override Enabled", kHoodMotor.OVERRIDE_ENABLED)
        self.nt.float("Override Angle (deg)", kHoodMotor.OVERRIDE_ANGLE_DEG)
        self.nt.bool("Force Hide", False)

        self.editable_pid = EditablePID(
            "Shooter/Hood", self.hood_motor, kHoodMotor._CONFIG
        )

        self.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))
        self.robot_zone = "alliance"

    def set_target_angle(self, degrees: float) -> None:
        self._target_angle_deg = degrees
        self._apply_angle()

    def set_speed(self, speed: float) -> None:
        self.hood_motor.set_control(self.home_request.with_output(speed))

    def set_position(self, position_revs: float) -> None:
        max_revs = kHoodMotor.to_revs(self.nt.get("Max Hood Position (deg)"))
        position_revs = min(position_revs, max_revs)
        self._target_position_revs = position_revs
        self.hood_motor.set_control(self.position_request.with_position(position_revs))

    def is_at_angle(self) -> bool:
        current_deg = kHoodMotor.to_deg(self.hood_motor.get_position().value)
        target_deg = kHoodMotor.to_deg(self._target_position_revs)
        return abs(current_deg - target_deg) <= kHoodMotor.REACH_TARGET_ANGLE_ERROR

    def is_hard_stopped(self) -> bool:
        return (
            self.hood_motor.get_reverse_limit().value
            is ReverseLimitValue.CLOSED_TO_GROUND
        )

    def reset(self) -> None:
        self._target_angle_deg = kHoodMotor.HOME_ANGLE_DEG
        self.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))
    
    def toggle_force_hide(self, is_enabled: bool) -> None:
        self.force_hide = is_enabled

    def angle_by_position(self, robot_pose: Pose2d, target_pose: Pose2d) -> None:
        distance_to_target = distanceFromPose2dtoPose2d(robot_pose, target_pose)
        interpolation_distance_data, interpolation_position_data = zip(
            *kShooterData.SHOT_ANGLES
        )
        target_deg = np.interp(
            distance_to_target, interpolation_distance_data, interpolation_position_data
        )
        self.set_target_angle(target_deg)

    def _apply_angle(self) -> None:
        if self.force_hide:
            angle_deg = kHoodMotor.HOME_ANGLE_DEG
        elif self.nt.get("Override Enabled"):
            angle_deg = self.nt.get("Override Angle (deg)")
        elif self.robot_zone == "opposing":
            angle_deg = kHoodMotor.OPPOSING_ANGLE_DEG
        else:
            angle_deg = self._target_angle_deg
            
        self.set_position(kHoodMotor.to_revs(angle_deg))

    def periodic(self) -> None:
        if not self.resetting:
            self._apply_angle()

        hard_stopped = self.is_hard_stopped()
        if hard_stopped and not self._was_hard_stopped:
            self.hood_motor.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))
        self._was_hard_stopped = hard_stopped

        self.nt.set(
            "Hood Position (deg)",
            kHoodMotor.to_deg(self.hood_motor.get_position().value),
        )
        self.nt.set("Target Angle (deg)", kHoodMotor.to_deg(self._target_position_revs))
        self.nt.set("Force Hide", self.force_hide)

        kHoodMotor.RESET_HOME_SPEED = self.nt.get("Reset Home Speed")

        self.editable_pid.periodic()
        
        from wpilib import SmartDashboard
        SmartDashboard.putString("Robot Zone", self.robot_zone)
