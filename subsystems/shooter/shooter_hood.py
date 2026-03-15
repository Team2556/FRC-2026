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

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.float("Hood Position", 0.0)
        self.nt.float("Ideal Hood Position", 0.0)
        self.nt.float("Reset Home Speed", kHoodMotor.RESET_HOME_SPEED)
        self.nt.float("Increment Amount", kHoodMotor.INCREMENT_AMOUNT)

        self._was_hard_stopped = False

        self.editable_pid = EditablePID(
            "Shooter/Hood", self.hood_motor, kHoodMotor._CONFIG
        )

    def set_speed(self, speed):
        self.hood_motor.set_control(self.home_request.with_output(speed))

    def increment(self, mult):
        self.set_position(
            self.hood_motor.get_position().value
            + ((kHoodMotor.INCREMENT_AMOUNT / 20) * mult)
        )

    def set_position(self, position):
        self.hood_motor.set_control(self.position_request.with_position(position))
        self.nt.set("Ideal Hood Position", position)

    def is_hard_stopped(self):
        return (
            self.hood_motor.get_reverse_limit().value
            is ReverseLimitValue.CLOSED_TO_GROUND
        )

    def reset(self):
        self.set_position(0)
        self.nt.set("Ideal Home Position", 0)

    def angle_by_position(self, robot_pose: Pose2d, target_pose: Pose2d) -> None:
        distance_to_target = distanceFromPose2dtoPose2d(robot_pose, target_pose)

        interpolation_distance_data, interpolation_position_data = zip(
            *kShooterData.SHOT_ANGLES
        )
        target_hood_position = np.interp(
            distance_to_target, interpolation_distance_data, interpolation_position_data
        )

        self.set_position(target_hood_position)

    def periodic(self):
        hard_stopped = self.is_hard_stopped()
        if hard_stopped and not self._was_hard_stopped:
            self.hood_motor.set_position(0)
        self._was_hard_stopped = hard_stopped

        self.nt.set("Hood Position", self.hood_motor.get_position().value)
        kHoodMotor.RESET_HOME_SPEED = self.nt.get("Reset Home Speed")
        kHoodMotor.INCREMENT_AMOUNT = self.nt.get("Increment Amount")

        self.editable_pid.periodic()
