import numpy as np

from commands2 import Subsystem

from wpimath.geometry import Pose2d

from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionVoltage, DutyCycleOut
from phoenix6.signals import NeutralModeValue, ForwardLimitValue

from util.editable_pid import EditablePID
from util.nt_util import NTTable
from util.math_helpers import distanceFromPose2dtoPose2d

from constants.shooter import kHoodMotor, kShooterData


# Currently not tested yet. Also there is no "shooter hood" command as all the controlling for
# this subsystem will be in the hub align command (eventually)
class ShooterHood(Subsystem):
    def __init__(self):
        self.hood_motor = TalonFX(kHoodMotor.CAN_ID, "rio")
        self.hood_motor.configurator.apply(kHoodMotor._CONFIG)
        self.hood_motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.position_voltage = PositionVoltage(0)
        self.home_voltage = DutyCycleOut(0)

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.float("Hood Position", 0.0)
        self.nt.float("Reset Home Speed", kHoodMotor.RESET_HOME_SPEED)
        self.nt.float("Increment Amount", kHoodMotor.INCREMENT_AMOUNT)
        self.editable_pid = EditablePID("Hood", self.hood_motor, kHoodMotor._CONFIG)

    def set_speed(self, speed):
        self.hood_motor.set_control(self.home_voltage.with_output(speed))

    def increment(self, mult):
        self.hood_motor.set_control(
            self.hood_motor.get() + ((kHoodMotor.INCREMENT_AMOUNT / 20) * mult)
        )

    def set_position(self, position):
        self.hood_motor.set_control(self.position_voltage.with_position(position))

    def is_hard_stopped(self):
        return (
            self.hood_motor.get_reverse_limit().value
            is ForwardLimitValue.CLOSED_TO_GROUND
        )

    def reset(self):
        self.set_position(0)
    
    def angle_by_position(self, robot_pose: Pose2d, target_pose: Pose2d) -> None:
        distance_to_target = distanceFromPose2dtoPose2d(robot_pose, target_pose)
        
        interpolation_distance_data, interpolation_position_data = zip(*kShooterData.SHOT_ANGLES)
        target_hood_position = np.interp(distance_to_target, interpolation_distance_data, interpolation_position_data)
        
        self.set_position(target_hood_position)

    def periodic(self):
        if self.is_hard_stopped():
            self.hood_motor.set_position(0)

        self.nt.set("Hood Angle", self.hood_motor.get())
        kHoodMotor.RESET_HOME_SPEED = self.nt.get("Reset Home Speed")
        kHoodMotor.INCREMENT_AMOUNT = self.nt.get("Increment Amount")

        self.editable_pid.periodic()
