import commands2

from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from util.math import kMath

from commands.path_commands.drive_to_a_spot import DriveToASpot
from commands.path_commands.drive_to_a_spot_sequence import DriveToASpotSequence
from constants.key_poses import kPoses
from constants.field import kHub

from util.flip_util import FlipUtil
from util.robot_zone_checker import RobotZoneChecker
from math import pi
import math

class GoToShootingSpot(commands2.Command):
    def __init__(self, subsystem: SwerveDriveTrain):
        super().__init__()
        
        self.drivetrain : SwerveDriveTrain = subsystem
        
        self.left_path = DriveToASpot(
            self.drivetrain, target_pose=Pose2d(2.0, 2.67, Rotation2d(-0.927))
        ).with_precise_values()
        
        self.right_path = DriveToASpot(
            self.drivetrain, target_pose=Pose2d(2.0, 5.33, Rotation2d(-1.935))
        ).with_precise_values()

    def initialize(self):
        pose = self.drivetrain.get_state().pose
        
        if not RobotZoneChecker.is_in_left_alliance_zone(pose):
            self.left_path.schedule()
        
        if not RobotZoneChecker.is_in_right_alliance_zone(pose):
            self.right_path.schedule()
    
    def calculate_target_yaw(self, pose : Pose2d):
        SHOOTER_OFFSET = Translation2d(-10.432 * kMath.MetersPerInch, 0)  # Meters
        SHOOTER_DIRECTION = -90  # 180 for Reverse
        
        shooter_field_pos = pose.translation() + SHOOTER_OFFSET.rotateBy(
            pose.rotation()
        )
        vector_pointer = kHub.POS.translation() - shooter_field_pos
        target_robot_yaw = math.degrees(
            math.atan2(vector_pointer.Y(), vector_pointer.X())
        )
        return target_robot_yaw - SHOOTER_DIRECTION
    
    def end(self, interrupted):
        # This function is called after the command ends
        # the interrupted variable stores whether or not the command was interuppted or canceled.
        self.left_path.cancel()
        self.right_path.cancel()