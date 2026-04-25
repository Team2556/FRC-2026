import math

import commands2

from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.units import rotationsToRadians
from wpimath.kinematics import ChassisSpeeds

from wpilib import DriverStation, Timer

from util.flip_util import FlipUtil
from util.math_helpers import clamp
from util.nt_util import NTTable

from subsystems.drivetrain import drivetrain

from constants.path.key_poses import kPath

class DriveToASpot(commands2.Command):
    def __init__(
        self, 
        subsystem: drivetrain.SwerveDriveTrain, 
        target_pose : Pose2d | tuple = Pose2d(),
        max_speed : float = kPath.default_path_speed, 
        max_rps : float = 0.75, 
        end_tolerance : float = 0.3,
        end_rotation_tolerance : float = 0.4,
        goal_end_velocity : float = 0.0,
    ) -> None:
        """
        Command that makes the robot go to a location. Ignores the fact that walls exist
        so only use this if you know there are no walls in the way (e.g, precise movements)

        :param subsystem:               Drive subsystem with a set_control command that tells robot to move
        :type subsystem:                SwerveSubsystem
        :param max_speed:               Max speed of robot when going to pose
        :type max_speed:                float (m/s)
        :param max_rps:                 Max revolutions per second of robot when going to pose
        :type max_rps:                  float (rps)
        :param target_pose:             Target pose of the robot (set x=0, y=0 to ignore moving to pose and set rotation=0
                                        to ignore rotation)
        :type target_pose:              Pose2d
        :param end_tolerance:           Command can end if it's estimated pose is within this distance of the target pose
        :type end_tolerance:            float (meters)
        :param end_rotation_tolerance:  Command can end if it's estimated rotation is within this distance of the target rotation
        :type end_rotation_tolerance:   float (radians)
        :param goal_end_velocity:       Target velocity for robot to finish when it reaches that pose
                                        (will make the command go faster if this is increased)
        :type goal_end_velocity:        float (m/s)
        """
        
        super().__init__()
         
        self.drivetrain : drivetrain.SwerveDriveTrain = subsystem
        
        self.max_speed = max_speed
        self.max_radians_per_second = rotationsToRadians(max_rps)
        self.smoothing_radius = kPath.smoothing_radius_auto
        
        self.blue_alliance_target_pose = target_pose
        
        self.end_tolerance : float = end_tolerance
        self.end_rotation_tolerance : float = end_rotation_tolerance
        self.goal_end_velocity : float = goal_end_velocity
        
        self.addRequirements(self.drivetrain)
        
        self.pose_estimate = Pose2d()
        
        self.override_speed : float | None = None
        self.override_rps : float | None = None
        self.override_smoothing_radius : float | None = None
        self.override_max_acceleration : float | None = None
        
        self.is_part_of_sequence = False
        self.is_lock_command = False
        self.do_closest_180 = False
        self._nt = NTTable("Autonomous")
        
        self.timer = Timer()
        self.max_time = 0
        
        self.reset_variables()
        
    def initialize(self):
        self.timer.stop()
        self.timer.reset()
        self.reset_variables()
    
    def reset_variables(self):
        self.timer.stop()
        self.timer.reset()

        self.is_within_distance = False
        self.is_within_rotation = False
        
        self.current_velocity = self.get_robot_velocity()
        
        self.command_weight = 1.0
        self.next_command_velocity : Pose2d = Pose2d()
        
        # Create target pose with all the modifier stuff
        if type(self.blue_alliance_target_pose) is tuple:
            if kPath.MIRROR_REVERSE_PATHS and DriverStation.isAutonomous():
                self.target_pose = self.blue_alliance_target_pose[1]
            else:
                self.target_pose = self.blue_alliance_target_pose[0]
        else:
            self.target_pose = self.blue_alliance_target_pose
        
        self.target_pose = FlipUtil.fieldPose(self.target_pose)
        
        if kPath.MIRROR_REVERSE_PATHS and DriverStation.isAutonomous():
            self.target_pose = FlipUtil.mirrorPose(self.target_pose)
        
        if self.is_lock_command: # Override pose to current pose if lock command
            self.target_pose = self.drivetrain.get_state().pose
        
        # Extra auto stuff
        if DriverStation.isAutonomous():
            self.smoothing_radius = kPath.smoothing_radius_auto
        else:
            self.smoothing_radius = kPath.smoothing_radius_teleop
        
        if not self.override_speed == None:
            self.max_speed = self.override_speed
        if not self.override_rps == None:
            self.max_radians_per_second = rotationsToRadians(self.override_rps)
        if not self.override_smoothing_radius == None:
            self.smoothing_radius = self.override_smoothing_radius
    
    def update_pose_estimate(self):
        self.pose_estimate = self.drivetrain.get_state().pose
    
    def get_robot_velocity(self):
        return Pose2d(
            self.drivetrain.get_state().velocity.X(),
            self.drivetrain.get_state().velocity.Y(),
            self.drivetrain.get_state().velocity.rotation()
        )
    
    def calculate_velocity(self) -> Pose2d:
        '''Get target pose velocity to get to target pose spot'''
        
        self.update_pose_estimate()
        
        if not self.is_within_distance:
            max_velocity = self.calculate_translation()
        else:
            max_velocity = Translation2d()
        
        target_angular_velocity = self.calcutate_angular_velocity()
        
        return Pose2d(max_velocity, target_angular_velocity)
    
    def calculate_translation(self) -> Translation2d:
        
        distance = Translation2d(
            self.target_pose.X() - self.pose_estimate.X(),
            self.target_pose.Y() - self.pose_estimate.Y()
        )
        
        linear_distance = distance.norm()
        
        if not self.timer.isRunning() and self.max_speed > 0:
            self.max_time = (linear_distance / self.max_speed) * 2.2 # <-- time to wait to skip path multiplier
            self.timer.start()
        
        if self.override_max_acceleration: # TODO maybe fix override acceleration logic it's kinda copied
            max_acceleration = self.override_max_acceleration
        else:
            max_acceleration = kPath.max_translational_acceleration
        self.slow_distance = abs(self.max_speed ** 2) / (max_acceleration * 2)
        
        # If robot is within "slow distance" then make robot move slower
        target_speed = self.max_speed
        if linear_distance < self.slow_distance:
            progress_ratio = linear_distance / self.slow_distance
            target_speed = (target_speed - self.goal_end_velocity) * progress_ratio + self.goal_end_velocity
        
        return Translation2d(target_speed, 0).rotateBy(distance.angle())
        
    
    def calcutate_angular_velocity(self) -> Rotation2d:
        
        if self.do_closest_180:
            rotation_offset_straight = Rotation2d().relativeTo(self.pose_estimate.rotation())
            rotation_offset_flipped = Rotation2d(math.pi).relativeTo(self.pose_estimate.rotation())
            if abs(rotation_offset_straight.radians()) < abs(rotation_offset_flipped.radians()):
                rotation_offset = rotation_offset_straight
            else:
                rotation_offset = rotation_offset_flipped
        else:
            rotation_offset = (self.target_pose.rotation().relativeTo(self.pose_estimate.rotation()))
        
        # Probably correct calculations 
        # TODO redo this once sequence path is fixed maybe
        current_speed = self.drivetrain.get_state().velocity.translation().norm() # Maybe need this
        rotation_slow_distance = (self.max_radians_per_second) ** 2 / kPath.max_rotational_acceleration
        
        rotation_direction = (-1 if rotation_offset.radians() < 0 else 1)
        
        if abs(rotation_offset.radians()) > rotation_slow_distance:
            target_angular_velocity = self.max_radians_per_second * rotation_direction
        else:
            rotation_ratio = rotation_direction * abs(rotation_offset.radians() / rotation_slow_distance) ** 1.5
            target_angular_velocity = self.clamp_speed(self.max_radians_per_second * rotation_ratio, self.max_radians_per_second)

        return Rotation2d(target_angular_velocity)
    
    def clamp_speed(self, value, max_speed):
        max_speed = abs(max_speed)
        return max(min(value, max_speed), max_speed * -1)
    
    def execute(self):
        self.target_velocity = self.calculate_velocity()

        field_speeds = ChassisSpeeds(
            (self.target_velocity * self.command_weight).X() + self.next_command_velocity.X(),
            (self.target_velocity * self.command_weight).Y() + self.next_command_velocity.Y(),
            self.target_velocity.rotation().radians()
        )

        # calculate_velocity returns field-relative speeds; run_velocity expects robot-relative
        robot_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            field_speeds, self.drivetrain.get_rotation()
        )
        self.drivetrain.run_velocity(robot_speeds, ignore_modifiers=True)
    
    def isFinished(self):
        
        # Timer stuff doing the override if it's taking too long
        # if self.timer.get() >= self.max_time and self.max_speed > 0:
        #     return True
        
        # Translation stuff
        distance_from_target = self.pose_estimate.translation().distance(self.target_pose.translation())
        self.is_within_distance = distance_from_target < self.end_tolerance
        
        # Rotation stuff
        distance_from_target_rotation = -(self.target_pose.rotation().relativeTo(self.pose_estimate.rotation())).radians()
        self.is_within_rotation = abs(distance_from_target_rotation) < self.end_rotation_tolerance
        if not self.drivetrain._align_rotation == None: # Extra thing so command ignores rotation check if aligning to shoot
            self.is_within_rotation = True
        
        return self.is_within_distance and self.is_within_rotation
    
    def end(self, interrupted: bool):
        if not self.is_part_of_sequence:
            self.drivetrain.stop()
        self.timer.stop()
        self.timer.reset()
    
    def get_distance_progress(self):
        self.update_pose_estimate()
        return Translation2d(
            self.pose_estimate.X() - self.target_pose.X(),
            self.pose_estimate.Y() - self.target_pose.Y()
        ).norm()
    
    def with_target_pose(self, value): self.target_pose = value; return self
    def with_max_speed(self, value): self.max_speed = value; return self
    def with_max_rps(self, value): self.max_radians_per_second = rotationsToRadians(value); return self
    def with_end_tolerance(self, value): self.end_tolerance = value; return self
    def with_end_rotation_tolerance(self, value): self.end_rotation_tolerance = value; return self
    def with_goal_end_velocity(self, value): self.goal_end_velocity = value; return self
    
    def with_override_speed(self, new_speed): self.override_speed = new_speed; return self
    def with_override_rps(self, new_rps): self.override_rps = new_rps; return self
    def with_override_smoothing_radius(self, new_smooth_radius): self.override_smoothing_radius = new_smooth_radius; return self
    def with_override_max_acceleration(self, new_max_acceleration): self.override_max_acceleration = new_max_acceleration; return self
    
    def with_closest_180(self): self.do_closest_180 = True; return self
    
    def with_precise_values(self):
        self.max_speed = 2.7
        self.max_rps = 0.5
        self.end_tolerance = 0.2
        self.end_rotation_tolerance = 0.1
        self.goal_end_velocity = 0.03
        return self

    def with_sequence_pose_values(self):
        self.is_part_of_sequence = True
        
        self.end_tolerance = 0.0
        self.end_rotation_tolerance = 0.0
        
        if self.override_max_acceleration:
            max_acceleration = self.override_max_acceleration
        else:
            max_acceleration = kPath.max_translational_acceleration
        
        # Shoutout to third kinematic equation
        self.slow_distance = abs(self.max_speed ** 2) / (max_acceleration * 2)
        self.goal_end_velocity = (self.max_speed ** 2 - 2 * max_acceleration * self.slow_distance) ** 0.5
        
        self.smoothing_time = self.smoothing_radius / self.max_speed * kPath.smoothing_time_multiplier
        
        return self

    def with_lock_values(self):
        self.is_lock_command = True
        
        self.end_tolerance = 0
        
        self.max_speed = 4
        self.goal_end_velocity = 0
        self.smoothing_radius = 0.25
        
        return self