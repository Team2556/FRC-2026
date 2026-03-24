import commands2

from phoenix6 import swerve
from subsystems.drivetrain import drivetrain
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
import math
from util.flip_util import FlipUtil
from constants.key_poses import kPath
from wpilib import DriverStation 

from wpimath.units import rotationsToRadians

class DriveToASpot(commands2.Command):
    def __init__(
        self, 
        subsystem: drivetrain.SwerveDriveTrain, 
        target_pose : Pose2d = Pose2d(),
        max_speed : float = 1.0, 
        max_rps : float = 0.75, 
        end_tolerance : float = 0.1,
        end_rotation_tolerance : float = 0.1,
        goal_end_velocity : float = 0.0,
        slow_distance : float = 0.5
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
        :param slow_distance:           Robot will start slowing down to end velocity this distance away from target pose
        :type slow_distance:            float (meters)
        """
        
        super().__init__()
         
        self.drivetrain : drivetrain.SwerveDriveTrain = subsystem
        
        self.max_speed = max_speed
        self.max_radians_per_second = rotationsToRadians(max_rps)
        
        self.blue_alliance_target_pose = target_pose
        
        self.end_tolerance : float = end_tolerance
        self.end_rotation_tolerance : float = end_rotation_tolerance
        self.goal_end_velocity : float = goal_end_velocity
        self.slow_distance: float = slow_distance
        
        self.addRequirements(self.drivetrain)
        
        self.pose_estimate = Pose2d()
        
        self.do_override_speed = False
        self.parallel_command = None
        
        self.reset_variables()
        
    def initialize(self):
        if self.parallel_command:
            self.parallel_command.schedule()
        self.reset_variables()
    
    def reset_variables(self):
        self.is_within_distance = False
        self.is_within_rotation = False
        self.is_within_slow_distance = False
        
        self.current_velocity = self.get_robot_velocity()
        
        self.command_weight = 1.0
        self.next_command_velocity : Pose2d = Pose2d()
        
        self.target_pose = FlipUtil.fieldPose(self.blue_alliance_target_pose)
        
    def update_pose_estimate(self):
        # NOTE: change self.drivetrain.get_state().pose to the actual 
        # pose estimating function once I can use better pose estimate
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
            self.pose_estimate.X() - self.target_pose.X(),
            self.pose_estimate.Y() - self.target_pose.Y()
        )
        
        linear_distance = distance.norm()
        
        target_speed = self.max_speed
        
        # If robot is within "slow distance" then make robot move slower
        if linear_distance < self.slow_distance:
            self.is_within_slow_distance = True
            progress_ratio = linear_distance / self.slow_distance
            target_speed = (target_speed - self.goal_end_velocity) * progress_ratio + self.goal_end_velocity
        else:
            self.is_within_slow_distance = False
        
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return Translation2d(target_speed, 0).rotateBy(distance.angle())
        else:
            return Translation2d(target_speed, 0).rotateBy(distance.angle() + Rotation2d(math.pi))
    
    def calcutate_angular_velocity(self) -> Rotation2d:
        
        rotation_offset = self.target_pose.rotation().relativeTo(self.pose_estimate.rotation())
        rotation_offset = rotation_offset.rotateBy(Rotation2d(math.pi))
        
        # P value for changing rotation
        # This actually just turns into an increase number to change "tolerance" it's kinda buggy maybe
        # do it in a different way if it needs to be better
        rotation_multiplier = 8
        
        target_angular_velocity = self.clamp_speed(
            rotation_offset.radians() * rotation_multiplier, self.max_radians_per_second
        )
        
        return Rotation2d(target_angular_velocity)
    
    def clamp_speed(self, value, max_speed):
        max_speed = abs(max_speed)
        return max(min(value, max_speed), max_speed * -1)
    
    def execute(self):
        self.target_velocity = self.calculate_velocity()
        
        self.target_velocity = Pose2d(
            (self.target_velocity * self.command_weight).X() + self.next_command_velocity.X(),
            (self.target_velocity * self.command_weight).Y() + self.next_command_velocity.Y(),
            self.target_velocity.rotation()
        )
        
        self.drivetrain.drive_with_values(
            self.target_velocity.X(),
            self.target_velocity.Y(),
            self.target_velocity.rotation().radians()
        )
    
    def isFinished(self):
        # Translation stuff
        distance_from_target = self.pose_estimate.translation().distance(self.target_pose.translation())
        self.is_within_distance = distance_from_target < self.end_tolerance
        
        # Rotation stuff
        distance_from_target_rotation = self.target_pose.rotation().relativeTo(self.pose_estimate.rotation())
        distance_from_target_rotation = distance_from_target_rotation.rotateBy(Rotation2d(math.pi)).radians()
        self.is_within_rotation = abs(distance_from_target_rotation) < self.end_rotation_tolerance
        
        return self.is_within_distance and self.is_within_rotation
    
    def end(self, interrupted: bool):
        self.drivetrain.stop()
    
    def get_distance_progress(self):
        self.update_pose_estimate()
        return Translation2d(
            self.pose_estimate.X() - self.target_pose.X(),
            self.pose_estimate.Y() - self.target_pose.Y()
        ).norm()
    
    def with_parallel_command(self, command : commands2.Command):
        '''
        If you give DriveToASpot a parallel command with this function, said parallel command 
        will be scheduled at the same time this command is scheduled
        '''
        self.parallel_command = command
        return self
    
    def with_target_pose(self, value): self.target_pose = value; return self
    def with_max_speed(self, value): self.max_speed = value; return self
    def with_max_rps(self, value): self.max_rps = value; return self
    def with_end_tolerance(self, value): self.end_tolerance = value; return self
    def with_end_rotation_tolerance(self, value): self.end_rotation_tolerance = value; return self
    def with_goal_end_velocity(self, value): self.goal_end_velocity = value; return self
    def with_slow_distance(self, value): self.slow_distance = value; return self
    
    def with_precise_values(self):
        self.max_speed = 2.7
        self.max_rps = 0.5
        self.end_tolerance = 0.1
        self.end_rotation_tolerance = 0.001
        self.goal_end_velocity = 0.01
        self.slow_distance = 0.6
        return self

    def with_sequence_pose_values(self):
        self.max_rps = 0.5
        self.end_tolerance = 0.0
        self.end_rotation_tolerance = 0.0
        self.slow_distance = self.max_speed * kPath.percent_slow_distance_proportional_to_max_speed_for_sequence_path
        self.goal_end_velocity = self.max_speed * kPath.path_transition_slow_multiplier
        return self

    def with_override_speed(self, new_speed):
        self.do_override_speed = True
        self.better_max_speed = new_speed
        return self