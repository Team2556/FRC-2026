from subsystems.drivetrain.drivetrain import SwerveDriveTrain
from commands.path_commands.drive_to_a_spot import DriveToASpot
from wpimath.geometry import Pose2d
from typing import Callable

class PathfindInstruction:
    def __init__(
        self, 
        zone_condition : Callable,
        *target_poses : Pose2d
        ):
        self.zone_condition = zone_condition
        self.target_poses = target_poses
    
    def should_call_instruction(self, current_pose):
        # self.zone_condition should be a RobotZoneChecker
        return self.zone_condition(current_pose)

    def get_poses(self):
        return self.target_poses

class PathfindToStart:
    def __init__(
        self, 
        drivetrain : SwerveDriveTrain,
        *instructions : PathfindInstruction,
        ):
        self.instructions = instructions
        self.drivetrain = drivetrain
    
    def generate_instructions(self):
        '''Returns a list of DriveToASpot commands that the robot should do to get to the ideal starting spot'''
        current_pose = self.drivetrain.get_state().pose
        for instruction in self.instructions:
            if instruction.should_call_instruction(current_pose):
                commands = []
                for pose in instruction.get_poses():
                    commands.append(DriveToASpot(self.drivetrain, target_pose=pose))
                return commands
        return []