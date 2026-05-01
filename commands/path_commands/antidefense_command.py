from commands.path_commands.drive_to_a_spot import DriveToASpot
from wpimath.geometry import Translation2d
from constants.path.key_poses import kPath

class AntiDefenseCommand(DriveToASpot):
    def __init__(self, drivetrain):
        super().__init__(drivetrain)
        
        self.is_lock_command = True
        
        self.end_tolerance = 0
        
        self.max_speed = 5
        self.goal_end_velocity = 0
        self.slow_distance = 0.25
        
        self.max_rps = 1
    
    def reset_variables(self):
        super().reset_variables()
        self.target_pose = self.drivetrain.get_state().pose
    
    def execute(self):
        linear_distance = Translation2d(
            self.target_pose.X() - self.drivetrain.get_state().pose.X(),
            self.target_pose.Y() - self.drivetrain.get_state().pose.Y()
        ).norm()
        rotation_offset = (self.target_pose.rotation().relativeTo(self.pose_estimate.rotation())).radians()
        
        if linear_distance < kPath.antidefense_lock_radius and rotation_offset < kPath.antidefense_lock_rotation:
            self.drivetrain.stop_with_brake()
        else:
            super().execute()
            