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
        self.smoothing_radius = 0.25
    
    def reset_variables(self):
        super().reset_variables()
        self.target_pose = self.drivetrain.get_state().pose
    
    def execute(self):
        linear_distance = Translation2d(
            self.target_pose.X() - self.drivetrain.get_state().pose.X(),
            self.target_pose.Y() - self.drivetrain.get_state().pose.Y()
        ).norm()
        
        if linear_distance < kPath.antidefense_lock_radius:
            self.drivetrain.stop_with_brake()
        else:
            super().execute()
            
        print(self.target_pose)