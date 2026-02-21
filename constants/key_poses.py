'''
All poses that are important for now
'''

from wpimath.geometry import Pose2d, Rotation2d
from math import pi

class kPath:
    general_path_speed = 1

class kPoses:
    behind_trench_bottom = Pose2d(6.0, 0.65, Rotation2d(0))
    behind_trench_top = Pose2d(6.0, 7.35, Rotation2d(0))
    
    alliance_zone_bottom = Pose2d(3.0, 0.55, Rotation2d(0))
    alliance_zone_top = Pose2d(3.0, 7.45, Rotation2d(0))
    
    opposing_zone_bottom = Pose2d(13.2, 0.65, Rotation2d(0))
    opposing_zone_top = Pose2d(13.2, 7.35, Rotation2d(0))
    
    start_shooting_point = Pose2d(2.345, 2.330, Rotation2d(0.279))
    bottom_climb_test = Pose2d(0.841, 2.747, Rotation2d(0))
    