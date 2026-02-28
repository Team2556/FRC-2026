'''
All poses that are important for now
'''

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from math import pi

class kPath:
    default_path_speed = 3
    default_smoothing_time = 0.5 # Try 15-20% of path speed
    default_smoothing_radius = 0.5 * default_path_speed * default_smoothing_time

class kPoses:
    behind_trench_bottom = Pose2d(6.0, 0.7, Rotation2d(0))
    behind_trench_top = Pose2d(6.0, 7.3, Rotation2d(0))
    
    alliance_zone_bottom = Pose2d(3.0, 0.55, Rotation2d(0))
    alliance_zone_top = Pose2d(3.0, 7.45, Rotation2d(0))
    
    opposing_zone_bottom = Pose2d(13.2, 0.7, Rotation2d(0))
    opposing_zone_top = Pose2d(13.2, 7.3, Rotation2d(0))
    
    
    start_shooting_point = Pose2d(2.345, 2.330, Rotation2d(0.279))
    bottom_climb_test = Pose2d(0.841, 2.747, Rotation2d(0))
    
    
    test_start_spot = Pose2d(8.328, 5.974, Rotation2d(pi))
    test_end_spot = Pose2d(9.677, 5.874, Rotation2d(pi))

class kTranslations:
    field_x = 16.54
    field_y = 8.07
    
    bottom_left = Translation2d(0, 0)
    bottom_right = Translation2d(16.54, 0)
    top_left = Translation2d(0, 8.07)
    top_right = Translation2d(16.54, 8.07)
    
    middle_left = Translation2d(0, 4.03)
    middle_right = Translation2d(16.54, 4.03)
    
    alliance_hub = Translation2d(4, 4)
    opposing_alliance_hub = Translation2d(12, 4)
    
    bottom_left_neutral_zone = Translation2d(4, 0)
    bottom_right_neutral_zone = Translation2d(12, 0)
    top_left_neutral_zone = Translation2d(4, 8.07)
    top_right_neutral_zone = Translation2d(12, 8.07)
    
    midpoint = Translation2d(4.03, 8.27)
    