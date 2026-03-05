'''
All poses that are important for now
'''

from wpimath.geometry import Pose2d, Rotation2d
from math import pi

class kPath:
    default_path_speed = 3
    default_smoothing_time = 0.5 # Try 15-20% of path speed
    default_smoothing_radius = 0.5 * default_path_speed * default_smoothing_time

class kPoses:
    '''Locations of every pose that has to do with drive_to_a_spot pathfinding'''
    
    alliance_zone_left_trench = Pose2d(3.0, 7.45, Rotation2d(pi/2))
    neutral_zone_left_trench = Pose2d(6.0, 7.3, Rotation2d(pi/2))
    
    alliance_zone_right_trench = Pose2d(3.0, 0.55, Rotation2d(-pi/2))
    neutral_zone_right_trench = Pose2d(6.0, 0.7, Rotation2d(-pi/2))
    
    alliance_zone_right_bump = Pose2d(3.0, 2.5, Rotation2d(-pi/2))
    neutral_zone_right_bump = Pose2d(6.0, 2.5, Rotation2d(-pi/2))
    
    alliance_zone_left_bump = Pose2d(3.0, 5.52, Rotation2d(pi/2))
    neutral_zone_left_bump = Pose2d(6.0, 5.52, Rotation2d(pi/2))
    
    opposing_zone_left_trench = Pose2d(13.2, 7.3, Rotation2d(pi/2))
    opposing_zone_right_trench = Pose2d(13.2, 0.7, Rotation2d(-pi/2))
    
    
    # Temporary Poses
    start_shooting_point = Pose2d(2.345, 2.330, Rotation2d(0.279))
    bottom_climb_test = Pose2d(0.841, 2.747, Rotation2d(0))
    
    test_start_spot = Pose2d(8.328, 5.974, Rotation2d(pi))
    test_end_spot = Pose2d(9.677, 5.874, Rotation2d(pi))