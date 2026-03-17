'''
All poses that are important for all the paths.
These aren't known locations and should be tuned a bunch (unlike field.py)
'''

from wpimath.geometry import Pose2d, Rotation2d
from math import pi
from constants.field import kField

class kPath:
    default_path_speed = 3
    default_smoothing_time = 0.5 # Try 15-20% of path speed maybe
    default_smoothing_radius = 0.6 * default_path_speed * default_smoothing_time
    
    # High quality variable name
    percent_slow_distance_proportional_to_max_speed_for_sequence_path = 0.25
    path_transition_slow_multiplier = 0.5 # In percent from 0 to 1

class kPoses:
    '''Locations of every pose that has to do with drive_to_a_spot pathfinding'''
    
    from_edge = 0.67
    from_edge_neutral = 0.73
    
    alliance_zone_left_trench = Pose2d(3.0, kField.WIDTH - from_edge, Rotation2d(pi))
    neutral_zone_left_trench = Pose2d(6.0, kField.WIDTH - from_edge_neutral, Rotation2d(pi))
    opposing_zone_left_trench = Pose2d(13.2, kField.WIDTH - from_edge_neutral, Rotation2d(pi))
    
    alliance_zone_right_trench = Pose2d(3.0, from_edge, Rotation2d(pi))
    neutral_zone_right_trench = Pose2d(6.0, from_edge_neutral, Rotation2d(pi))
    opposing_zone_right_trench = Pose2d(13.2, from_edge_neutral, Rotation2d(pi))
    
    alliance_zone_right_bump = Pose2d(3.0, 2.5, Rotation2d())
    neutral_zone_right_bump = Pose2d(6.0, 2.5, Rotation2d())
    opposing_zone_right_bump = Pose2d(13.2, 2.5, Rotation2d())
    
    alliance_zone_left_bump = Pose2d(3.0, kField.WIDTH - 2.5, Rotation2d())
    neutral_zone_left_bump = Pose2d(6.0, kField.WIDTH - 2.5, Rotation2d())
    opposing_zone_left_bump = Pose2d(13.2, kField.WIDTH - 2.5, Rotation2d())
    
    # Temporary Poses
    simple_right0 = Pose2d(3.7, from_edge, Rotation2d(pi))
    simple_right1 = Pose2d(2.7, from_edge, Rotation2d((4*pi)/3))
    simple_right2 = Pose2d(2.7, 1.3, Rotation2d((4*pi)/3))
    
    neutral_grab_right0 = Pose2d(4.3, from_edge, Rotation2d())
    neutral_grab_right1 = Pose2d(7.5, from_edge, Rotation2d(-pi/2))
    neutral_grab_right2 = Pose2d(7.8, 1.0, Rotation2d(-pi/2))
    neutral_grab_right3 = Pose2d(7.8, 3.5, Rotation2d(-pi/2))
    neutral_grab_right4 = Pose2d(6.0, 0.6, Rotation2d(pi))
    neutral_grab_right5 = Pose2d(3.0, 0.5, Rotation2d(pi))
    neutral_grab_right6 = Pose2d(2, 1.5, Rotation2d(pi))
    
    auto0 = Pose2d(4.5, 0.6, Rotation2d())
    auto1 = Pose2d(6, 0.6, Rotation2d())
    auto2 = Pose2d(7.81, 1.5, Rotation2d(pi))
    auto3 = Pose2d(7.81, 5.3, Rotation2d(pi))
    auto4 = Pose2d(5.8, 5.5, Rotation2d(pi))
    auto5 = Pose2d(2.3, 5.4, Rotation2d(3*pi/4))
    auto6 = Pose2d(0.9, 4.66, Rotation2d())
    
    to_outpost0 = Pose2d(1.8, 4.8, Rotation2d())
    to_outpost1 = Pose2d(1.8, 0.9, Rotation2d())
    to_outpost_final = Pose2d(0.521, 0.604, Rotation2d())
    
    test_start_spot = Pose2d(8.328, 5.974, Rotation2d(pi))
    test_end_spot = Pose2d(9.677, 5.874, Rotation2d(pi))
    
