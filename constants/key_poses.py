'''
All poses that are important for all the paths.
These aren't known locations and should be tuned a bunch (unlike field.py)
'''

from wpimath.geometry import Pose2d, Rotation2d
from math import pi
from constants.field import kField

class kPath:
    default_path_speed = 3
    auto_path_speed = 3.5
    default_smoothing_radius = 0.6
    default_smoothing_time = 0.14 # Try 15-20% of path speed maybe
    
    # High quality variable name
    percent_slow_distance_proportional_to_max_speed_for_sequence_path = 0.4
    path_transition_slow_multiplier = 0.4 # In percent from 0 to 1

class kPoses:
    '''Locations of every pose that has to do with drive_to_a_spot pathfinding'''
    
    from_edge = 0.55
    from_edge_neutral = 0.6
    
    alliance_zone_left_trench = Pose2d(3.0, kField.WIDTH - from_edge, Rotation2d(pi))
    neutral_zone_left_trench = Pose2d(6.4, kField.WIDTH - from_edge_neutral, Rotation2d(pi))
    opposing_zone_left_trench = Pose2d(13.6, kField.WIDTH - from_edge_neutral, Rotation2d(pi))
    
    alliance_zone_right_trench = Pose2d(3.0, from_edge, Rotation2d(pi))
    neutral_zone_right_trench = Pose2d(6.4, from_edge_neutral, Rotation2d(pi))
    opposing_zone_right_trench = Pose2d(13.6, from_edge_neutral, Rotation2d(pi))
    
    alliance_zone_right_bump = Pose2d(3.0, 2.55, Rotation2d())
    neutral_zone_right_bump = Pose2d(6.2, 2.55, Rotation2d())
    opposing_zone_right_bump = Pose2d(13.2, 2.55, Rotation2d())
    
    alliance_zone_left_bump = Pose2d(3.0, kField.WIDTH - 2.55, Rotation2d())
    neutral_zone_left_bump = Pose2d(6.2, kField.WIDTH - 2.55, Rotation2d())
    opposing_zone_left_bump = Pose2d(13.2, kField.WIDTH - 2.55, Rotation2d())
    
    left_trench_feed = Pose2d(6.4, kField.WIDTH - from_edge, Rotation2d())
    right_trench_feed = Pose2d(6.4, from_edge, Rotation2d())
    
    # Auto Poses
    simple_right0 = Pose2d(3.7, from_edge, Rotation2d())
    simple_right1 = Pose2d(2.7, from_edge, Rotation2d((4*pi)/3))
    simple_right2 = Pose2d(2.7, 3, Rotation2d((4*pi)/3))
    
    neutral_grab_right0 = Pose2d(3.7, from_edge, Rotation2d())
    neutral_grab_right1 = Pose2d(6.5, from_edge_neutral, Rotation2d())
    neutral_grab_right2 = Pose2d(8.0, 1.0, Rotation2d(-pi/2))
    neutral_grab_right3 = Pose2d(7.8, 3.5, Rotation2d(-pi/2))
    neutral_grab_right4 = Pose2d(6.0, from_edge_neutral, Rotation2d(pi))
    neutral_grab_right5 = Pose2d(2.5, from_edge, Rotation2d(pi))
    neutral_grab_right6 = Pose2d(2.5, 2.5, Rotation2d(pi))
    
    neutral_grab_left0 = Pose2d(3.7, kField.WIDTH - from_edge, Rotation2d())
    neutral_grab_left1 = Pose2d(6.5, kField.WIDTH - from_edge, Rotation2d())
    neutral_grab_left2 = Pose2d(8.0, kField.WIDTH - 1.0, Rotation2d(pi/2))
    neutral_grab_left3 = Pose2d(7.8, kField.WIDTH - 3.5, Rotation2d(pi/2))
    neutral_grab_left4 = Pose2d(6.0, kField.WIDTH - from_edge_neutral, Rotation2d(pi))
    neutral_grab_left5 = Pose2d(2.5, kField.WIDTH - from_edge, Rotation2d(pi))
    neutral_grab_left6 = Pose2d(2.5, kField.WIDTH - 2.5, Rotation2d(pi))
    
    # -------------------------------------------------
    # do NOT EVER have both of these uncommented (i guess it will just do trench i guess)
    # BUMPING
    # double_grab_right0 = Pose2d(3.7, from_edge, Rotation2d())
    # double_grab_right1 = Pose2d(6.5, from_edge_neutral, Rotation2d())
    # double_grab_right2 = Pose2d(8.0, 1.0, Rotation2d(-pi/2))
    # double_grab_right3 = Pose2d(7.8, 3.5, Rotation2d(-pi/2))
    # double_grab_right4 = Pose2d(6.0, 2.50, Rotation2d(pi))
    # double_grab_right5 = Pose2d(2.5, 2.55, Rotation2d(pi))
    # double_grab_right6 = Pose2d(2.5, 2.5, Rotation2d(pi))
    # double_grab_right7 = Pose2d(2.5, 2.55, Rotation2d())
    # double_grab_right8 = Pose2d(6.5, 2.55, Rotation2d())
    # double_grab_right9 = Pose2d(7.7, 3.5, Rotation2d(-pi/2))
    # double_grab_right10 = Pose2d(7.5, 6.2, Rotation2d(-pi/2))
    # double_grab_right11 = Pose2d(6.0, 2.55, Rotation2d(pi))
    # double_grab_right12 = Pose2d(2.5, 2.55, Rotation2d(pi))
    # double_grab_right13 = Pose2d(2.5, 2.5, Rotation2d(pi))
    # TRENCHING
    double_grab_right0 = Pose2d(3.7, from_edge, Rotation2d())
    double_grab_right1 = Pose2d(6.5, from_edge_neutral, Rotation2d())
    double_grab_right2 = Pose2d(7.35, 1.0, Rotation2d(-pi/2))
    double_grab_right3 = Pose2d(7.15, 2.7, Rotation2d(-pi/2))
    double_grab_right4 = Pose2d(6.0, from_edge_neutral, Rotation2d(pi))
    double_grab_right5 = Pose2d(2.5, from_edge, Rotation2d(pi))
    double_grab_right6 = Pose2d(2.5, 2.5, Rotation2d(pi))
    double_grab_right7 = Pose2d(2.5, from_edge, Rotation2d())
    double_grab_right8 = Pose2d(6.5, from_edge_neutral, Rotation2d())
    double_grab_right9 = Pose2d(7.0, 2.5, Rotation2d(-pi/2))
    double_grab_right10 = Pose2d(6.8, 4.0, Rotation2d(-pi/2))
    double_grab_right11 = Pose2d(6.0, from_edge_neutral, Rotation2d(pi))
    double_grab_right12 = Pose2d(2.5, from_edge, Rotation2d(pi))
    double_grab_right13 = Pose2d(2.5, 2.5, Rotation2d(pi))
    # -------------------------------------------------
    
    # -------------------------------------------------
    # do NOT EVER have both of these uncommented (i guess it will just do trench i guess)
    # BUMPING
    # double_grab_left0 = Pose2d(3.7, kField.WIDTH - from_edge, Rotation2d())
    # double_grab_left1 = Pose2d(6.5, kField.WIDTH - from_edge_neutral, Rotation2d())
    # double_grab_left2 = Pose2d(8.0, kField.WIDTH - 1.0, Rotation2d(pi/2))
    # double_grab_left3 = Pose2d(7.8, kField.WIDTH - 3.5, Rotation2d(pi/2))
    # double_grab_left4 = Pose2d(6.0, kField.WIDTH - 2.50, Rotation2d(pi))
    # double_grab_left5 = Pose2d(2.5, kField.WIDTH - 2.55, Rotation2d(pi))
    # double_grab_left6 = Pose2d(2.5, kField.WIDTH - 2.5, Rotation2d(pi))
    # double_grab_left7 = Pose2d(2.5, kField.WIDTH - 2.55, Rotation2d())
    # double_grab_left8 = Pose2d(6.5, kField.WIDTH - 2.55, Rotation2d())
    # double_grab_left9 = Pose2d(7.7, kField.WIDTH - 3.5, Rotation2d(pi/2))
    # double_grab_left10 = Pose2d(7.5, kField.WIDTH - 6.2, Rotation2d(pi/2))
    # double_grab_left11 = Pose2d(6.0, kField.WIDTH - 2.55, Rotation2d(pi))
    # double_grab_left12 = Pose2d(2.5, kField.WIDTH - 2.55, Rotation2d(pi))
    # double_grab_left13 = Pose2d(2.5, kField.WIDTH - 2.5, Rotation2d(pi))
    # TRENCHING
    double_grab_left0 = Pose2d(3.7, kField.WIDTH - from_edge, Rotation2d())
    double_grab_left1 = Pose2d(6.5, kField.WIDTH - from_edge_neutral, Rotation2d())
    double_grab_left2 = Pose2d(7.35, kField.WIDTH - 1.0, Rotation2d(pi/2))
    double_grab_left3 = Pose2d(7.15, kField.WIDTH - 2.7, Rotation2d(pi/2))
    double_grab_left4 = Pose2d(6.0, kField.WIDTH - from_edge_neutral, Rotation2d(pi))
    double_grab_left5 = Pose2d(2.5, kField.WIDTH - from_edge, Rotation2d(pi))
    double_grab_left6 = Pose2d(2.5, kField.WIDTH - 2.5, Rotation2d(pi))
    double_grab_left7 = Pose2d(2.5, kField.WIDTH - from_edge, Rotation2d())
    double_grab_left8 = Pose2d(6.5, kField.WIDTH - from_edge_neutral, Rotation2d())
    double_grab_left9 = Pose2d(7.0, kField.WIDTH - 2.5, Rotation2d(pi/2))
    double_grab_left10 = Pose2d(6.8, kField.WIDTH - 4.0, Rotation2d(pi/2))
    double_grab_left11 = Pose2d(6.0, kField.WIDTH - from_edge_neutral, Rotation2d(pi))
    double_grab_left12 = Pose2d(2.5, kField.WIDTH - from_edge, Rotation2d(pi))
    double_grab_left13 = Pose2d(2.5, kField.WIDTH - 2.5, Rotation2d(pi))
    # -------------------------------------------------
    
    depot_grab0 = Pose2d(3.7, 5.5, Rotation2d())
    depot_grab1 = Pose2d(2.0, 5.5, Rotation2d())
    depot_grab2 = Pose2d(1.5, 5.9, Rotation2d()) # without sequence for more precision
    depot_grab3 = Pose2d(0.55, 5.9, Rotation2d()) # Race with 3 seconds in case target not met
    depot_grab4 = Pose2d(2.5, 5, Rotation2d())
    
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
    
