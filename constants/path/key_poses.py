'''
All poses that are important for all the paths.
These aren't known locations and should be tuned a bunch (unlike field.py)
'''

from math import pi

from wpimath.geometry import Pose2d, Rotation2d

from constants.field import kField

class kPath:
    max_translational_acceleration = 8.0
    max_rotational_acceleration = 30.0
    
    default_path_speed = 5.5
    auto_path_speed = 5.5
    intaking_speed = 2.0
    while_shooting_speed = 1.0
    
    smoothing_radius = 0.6
    
    min_goal_end_velocity_mult = 0.3

class kPoses:
    '''Locations of every pose that has to do with drive_to_a_spot pathfinding'''
    
    # Numbers that will probably be used in every pose ever
    field_height = 8.07
    field_width = 16.54
    
    trench_from_edge = 0.55
    bump_from_edge = 2.55
    
    alliance_x = 3.0
    neutral_close_x = 7.35
    neutral_far_x = 10.3
    opposing_x = 13.6
    
    # General poses
    alliance_left_trench = Pose2d(alliance_x, field_height - trench_from_edge, Rotation2d(pi))
    alliance_left_bump = Pose2d(alliance_x, field_height - bump_from_edge, Rotation2d())
    alliance_right_bump = Pose2d(alliance_x, bump_from_edge, Rotation2d())
    alliance_right_trench = Pose2d(alliance_x, trench_from_edge, Rotation2d(pi))
    
    neutral_close_left_trench = Pose2d(neutral_close_x, field_height - trench_from_edge, Rotation2d(pi))
    neutral_close_left_bump = Pose2d(neutral_close_x, field_height - bump_from_edge, Rotation2d())
    neutral_close_right_bump = Pose2d(neutral_close_x, bump_from_edge, Rotation2d())
    neutral_close_right_trench = Pose2d(neutral_close_x, trench_from_edge, Rotation2d(pi))
    
    neutral_far_left_trench = Pose2d(neutral_far_x, field_height - trench_from_edge, Rotation2d(pi))
    neutral_far_left_bump = Pose2d(neutral_far_x, field_height - bump_from_edge, Rotation2d())
    neutral_far_right_bump = Pose2d(neutral_far_x, bump_from_edge, Rotation2d())
    neutral_far_right_trench = Pose2d(neutral_far_x, trench_from_edge, Rotation2d(pi))
    
    opposing_left_trench = Pose2d(opposing_x, field_height - trench_from_edge, Rotation2d(pi))
    opposing_right_bump = Pose2d(opposing_x, bump_from_edge, Rotation2d())
    opposing_left_bump = Pose2d(opposing_x, field_height - bump_from_edge, Rotation2d())
    opposing_right_trench = Pose2d(opposing_x, trench_from_edge, Rotation2d(pi))
    
    left_trench_feed = Pose2d(neutral_close_x, field_height - trench_from_edge, Rotation2d())
    right_trench_feed = Pose2d(neutral_close_x, trench_from_edge, Rotation2d())
    
    # Auto Poses
    right_sweep_1 = Pose2d(7, 4, Rotation2d(pi/2))
    right_sweep_2 = Pose2d(6.7, 1.3, Rotation2d(-pi/2))
    
