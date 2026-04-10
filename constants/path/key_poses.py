'''
All poses that are important for all the paths.
These aren't known locations and should be tuned a bunch (unlike field.py)
'''

from math import pi

from wpimath.geometry import Pose2d, Rotation2d

from constants.field import kField

class kPath:
    max_translational_acceleration = 8.0
    max_rotational_acceleration = 21.0
    
    default_path_speed = 4 # 4.0 maybe
    auto_path_speed = 3.2 # 3.4 maybe
    bump_speed = 2.3
    intaking_speed = 0.6
    while_shooting_speed = 1.0
    
    smoothing_radius_auto = 0.15 # 1.3 with 5.5 speed
    smoothing_radius_teleop = 0.28
    smoothing_radius_wide = 2
    
    min_goal_end_velocity_mult = 0.1
    smoothing_time_multiplier = 1
    
    # Global variable thing for mirroring paths.
    MIRROR_REVERSE_PATHS = False

class kPoses:
    '''Locations of every pose that has to do with drive_to_a_spot pathfinding'''
    
    # Numbers that will probably be used in every pose ever
    field_height = 8.07
    field_width = 16.54
    
    trench_from_edge = 0.72 # was 0.55 maybe this will work better
    bump_from_edge = 2.66 # was 2.55
    
    alliance_x = 3.0
    neutral_close_x = 6.35
    neutral_far_x = 10.3
    opposing_x = 13.6

    initial_poses = {
        "_none" : None,
        "BUMP" : Pose2d(3.65, bump_from_edge, Rotation2d()),
        "TRENCH" : Pose2d(4, trench_from_edge, Rotation2d(pi)),
    }
    
    # ------------------------------------------------------------------------------------------------------------------
    # General poses (the rotations are the ideal direction to face when traveling in the positive x direction)
    # ------------------------------------------------------------------------------------------------------------------
    
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
    
    # -------------------------------------------------------------------
    # Auto Poses
    # -------------------------------------------------------------------
    
    short_sweep_1 = Pose2d(6.5, trench_from_edge + 0.1, Rotation2d(pi))
    short_sweep_2 = Pose2d(8.9, 1.6, Rotation2d(pi/2))
    short_sweep_3 = Pose2d(7.9, 2.5, Rotation2d(pi/2 + 0.1))
    short_sweep_4 = Pose2d(7.9, 3.0, Rotation2d(pi))
    short_sweep_5 = Pose2d(6.5, 3.05, Rotation2d(pi))
    
    close_sweep_1 = Pose2d(6.5, trench_from_edge + 0.1, Rotation2d(pi))
    close_sweep_2 = Pose2d(7.5, 1.6, Rotation2d(pi/2))
    close_sweep_3 = Pose2d(7, 3.9, Rotation2d(pi/2 + 0.1))
    close_sweep_4 = Pose2d(5.85, 3.4, Rotation2d(pi))
    
    wide_sweep_1 = Pose2d(6.5, trench_from_edge + 0.1, Rotation2d(pi))
    wide_sweep_2 = Pose2d(8.9, 1.6, Rotation2d(pi/2))
    wide_sweep_3 = Pose2d(8.0, 4.0, Rotation2d(pi/2 + 0.1))
    wide_sweep_4 = Pose2d(6.5, bump_from_edge, Rotation2d(2*pi/3))
    
    cleanup_sweep_1 = Pose2d(6.5, trench_from_edge + 0.1, Rotation2d(pi))
    cleanup_sweep_2 = Pose2d(8.9, 1.6, Rotation2d(pi/2))
    cleanup_sweep_3 = Pose2d(7.8, 3.5, Rotation2d(pi/2 + 0.1))
    cleanup_sweep_4 = Pose2d(6.5, 4.0, Rotation2d(pi))
    cleanup_sweep_5 = Pose2d(6.2, 2.2, Rotation2d(4*pi/3))
    
    bump_shoot_1 = Pose2d(6.5, bump_from_edge + 0.20, Rotation2d(pi))
    bump_shoot_2 = Pose2d(2.5, bump_from_edge, Rotation2d(pi))
    bump_shoot_3 = Pose2d(2.5, 0, Rotation2d(pi))
    bump_shoot_4 = Pose2d(2.5, 3, Rotation2d(pi))
    bump_shoot_5 = Pose2d(2.8, trench_from_edge + 0.05, Rotation2d(pi))
    left_bump_shoot_1 = Pose2d(2.3, 2.6, Rotation2d(pi))
    
    bump_half_shoot_1 = Pose2d(6.5, bump_from_edge + 0.20, Rotation2d(pi))
    bump_half_shoot_2 = Pose2d(2.5, bump_from_edge, Rotation2d(5*pi/4))
    bump_half_shoot_3 = Pose2d(2.5, trench_from_edge + 0.1, Rotation2d(pi))
    
    spot_shoot_1 = Pose2d(2.9, trench_from_edge, Rotation2d(pi))
    spot_shoot_2 = Pose2d(3.2, trench_from_edge, Rotation2d(pi))
    
    trench_extake_1 = Pose2d(6.5, trench_from_edge + 0.2, Rotation2d(pi))
    
    stay_bump_shoot_1 = Pose2d(6.5, bump_from_edge + 0.10, Rotation2d(pi))
    stay_bump_shoot_2 = Pose2d(2.5, bump_from_edge, Rotation2d(pi))
    stay_bump_shoot_3 = Pose2d(3.5, bump_from_edge, Rotation2d())
    
    bump_middle_sweep_1 = Pose2d(6.2, bump_from_edge, Rotation2d())
    bump_middle_sweep_2 = Pose2d(6.5, 3.8, Rotation2d())
    bump_middle_sweep_3 = Pose2d(8.5, 3.8, Rotation2d())
    bump_middle_sweep_4 = Pose2d(7.2, 2.5, Rotation2d(7*pi/6))
    
    bump_side_sweep_1 = Pose2d(6.3, bump_from_edge, Rotation2d())
    bump_side_sweep_2 = Pose2d(7.5, 3.5, Rotation2d(-pi/2 + 0.1))
    bump_side_sweep_3 = Pose2d(7.5, 1.2, Rotation2d(-pi/2 - 0.1))
    bump_side_sweep_4 = Pose2d(6.2, bump_from_edge, Rotation2d(3*pi/4))
    # bump_side_sweep_4 = Pose2d(6.2, 3.5, Rotation2d())
    
    # bump_close_sweep_1 = Pose2d(6.3, bump_from_edge, Rotation2d())
    # bump_close_sweep_2 = Pose2d(7.5, 3.5, Rotation2d(-pi/2 + 0.1))
    # bump_close_sweep_3 = Pose2d(7.5, 1.2, Rotation2d(-pi/2 - 0.1))
    # bump_close_sweep_4 = Pose2d(6.2, bump_from_edge, Rotation2d(3*pi/4))
    
    sit_middle = Pose2d(8.5, 4, Rotation2d())
    
