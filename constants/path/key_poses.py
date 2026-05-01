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
    auto_path_speed = 3 # 3.4 maybe
    bump_speed = 2.3
    intaking_speed = 1
    while_shooting_speed = 1.0
    depot_intake_speed = 0.4
    
    smoothing_radius_auto = 0.15 # 1.3 with 5.5 speed
    smoothing_radius_auto_sweep = 0.9
    smoothing_radius_teleop = 0.28
    smoothing_radius_wide = 2
    
    min_goal_end_velocity_mult = 0.1
    smoothing_time_multiplier = 1
    
    trench_path_max_acceleration = 7
    
    antidefense_lock_radius = 0.05
    antidefense_lock_rotation = 2 / 57.295
    
    # Global auto modifier variables
    MIRROR_REVERSE_PATHS = False
    START_DELAY = 0
    CHECKPOINT_1 = 0
    CHECKPOINT_2 = 0

class kPoses:
    '''Locations of every pose that has to do with drive_to_a_spot pathfinding'''
    
    # Numbers that will probably be used in every pose ever
    field_height = 8.07
    field_width = 16.54
    
    trench_from_edge = 0.65 # was 0.72 maybe this will work better
    bump_from_edge = 2.66 # was 2.55
    
    trench_from_edge_teleop_path = 0.70 # was 0.67
    
    alliance_x = 3.0
    neutral_close_x = 6.35
    neutral_far_x = 10.3
    opposing_x = 13.6

    initial_pose_bump = Pose2d(3.69, bump_from_edge, Rotation2d())
    initial_pose_trench = Pose2d(3.685, trench_from_edge, Rotation2d(pi))
    initial_pose_center = Pose2d(3.685, 4.035, Rotation2d(pi))
    
    # ------------------------------------------------------------------------------------------------------------------
    # General poses (the rotations are the ideal direction to face when traveling in the positive x direction)
    # ------------------------------------------------------------------------------------------------------------------
    
    alliance_left_trench = Pose2d(alliance_x, field_height - trench_from_edge_teleop_path, Rotation2d(pi))
    alliance_left_bump = Pose2d(alliance_x, field_height - bump_from_edge, Rotation2d())
    alliance_right_bump = Pose2d(alliance_x, bump_from_edge, Rotation2d())
    alliance_right_trench = Pose2d(alliance_x, trench_from_edge_teleop_path, Rotation2d(pi))
    
    neutral_close_left_trench = Pose2d(neutral_close_x, field_height - trench_from_edge_teleop_path, Rotation2d(pi))
    neutral_close_left_bump = Pose2d(neutral_close_x, field_height - bump_from_edge, Rotation2d())
    neutral_close_right_bump = Pose2d(neutral_close_x, bump_from_edge, Rotation2d())
    neutral_close_right_trench = Pose2d(neutral_close_x, trench_from_edge_teleop_path, Rotation2d(pi))
    
    neutral_far_left_trench = Pose2d(neutral_far_x, field_height - trench_from_edge_teleop_path, Rotation2d(pi))
    neutral_far_left_bump = Pose2d(neutral_far_x, field_height - bump_from_edge, Rotation2d())
    neutral_far_right_bump = Pose2d(neutral_far_x, bump_from_edge, Rotation2d())
    neutral_far_right_trench = Pose2d(neutral_far_x, trench_from_edge_teleop_path, Rotation2d(pi))
    
    opposing_left_trench = Pose2d(opposing_x, field_height - trench_from_edge_teleop_path, Rotation2d(pi))
    opposing_right_bump = Pose2d(opposing_x, bump_from_edge, Rotation2d())
    opposing_left_bump = Pose2d(opposing_x, field_height - bump_from_edge, Rotation2d())
    opposing_right_trench = Pose2d(opposing_x, trench_from_edge_teleop_path, Rotation2d(pi))
    
    # -------------------------------------------------------------------
    # Auto Poses
    # -------------------------------------------------------------------
    
    # The DOUBLE SWEEP
    double_sweep_1 = Pose2d(6.0, trench_from_edge, Rotation2d(pi))
    double_sweep_2 = Pose2d(8.8, 1.6, Rotation2d(pi/2))
    double_sweep_3 = Pose2d(8.0, 3.3, Rotation2d(pi/2 + 0.1)) # Is y value 3.3 change this for BIG sweep
    double_sweep_4 = Pose2d(6.0, bump_from_edge - 0.15, Rotation2d(pi))
    double_sweep_5 = Pose2d(2.8, bump_from_edge - 0.05, Rotation2d(pi))
    double_sweep_6 = Pose2d(2.8, trench_from_edge + 0.05, Rotation2d(pi))
    double_sweep_7 = Pose2d(6.5, trench_from_edge, Rotation2d(pi))
    double_sweep_8 = Pose2d(8.0, 2, Rotation2d(pi/2))
    double_sweep_9 = Pose2d(7.6, 3.2, Rotation2d(pi/2))
    double_sweep_10 = Pose2d(7.2, 4, Rotation2d(pi))
    double_sweep_11 = Pose2d(5.75, 3.5, Rotation2d(pi))
    double_sweep_12 = Pose2d(6.0, bump_from_edge - 0.05, Rotation2d(pi))
    
    double_sweep_left_4 = Pose2d(6.0, bump_from_edge, Rotation2d(pi))
    double_sweep_left_5 = Pose2d(2.6, bump_from_edge + 0.05, Rotation2d(pi))
    double_sweep_left_6 = Pose2d(2.6, trench_from_edge + 0.05, Rotation2d())
    double_sweep_left_extra_6 = Pose2d(4.25, trench_from_edge + 0.03, Rotation2d())
    double_sweep_left_7 = Pose2d(6.5, trench_from_edge, Rotation2d())
    double_sweep_left_12 = Pose2d(6.0, bump_from_edge - 0.15, Rotation2d(pi))
    
    # The DOUBLE BONUS SWEEP
    double_bonus_sweep_3 = Pose2d(8.0, 3.5, Rotation2d(pi/2 + 0.1))
    
    double_bonus_sweep_left_1 = Pose2d(6.0, trench_from_edge, Rotation2d())
    
    # The TRENCH DOUBLE SWEEP
    trench_double_sweep_4 = Pose2d(6.0, trench_from_edge + 0.16, Rotation2d(pi))
    trench_double_sweep_5 = Pose2d(3.7, trench_from_edge + 0.05, Rotation2d(pi))
    trench_double_sweep_extra_12 = Pose2d(6.5, 1, Rotation2d(pi))
    
    trench_double_sweep_left_4 = Pose2d(6.0, trench_from_edge + 0.16, Rotation2d())
    trench_double_sweep_left_5 = Pose2d(4.0, trench_from_edge + 0.05, Rotation2d())
    trench_double_sweep_left_extra_12 = Pose2d(6.3, 1, Rotation2d(0.2))
    
    # The MIDDLE SWEEP
    middle_sweep_1 = Pose2d(6.0, bump_from_edge, Rotation2d())
    middle_sweep_2 = Pose2d(6.7, 4.3, Rotation2d())
    middle_sweep_3 = Pose2d(8.1, 4.0, Rotation2d()) # Would be 8.6 if faster intake
    middle_sweep_4 = Pose2d(7.5, 3.5, Rotation2d(-pi/2))
    middle_sweep_5 = Pose2d(6.5, bump_from_edge - 0.2, Rotation2d(pi))
    middle_sweep_6 = Pose2d(2.9, bump_from_edge, Rotation2d(pi))
    middle_sweep_7 = Pose2d(1.7, 2.5, Rotation2d(3*pi/4))
    middle_sweep_8 = Pose2d(4.0, bump_from_edge, Rotation2d())
    middle_sweep_9 = Pose2d(6.0, bump_from_edge, Rotation2d())
    middle_sweep_10 = Pose2d(6.9, 4.3, Rotation2d())
    middle_sweep_11 = Pose2d(8.6, 4.0, Rotation2d())
    
    # The MIDDLE DEPOT
    middle_sweep_depot_7 = Pose2d(1.7, 2.5, Rotation2d(7*pi/4))
    middle_sweep_depot_8_before = Pose2d(2.4, 2.5, Rotation2d(pi))
    middle_sweep_depot_8_before_2 = Pose2d(2.2, 6.1, Rotation2d(pi))
    middle_sweep_depot_8 = Pose2d(1.5, 6.1, Rotation2d(pi))
    middle_sweep_depot_9 = Pose2d(0.6, 6.1, Rotation2d(pi))
    middle_sweep_depot_10 = Pose2d(1.7, field_height - 2.5, Rotation2d(3*pi/4))
    
    middle_sweep_depot_left_7 = Pose2d(1.7, 2.5, Rotation2d(7*pi/4))
    middle_sweep_depot_left_8 = Pose2d(1.5, field_height - 6.1, Rotation2d(pi))
    middle_sweep_depot_left_9 = Pose2d(0.6, field_height - 6.1, Rotation2d(pi))
    middle_sweep_depot_left_10 = Pose2d(1.7, 2.5, Rotation2d(7*pi/4))
    
    # The MIDDLE SUPPORT
    middle_support_4 = Pose2d(8.3, 5.5, Rotation2d(pi/4))
    middle_support_4_extra = Pose2d(8.3, 2.5, Rotation2d(-pi/4))
    middle_support_4_extra_2 = Pose2d(8.3, 4.0, Rotation2d(pi/4))
    middle_support_4_extra_3 = Pose2d(8.3, 4.0, Rotation2d())