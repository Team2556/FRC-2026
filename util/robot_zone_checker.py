from wpimath.geometry import Pose2d, Rotation2d

from util.flip_util import FlipUtil

class RobotZoneChecker:
    '''Cool utility that checks if a pose is within a certain zone (using opposite coordinates that make a rectangle)'''
    
    # All of these are in meters
    field_height = 8.07
    field_width = 16.54
    
    can_shoot_hub_x = 3.5
    
    right_alliance_zone_border = 4.6
    left_opposing_zone_border = 11.94
    
    left_neutral_zone_x = 5.7
    right_neutral_zone_x = 10.8
    
    left_transition_zone_x = 3.6
    right_transition_zone_x = 5.6
    
    left_opposing_transition_zone_x = 10.76
    right_opposing_transition_zone_x = 12.97
    
    left_opposing_alliance_zone_x = 13.00
    
    @staticmethod
    def is_between(value, num1, num2):
        return (value > num1 and value < num2) or (value > num2 and value < num1)
    
    @staticmethod
    def is_within_pose(pose : Pose2d, first_coordinate : Pose2d, opposite_coordinate : Pose2d) -> bool:
        return (
            RobotZoneChecker.is_between(pose.X(), first_coordinate.X(), opposite_coordinate.X())
        and RobotZoneChecker.is_between(pose.Y(), first_coordinate.Y(), opposite_coordinate.Y())
        )    
    @staticmethod
    def is_in_hub_shooting_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(0, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.can_shoot_hub_x, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    @staticmethod
    def is_in_alliance_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(0, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_alliance_zone_border, RobotZoneChecker.field_height, Rotation2d())),
        )
        
    @staticmethod
    def is_in_neutral_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_alliance_zone_border, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_opposing_zone_border, RobotZoneChecker.field_height, Rotation2d())),
        )
        
    @staticmethod
    def is_in_opposing_alliance_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_opposing_zone_border, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.field_width, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    @staticmethod
    def is_in_alliance_transition_zone(pose: Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_transition_zone_x, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_transition_zone_x, RobotZoneChecker.field_height, Rotation2d())),
        )

    @staticmethod
    def is_in_opposing_transition_zone(pose: Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_opposing_transition_zone_x, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_opposing_transition_zone_x, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    def is_near_transition_zone(pose: Pose2d):
        return RobotZoneChecker.is_in_alliance_transition_zone or RobotZoneChecker.is_in_opposing_transition_zone
    
    # Y value specific ones
    @staticmethod
    def is_in_left_passing_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_neutral_zone_x, RobotZoneChecker.field_height / 2, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.field_width, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    @staticmethod
    def is_in_right_passing_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.left_neutral_zone_x, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.field_width, RobotZoneChecker.field_height / 2, Rotation2d())),
        )
    
    @staticmethod
    def is_in_left_alliance_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(0, RobotZoneChecker.field_height / 2, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_alliance_zone_border + 0.3, RobotZoneChecker.field_height, Rotation2d())),
        )
    
    @staticmethod
    def is_in_right_alliance_zone(pose : Pose2d):
        return RobotZoneChecker.is_within_pose(
            pose, 
            FlipUtil.fieldPose(Pose2d(0, 0, Rotation2d())),
            FlipUtil.fieldPose(Pose2d(RobotZoneChecker.right_alliance_zone_border + 0.3, RobotZoneChecker.field_height / 2, Rotation2d())),
        )