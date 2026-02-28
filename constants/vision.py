from os import path
import wpilib
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout

from .math import kMath


class kLimelight:
    TARGET_INVALID_VALUE = 0.0
    TARGET_VALID_VALUE = 1.0
    MIN_HORIZONTAL_FOV = Rotation2d.fromDegrees(-29.8)
    MAX_HORIZONTAL_FOV = Rotation2d.fromDegrees(29.8)
    MIN_VERTICAL_FOV = Rotation2d.fromDegrees(-22.85)
    MAX_VERTICAL_FOV = Rotation2d.fromDegrees(22.85)
    NETWORK_TABLENAME = "limelight"
    TARGET_VALID_KEY = "tv"
    TARGET_HORIZONTAL_ANGLE_KEY = "tx"
    TARGET_VERTICAL_ANGLE_KEY = "ty"
    LED_MODE_KEY = "ledMode"
    TRACKER_MODULE_NAME = "limelight"

    FOV_HORIZONTAL = 75.9  # degrees
    FOVVERTICAL = 47.4  # degrees


class kCamera:
    LOCATION_PUBLISHER_KEY = "camera/location"

    class llFront:
        NAME = "limelight-front"
        ROBOT_TO_CAMERA_TRANSFORM = Transform3d(
            Pose3d(),
            Pose3d(
                0,
                0.2413,
                0.33655,
                Rotation3d.fromDegrees(0, 15, 0),
            ),
        )

    class llRight:
        NAME = "limelight-right"
        ROBOT_TO_CAMERA_TRANSFORM = Transform3d(
            Pose3d(),
            Pose3d(
                0.30575,
                0.2286,
                0.1143,
                Rotation3d.fromDegrees(0, 14.76, -90),
            ),
        )

class kOdometry:
    MAX_VISION_AMBIGUITY = 0.3
    MAX_VISION_Z_ERROR = 0.75
    XY_SD_COEFF = 0.02
    THETA_SD_COEFF = 0.06

    MAX_RPS = 2  # m/s
    USE_MEGATAG_2 = False
    
    MIN_APRILTAGS = 2
    MAX_ROTATIONAL_ERROR = 1


class kAprilTag:
    FIELD_LAYOUT = AprilTagFieldLayout(
        path.join(
            wpilib.getDeployDirectory(), "apriltags", "2026-rebuilt-andymark.json"
        )
    )

    POSITIONS = {
        1: Pose3d(
            (kMath.MetersPerInch * 657.37),
            (kMath.MetersPerInch * 25.80),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 126 * kMath.RadiansPerDegree),
        ),
        2: Pose3d(
            (kMath.MetersPerInch * 657.37),
            (kMath.MetersPerInch * 291.20),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 234 * kMath.RadiansPerDegree),
        ),
        3: Pose3d(
            (kMath.MetersPerInch * 455.15),
            (kMath.MetersPerInch * 317.15),
            (kMath.MetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 270 * kMath.RadiansPerDegree),
        ),
        4: Pose3d(
            (kMath.MetersPerInch * 365.20),
            (kMath.MetersPerInch * 241.64),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 0.0),
        ),
        5: Pose3d(
            (kMath.MetersPerInch * 365.20),
            (kMath.MetersPerInch * 75.39),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 0.0),
        ),
        6: Pose3d(
            (kMath.MetersPerInch * 530.49),
            (kMath.MetersPerInch * 130.17),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RadiansPerDegree),
        ),
        7: Pose3d(
            (kMath.MetersPerInch * 546.87),
            (kMath.MetersPerInch * 158.50),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        8: Pose3d(
            (kMath.MetersPerInch * 530.49),
            (kMath.MetersPerInch * 186.83),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RadiansPerDegree),
        ),
        9: Pose3d(
            (kMath.MetersPerInch * 497.77),
            (kMath.MetersPerInch * 186.83),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RadiansPerDegree),
        ),
        10: Pose3d(
            (kMath.MetersPerInch * 481.39),
            (kMath.MetersPerInch * 158.50),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RadiansPerDegree),
        ),
        11: Pose3d(
            (kMath.MetersPerInch * 497.77),
            (kMath.MetersPerInch * 130.17),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RadiansPerDegree),
        ),
        12: Pose3d(
            (kMath.MetersPerInch * 33.51),
            (kMath.MetersPerInch * 25.80),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 54 * kMath.RadiansPerDegree),
        ),
        13: Pose3d(
            (kMath.MetersPerInch * 33.51),
            (kMath.MetersPerInch * 291.20),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 306 * kMath.RadiansPerDegree),
        ),
        14: Pose3d(
            (kMath.MetersPerInch * 325.68),
            (kMath.MetersPerInch * 241.64),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 180 * kMath.RadiansPerDegree),
        ),
        15: Pose3d(
            (kMath.MetersPerInch * 325.68),
            (kMath.MetersPerInch * 75.39),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 180 * kMath.RadiansPerDegree),
        ),
        16: Pose3d(
            (kMath.MetersPerInch * 235.73),
            (kMath.MetersPerInch * -0.15),
            (kMath.MetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 90 * kMath.RadiansPerDegree),
        ),
        17: Pose3d(
            (kMath.MetersPerInch * 160.39),
            (kMath.MetersPerInch * 130.17),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RadiansPerDegree),
        ),
        18: Pose3d(
            (kMath.MetersPerInch * 144.00),
            (kMath.MetersPerInch * 158.50),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RadiansPerDegree),
        ),
        19: Pose3d(
            (kMath.MetersPerInch * 160.39),
            (kMath.MetersPerInch * 186.83),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RadiansPerDegree),
        ),
        20: Pose3d(
            (kMath.MetersPerInch * 193.10),
            (kMath.MetersPerInch * 186.83),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RadiansPerDegree),
        ),
        21: Pose3d(
            (kMath.MetersPerInch * 209.49),
            (kMath.MetersPerInch * 158.50),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        22: Pose3d(
            (kMath.MetersPerInch * 193.10),
            (kMath.MetersPerInch * 130.17),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RadiansPerDegree),
        ),
    }

    POSITIONS_ANDYMARK = {
        1: Pose3d(
            (kMath.MetersPerInch * 656.98),
            (kMath.MetersPerInch * 24.73),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 126 * kMath.RadiansPerDegree),
        ),
        2: Pose3d(
            (kMath.MetersPerInch * 656.98),
            (kMath.MetersPerInch * 291.90),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 234 * kMath.RadiansPerDegree),
        ),
        3: Pose3d(
            (kMath.MetersPerInch * 452.40),
            (kMath.MetersPerInch * 316.21),
            (kMath.MetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 270 * kMath.RadiansPerDegree),
        ),
        4: Pose3d(
            (kMath.MetersPerInch * 365.20),
            (kMath.MetersPerInch * 241.44),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 0.0),
        ),
        5: Pose3d(
            (kMath.MetersPerInch * 365.20),
            (kMath.MetersPerInch * 75.19),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 0.0),
        ),
        6: Pose3d(
            (kMath.MetersPerInch * 530.49),
            (kMath.MetersPerInch * 129.97),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RadiansPerDegree),
        ),
        7: Pose3d(
            (kMath.MetersPerInch * 546.87),
            (kMath.MetersPerInch * 158.30),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        8: Pose3d(
            (kMath.MetersPerInch * 530.49),
            (kMath.MetersPerInch * 186.63),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RadiansPerDegree),
        ),
        9: Pose3d(
            (kMath.MetersPerInch * 497.77),
            (kMath.MetersPerInch * 186.63),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RadiansPerDegree),
        ),
        10: Pose3d(
            (kMath.MetersPerInch * 481.39),
            (kMath.MetersPerInch * 158.30),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RadiansPerDegree),
        ),
        11: Pose3d(
            (kMath.MetersPerInch * 497.77),
            (kMath.MetersPerInch * 129.97),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RadiansPerDegree),
        ),
        12: Pose3d(
            (kMath.MetersPerInch * 33.91),
            (kMath.MetersPerInch * 24.73),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 54 * kMath.RadiansPerDegree),
        ),
        13: Pose3d(
            (kMath.MetersPerInch * 33.91),
            (kMath.MetersPerInch * 291.90),
            (kMath.MetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 306 * kMath.RadiansPerDegree),
        ),
        14: Pose3d(
            (kMath.MetersPerInch * 325.68),
            (kMath.MetersPerInch * 241.44),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 180 * kMath.RadiansPerDegree),
        ),
        15: Pose3d(
            (kMath.MetersPerInch * 325.68),
            (kMath.MetersPerInch * 75.19),
            (kMath.MetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kMath.RadiansPerDegree, 180 * kMath.RadiansPerDegree),
        ),
        16: Pose3d(
            (kMath.MetersPerInch * 238.49),
            (kMath.MetersPerInch * 0.42),
            (kMath.MetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 90 * kMath.RadiansPerDegree),
        ),
        17: Pose3d(
            (kMath.MetersPerInch * 160.39),
            (kMath.MetersPerInch * 129.97),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kMath.RadiansPerDegree),
        ),
        18: Pose3d(
            (kMath.MetersPerInch * 144.00),
            (kMath.MetersPerInch * 158.30),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kMath.RadiansPerDegree),
        ),
        19: Pose3d(
            (kMath.MetersPerInch * 160.39),
            (kMath.MetersPerInch * 186.63),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kMath.RadiansPerDegree),
        ),
        20: Pose3d(
            (kMath.MetersPerInch * 193.10),
            (kMath.MetersPerInch * 186.63),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kMath.RadiansPerDegree),
        ),
        21: Pose3d(
            (kMath.MetersPerInch * 209.49),
            (kMath.MetersPerInch * 158.30),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        22: Pose3d(
            (kMath.MetersPerInch * 193.10),
            (kMath.MetersPerInch * 129.97),
            (kMath.MetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kMath.RadiansPerDegree),
        ),
    }
