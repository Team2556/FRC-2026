from os import path
import wpilib
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout

from .math import kRadiansPerDegree, kMetersPerInch


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


class kAprilTag:
    FIELD_LAYOUT = AprilTagFieldLayout(
        path.join(
            wpilib.getDeployDirectory(), "apriltags", "2026-rebuilt-andymark.json"
        )
    )

    POSITIONS = {
        1: Pose3d(
            (kMetersPerInch * 657.37),
            (kMetersPerInch * 25.80),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
        ),
        2: Pose3d(
            (kMetersPerInch * 657.37),
            (kMetersPerInch * 291.20),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
        ),
        3: Pose3d(
            (kMetersPerInch * 455.15),
            (kMetersPerInch * 317.15),
            (kMetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
        ),
        4: Pose3d(
            (kMetersPerInch * 365.20),
            (kMetersPerInch * 241.64),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
        ),
        5: Pose3d(
            (kMetersPerInch * 365.20),
            (kMetersPerInch * 75.39),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
        ),
        6: Pose3d(
            (kMetersPerInch * 530.49),
            (kMetersPerInch * 130.17),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
        ),
        7: Pose3d(
            (kMetersPerInch * 546.87),
            (kMetersPerInch * 158.50),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        8: Pose3d(
            (kMetersPerInch * 530.49),
            (kMetersPerInch * 186.83),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
        ),
        9: Pose3d(
            (kMetersPerInch * 497.77),
            (kMetersPerInch * 186.83),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
        ),
        10: Pose3d(
            (kMetersPerInch * 481.39),
            (kMetersPerInch * 158.50),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
        ),
        11: Pose3d(
            (kMetersPerInch * 497.77),
            (kMetersPerInch * 130.17),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
        ),
        12: Pose3d(
            (kMetersPerInch * 33.51),
            (kMetersPerInch * 25.80),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
        ),
        13: Pose3d(
            (kMetersPerInch * 33.51),
            (kMetersPerInch * 291.20),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
        ),
        14: Pose3d(
            (kMetersPerInch * 325.68),
            (kMetersPerInch * 241.64),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
        ),
        15: Pose3d(
            (kMetersPerInch * 325.68),
            (kMetersPerInch * 75.39),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
        ),
        16: Pose3d(
            (kMetersPerInch * 235.73),
            (kMetersPerInch * -0.15),
            (kMetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
        ),
        17: Pose3d(
            (kMetersPerInch * 160.39),
            (kMetersPerInch * 130.17),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
        ),
        18: Pose3d(
            (kMetersPerInch * 144.00),
            (kMetersPerInch * 158.50),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
        ),
        19: Pose3d(
            (kMetersPerInch * 160.39),
            (kMetersPerInch * 186.83),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
        ),
        20: Pose3d(
            (kMetersPerInch * 193.10),
            (kMetersPerInch * 186.83),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
        ),
        21: Pose3d(
            (kMetersPerInch * 209.49),
            (kMetersPerInch * 158.50),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        22: Pose3d(
            (kMetersPerInch * 193.10),
            (kMetersPerInch * 130.17),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
        ),
    }

    POSITIONS_ANDYMARK = {
        1: Pose3d(
            (kMetersPerInch * 656.98),
            (kMetersPerInch * 24.73),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
        ),
        2: Pose3d(
            (kMetersPerInch * 656.98),
            (kMetersPerInch * 291.90),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
        ),
        3: Pose3d(
            (kMetersPerInch * 452.40),
            (kMetersPerInch * 316.21),
            (kMetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
        ),
        4: Pose3d(
            (kMetersPerInch * 365.20),
            (kMetersPerInch * 241.44),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
        ),
        5: Pose3d(
            (kMetersPerInch * 365.20),
            (kMetersPerInch * 75.19),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
        ),
        6: Pose3d(
            (kMetersPerInch * 530.49),
            (kMetersPerInch * 129.97),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
        ),
        7: Pose3d(
            (kMetersPerInch * 546.87),
            (kMetersPerInch * 158.30),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        8: Pose3d(
            (kMetersPerInch * 530.49),
            (kMetersPerInch * 186.63),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
        ),
        9: Pose3d(
            (kMetersPerInch * 497.77),
            (kMetersPerInch * 186.63),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
        ),
        10: Pose3d(
            (kMetersPerInch * 481.39),
            (kMetersPerInch * 158.30),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
        ),
        11: Pose3d(
            (kMetersPerInch * 497.77),
            (kMetersPerInch * 129.97),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
        ),
        12: Pose3d(
            (kMetersPerInch * 33.91),
            (kMetersPerInch * 24.73),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
        ),
        13: Pose3d(
            (kMetersPerInch * 33.91),
            (kMetersPerInch * 291.90),
            (kMetersPerInch * 58.50),
            Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
        ),
        14: Pose3d(
            (kMetersPerInch * 325.68),
            (kMetersPerInch * 241.44),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
        ),
        15: Pose3d(
            (kMetersPerInch * 325.68),
            (kMetersPerInch * 75.19),
            (kMetersPerInch * 73.54),
            Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
        ),
        16: Pose3d(
            (kMetersPerInch * 238.49),
            (kMetersPerInch * 0.42),
            (kMetersPerInch * 51.25),
            Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
        ),
        17: Pose3d(
            (kMetersPerInch * 160.39),
            (kMetersPerInch * 129.97),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
        ),
        18: Pose3d(
            (kMetersPerInch * 144.00),
            (kMetersPerInch * 158.30),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
        ),
        19: Pose3d(
            (kMetersPerInch * 160.39),
            (kMetersPerInch * 186.63),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
        ),
        20: Pose3d(
            (kMetersPerInch * 193.10),
            (kMetersPerInch * 186.63),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
        ),
        21: Pose3d(
            (kMetersPerInch * 209.49),
            (kMetersPerInch * 158.30),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 0.0),
        ),
        22: Pose3d(
            (kMetersPerInch * 193.10),
            (kMetersPerInch * 129.97),
            (kMetersPerInch * 12.13),
            Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
        ),
    }
