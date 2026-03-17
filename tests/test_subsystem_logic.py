"""Unit tests for subsystem logic, constants, and utilities."""

import math
import pytest
from wpimath.geometry import Pose2d, Rotation2d


# ── kHoodMotor conversion helpers ──────────────────────────────────────────


class TestHoodMotorConversions:
    def test_to_revs_known_value(self):
        from constants.shooter import kHoodMotor

        # 35 degrees of hood travel = 15.1 motor revolutions
        assert math.isclose(kHoodMotor.to_revs(35.0), 15.1, rel_tol=1e-6)

    def test_to_deg_known_value(self):
        from constants.shooter import kHoodMotor

        assert math.isclose(kHoodMotor.to_deg(15.1), 35.0, rel_tol=1e-6)

    def test_round_trip(self):
        from constants.shooter import kHoodMotor

        for deg in [0.0, 5.0, 15.0, 25.0, 35.0]:
            result = kHoodMotor.to_deg(kHoodMotor.to_revs(deg))
            assert math.isclose(result, deg, rel_tol=1e-9), f"Round-trip failed for {deg}"


# ── Interpolation data integrity ──────────────────────────────────────────


class TestInterpolationData:
    def test_shot_time_monotonically_increasing(self):
        from constants.shooter import kShooterData

        distances = [d for d, _ in kShooterData.SHOT_TIME]
        for i in range(len(distances) - 1):
            assert distances[i] < distances[i + 1], (
                f"SHOT_TIME not sorted: {distances[i]} >= {distances[i+1]}"
            )

    def test_shot_angles_monotonically_increasing(self):
        from constants.shooter import kShooterData

        distances = [d for d, _ in kShooterData.SHOT_ANGLES]
        for i in range(len(distances) - 1):
            assert distances[i] < distances[i + 1], (
                f"SHOT_ANGLES not sorted: {distances[i]} >= {distances[i+1]}"
            )

    def test_shot_time_has_data(self):
        from constants.shooter import kShooterData

        assert len(kShooterData.SHOT_TIME) >= 2

    def test_shot_angles_has_data(self):
        from constants.shooter import kShooterData

        assert len(kShooterData.SHOT_ANGLES) >= 2


# ── DualMotorShooter state machine ────────────────────────────────────────


class TestShooterStateMachine:
    @pytest.fixture
    def shooter(self):
        from subsystems.shooter.dual_shooter import DualMotorShooter

        return DualMotorShooter()

    def test_starts_idle(self, shooter):
        from subsystems.shooter.dual_shooter import ShooterState

        assert shooter._state == ShooterState.IDLE
        assert shooter.is_charged is False

    def test_enable_sets_state(self, shooter):
        from subsystems.shooter.dual_shooter import ShooterState

        shooter.enable()
        assert shooter._state == ShooterState.ENABLED
        assert shooter.is_charged is False

    def test_disable_returns_to_idle(self, shooter):
        from subsystems.shooter.dual_shooter import ShooterState

        shooter.enable()
        shooter.disable()
        assert shooter._state == ShooterState.IDLE
        assert shooter.is_charged is False


# ── RobotZoneChecker ──────────────────────────────────────────────────────


class TestRobotZoneChecker:
    """Zone tests assume blue alliance (no flip).

    Blue alliance zone: x in [0, 3.5], y in [0, 8.07]
    Neutral zone: x in [5.7, 10.8], y in [0, 8.07]
    Left neutral: x in [5.7, 10.8], y in [4.035, 8.07]
    Right neutral: x in [5.7, 10.8], y in [0, 4.035]
    Transition zone: x in [3.5, 5.7] or x in [10.86, 13.07]
    """

    def test_alliance_zone_center(self):
        from util.robot_zone_checker import RobotZoneChecker

        pose = Pose2d(2.0, 4.0, Rotation2d())
        assert RobotZoneChecker.is_in_alliance_zone(pose)

    def test_neutral_zone_center(self):
        from util.robot_zone_checker import RobotZoneChecker

        pose = Pose2d(8.0, 4.0, Rotation2d())
        assert RobotZoneChecker.is_in_neutral_zone(pose)

    def test_left_neutral_zone(self):
        from util.robot_zone_checker import RobotZoneChecker

        # Left neutral = upper half of neutral zone (y > field_height/2)
        pose = Pose2d(8.0, 6.0, Rotation2d())
        assert RobotZoneChecker.is_in_left_neutral_zone(pose)
        assert not RobotZoneChecker.is_in_right_neutral_zone(pose)

    def test_right_neutral_zone(self):
        from util.robot_zone_checker import RobotZoneChecker

        # Right neutral = lower half of neutral zone (y < field_height/2)
        pose = Pose2d(8.0, 2.0, Rotation2d())
        assert RobotZoneChecker.is_in_right_neutral_zone(pose)
        assert not RobotZoneChecker.is_in_left_neutral_zone(pose)

    def test_transition_zone(self):
        from util.robot_zone_checker import RobotZoneChecker

        # Between alliance and neutral (x in [3.5, 5.7])
        pose = Pose2d(4.5, 4.0, Rotation2d())
        assert RobotZoneChecker.is_near_transition_zone(pose)

    def test_alliance_zone_not_transition(self):
        from util.robot_zone_checker import RobotZoneChecker

        pose = Pose2d(2.0, 4.0, Rotation2d())
        assert not RobotZoneChecker.is_near_transition_zone(pose)

    def test_is_between_helper(self):
        from util.robot_zone_checker import RobotZoneChecker

        assert RobotZoneChecker.is_between(5, 3, 7)
        assert RobotZoneChecker.is_between(5, 7, 3)  # reversed bounds
        assert not RobotZoneChecker.is_between(2, 3, 7)
        assert not RobotZoneChecker.is_between(3, 3, 7)  # boundary exclusive
