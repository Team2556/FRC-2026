import math

import numpy as np

from enum import Enum

import wpilib
from wpilib import RobotController
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import degreesToRadians, radiansToDegrees

from commands2 import Subsystem

from wpimath.geometry import Pose2d

from phoenix6.hardware import TalonFXS
from phoenix6.controls import MotionMagicVoltage, DutyCycleOut
from phoenix6.signals import NeutralModeValue, ReverseLimitValue

from util.editable_pid import EditablePID
from util.nt_util import NTTable
from util.math_helpers import distanceFromPose2dtoPose2d, clamp

from subsystems.drivetrain.driveio import CustomSwerve

from constants.shooter import kHoodMotor, kShooterData, kShooterMotor, kShooterConfig
from constants.canbus import kCANId


class HoodStates(Enum):
    HIDE = 0
    AUTO = 1
    MAX = 2
    OUTER_RING = 3
    INNER_RING = 4
    RESETTING = 5
    NONE = 6


class ShooterHood(Subsystem):
    def __init__(self):
        super().__init__()

        self._state: HoodStates = HoodStates.HIDE
        self._motor = TalonFXS(kCANId.shooter.HOOD_CONTROL, "rio")

        if not wpilib.RobotBase.isSimulation():
            self._motor.configurator.apply(kHoodMotor._CONFIG)
            self._motor.setNeutralMode(NeutralModeValue.BRAKE)

        self.position_request = MotionMagicVoltage(position=0, enable_foc=False)
        self.home_request = DutyCycleOut(0, enable_foc=False)

        self.hard_stopped = False
        self.auto_hood_angle = 5.0
        self.force_physics_aiming = False  # set by align command when R Bumper held

        self.editable_pid = EditablePID("Shooter/Hood", self._motor, kHoodMotor._CONFIG)

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.float("Hood Position (deg)", 0.0)
        self.nt.float("Target Angle (deg)", 0.0)
        self.nt.string("State", "HIDE")

        # Physics-based aiming tuning (dashboard-editable)
        self.nt.float("Velocity Efficiency", kHoodMotor.VELOCITY_EFFICIENCY)
        self.nt.float("Exit Angle Offset (deg)", kHoodMotor.EXIT_ANGLE_OFFSET_DEG)
        self.nt.bool("Use Physics Aiming", False)

        self._target_position_revs = 0

        # Simulation
        if wpilib.RobotBase.isSimulation():
            _model = DCMotor.krakenX60(1)  # closest proxy for Minion motor
            self._arm_sim = SingleJointedArmSim(
                gearbox=_model,
                gearing=kHoodMotor.SIM_GEARING,
                moi=SingleJointedArmSim.estimateMOI(
                    kHoodMotor.ARM_LENGTH_M, kHoodMotor.ARM_MASS_KG
                ),
                armLength=kHoodMotor.ARM_LENGTH_M,
                minAngle=degreesToRadians(kHoodMotor.MIN_ANGLE_DEG),
                maxAngle=degreesToRadians(kHoodMotor.MAX_ANGLE_DEG),
                simulateGravity=True,
                startingAngle=degreesToRadians(kHoodMotor.MIN_ANGLE_DEG),
            )

    # ------------------------------------------------------------------
    # State & control
    # ------------------------------------------------------------------

    def set_state(self, state: HoodStates) -> None:
        self._state = state

    def set_speed(self, speed: float) -> None:
        self._motor.set_control(self.home_request.with_output(speed))

    def set_position(self, position_revs: float) -> None:
        min_revs = kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG)
        max_revs = kHoodMotor.to_revs(kHoodMotor.MAX_ANGLE_DEG)

        self._target_position_revs = clamp(position_revs, min_revs, max_revs)
        self._motor.set_control(
            self.position_request.with_position(self._target_position_revs)
        )

    def is_at_angle(self) -> bool:
        current_deg = kHoodMotor.to_deg(self._motor.get_position().value)
        target_deg = kHoodMotor.to_deg(self._target_position_revs)

        return abs(current_deg - target_deg) <= kHoodMotor.REACH_TARGET_ANGLE_ERROR

    def is_hard_stopped(self) -> bool:
        return (
            self._motor.get_reverse_limit().value is ReverseLimitValue.CLOSED_TO_GROUND
        )

    # ------------------------------------------------------------------
    # Aiming
    # ------------------------------------------------------------------

    def add_auto_hood_measurement(
        self, drivestate: CustomSwerve.DriveState, target_pose: Pose2d
    ) -> None:
        robot_pose = drivestate.pose
        distance = distanceFromPose2dtoPose2d(robot_pose, target_pose)

        interpolated_angle = self.get_angle_by_distance(distance=distance)
        self.auto_hood_angle = clamp(
            interpolated_angle, kHoodMotor.HOME_ANGLE_DEG, kHoodMotor.MAX_ANGLE_DEG
        )

    def get_angle_by_distance(self, distance: float) -> float:
        """Return the target hood angle (degrees) for the given distance.

        Uses physics-based projectile kinematics when 'Use Physics Aiming' is
        enabled on the dashboard, otherwise falls back to the interpolation table.
        """
        if self.force_physics_aiming or self.nt.get("Use Physics Aiming"):
            return self.calculate_hood_angle(distance)

        interpolation_distance_data, interpolation_angle_data = zip(
            *kShooterData.SHOT_ANGLES_MAGNOLIA
        )
        return float(
            np.interp(distance, interpolation_distance_data, interpolation_angle_data)
        )

    def calculate_hood_angle(self, distance: float) -> float:
        """Projectile kinematics: optimal low-trajectory launch angle (degrees).

        Uses actual shooter RPM and wheel radius for exit velocity.
        Tunable via dashboard:
          - Velocity Efficiency: fraction of theoretical exit velocity realized
          - Exit Angle Offset (deg): additive correction after physics calc
        """
        efficiency = self.nt.get("Velocity Efficiency")
        offset_deg = self.nt.get("Exit Angle Offset (deg)")

        rpm = abs(kShooterMotor.CURRENT_TARGET_RPM)
        v = rpm * kShooterConfig.EXIT_VELOCITY_FACTOR * efficiency
        if v < 0.1:
            return kHoodMotor.MIN_ANGLE_DEG

        delta_h = kHoodMotor.HUB_HEIGHT_M - kHoodMotor.SHOOTER_HEIGHT_M
        g = 9.81

        discriminant = v**4 - g * (g * distance**2 + 2 * delta_h * v**2)
        if discriminant < 0:
            return kHoodMotor.MAX_ANGLE_DEG  # can't reach — max loft

        theta_rad = math.atan((v**2 - math.sqrt(discriminant)) / (g * distance))
        theta_deg = math.degrees(theta_rad) + offset_deg

        return max(kHoodMotor.MIN_ANGLE_DEG, min(kHoodMotor.MAX_ANGLE_DEG, theta_deg))

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _apply_angle(self) -> None:
        target_angle = kHoodMotor.HOME_ANGLE_DEG

        match self._state:
            case HoodStates.AUTO:
                target_angle = self.auto_hood_angle
            case HoodStates.MAX:
                target_angle = kHoodMotor.MAX_ANGLE_DEG
            case HoodStates.INNER_RING:
                target_angle = kHoodMotor.INNER_RING_ANGLE
            case HoodStates.OUTER_RING:
                target_angle = kHoodMotor.OUTER_RING_ANGLE
            case HoodStates.HIDE:
                target_angle = kHoodMotor.HOME_ANGLE_DEG

        target_revs = kHoodMotor.to_revs(target_angle)
        self.set_position(target_revs)

    # ------------------------------------------------------------------
    # Periodic
    # ------------------------------------------------------------------

    def periodic(self) -> None:
        if self.is_hard_stopped() and not self.hard_stopped:
            self._motor.set_position(kHoodMotor.to_revs(kHoodMotor.HOME_ANGLE_DEG))
        self.hard_stopped = self.is_hard_stopped()

        if self._state != HoodStates.RESETTING and self._state != HoodStates.NONE:
            self._apply_angle()

        self.nt.set(
            "Hood Position (deg)",
            round(kHoodMotor.to_deg(self._motor.get_position().value), 1),
        )
        self.nt.set("Target Angle (deg)", kHoodMotor.to_deg(self._target_position_revs))
        self.nt.set("State", self._state.name)

        self.editable_pid.periodic()

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def simulationPeriodic(self):
        self._motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        self._arm_sim.setInputVoltage(self._motor.sim_state.motor_voltage)
        self._arm_sim.update(0.02)

        arm_rad = self._arm_sim.getAngle()
        arm_rps_motor = (
            self._arm_sim.getVelocity() / (2 * math.pi)
        ) * kHoodMotor.SIM_GEARING

        self._motor.sim_state.set_rotor_velocity(arm_rps_motor)
        self._motor.sim_state.add_rotor_position(arm_rps_motor * 0.02)

        # Simulate limit switches at physical stops
        tol = degreesToRadians(1.0)
        self._motor.sim_state.set_forward_limit(
            arm_rad >= degreesToRadians(kHoodMotor.MAX_ANGLE_DEG) - tol
        )
        self._motor.sim_state.set_reverse_limit(
            arm_rad <= degreesToRadians(kHoodMotor.MIN_ANGLE_DEG) + tol
        )
