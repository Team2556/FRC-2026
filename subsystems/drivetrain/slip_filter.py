import math
from dataclasses import dataclass

import wpilib

from phoenix6.hardware.pigeon2 import Pigeon2

from wpimath.geometry import Pose2d, Rotation2d

from util.nt_util import NTTable


@dataclass
class SlipDetectorConfig:
    # Physics limits — tune these on the practice field
    max_credible_accel_mps2: float = 6.0       # ~0.6g, generous for FRC
    max_credible_jerk_mps3: float = 40.0       # accel change per second
    max_wheel_delta_per_cycle_m: float = 0.08  # ~3in per 20ms = ~4 m/s max

    # Recovery behavior
    slip_cooldown_cycles: int = 10             # cycles to stay in recovery mode

    # IMU noise floor — ignore accel below this to avoid false positives at rest
    imu_accel_noise_floor_mps2: float = 0.3


@dataclass
class SlipEvent:
    timestamp: float
    trigger: str           # "accel", "jerk", or "wheel_delta"
    magnitude: float       # how far over the threshold
    pre_slip_pose: Pose2d


class WheelSlipOdometryFilter:
    """Monitors CTRE swerve odometry for physically impossible wheel deltas and
    IMU acceleration spikes. When slip is detected, injects a corrected pose
    via add_vision_measurement() using short-window IMU dead reckoning."""

    GRAVITY = 9.81  # m/s²

    def __init__(
        self,
        config: SlipDetectorConfig,
        ctre_drivetrain,  # CommandSwerveDrivetrain (has .pigeon2, .add_vision_measurement, .get_state)
    ):
        self.config = config
        self._drivetrain = ctre_drivetrain
        self._pigeon: Pigeon2 = ctre_drivetrain.pigeon2

        # Cache acceleration signals — refresh once per cycle
        self._accel_x_signal = self._pigeon.get_acceleration_x(False)
        self._accel_y_signal = self._pigeon.get_acceleration_y(False)

        # Per-module last positions (meters)
        self._last_module_positions: list[float] = [0.0] * 4
        self._positions_initialized = False

        # IMU state for jerk gate
        self._last_accel_magnitude = 0.0
        self._current_accel_magnitude = 0.0
        self._last_timestamp = 0.0

        # IMU dead-reckoning velocity (reset after cooldown)
        self._imu_velocity_x = 0.0
        self._imu_velocity_y = 0.0

        # Slip state
        self._cooldown_remaining = 0
        self._event_log: list[SlipEvent] = []
        self._total_slip_count = 0

        # Telemetry
        self.nt = NTTable("SlipFilter")
        self.nt.int("Slip Event Count", 0)
        self.nt.bool("In Slip Recovery", False)
        self.nt.string("Last Trigger", "none")
        self.nt.float("Last Magnitude", 0.0)
        # Tunable thresholds on dashboard
        self.nt.float("Max Wheel Delta (m)", config.max_wheel_delta_per_cycle_m)
        self.nt.float("Max Accel (m/s2)", config.max_credible_accel_mps2)
        self.nt.float("Max Jerk (m/s3)", config.max_credible_jerk_mps3)
        self.nt.int("Cooldown Cycles", config.slip_cooldown_cycles)

    def update(self) -> bool:
        """Called once per robot periodic cycle from SwerveDriveTrain.periodic().
        Returns True if a slip event was detected this cycle."""
        now = wpilib.Timer.getFPGATimestamp()
        dt = now - self._last_timestamp if self._last_timestamp > 0 else 0.02
        self._last_timestamp = now

        # Read tunable thresholds from dashboard
        self.config.max_wheel_delta_per_cycle_m = self.nt.get("Max Wheel Delta (m)")
        self.config.max_credible_accel_mps2 = self.nt.get("Max Accel (m/s2)")
        self.config.max_credible_jerk_mps3 = self.nt.get("Max Jerk (m/s3)")
        self.config.slip_cooldown_cycles = int(self.nt.get("Cooldown Cycles"))

        state = self._drivetrain.get_state()
        pre_slip_pose = state.pose

        # --- Gate 1: Per-wheel displacement ---
        wheel_slip, wheel_mag = self._check_wheel_delta(state)

        # --- Gate 2: IMU acceleration ---
        accel_slip, accel_mag = self._check_imu_accel(dt)

        # --- Gate 3: Jerk (rate of acceleration change) ---
        jerk_slip, jerk_mag = self._check_jerk(dt)

        # --- Slip decision ---
        slip_detected = False
        trigger = ""
        magnitude = 0.0

        if wheel_slip:
            slip_detected = True
            trigger = "wheel_delta"
            magnitude = wheel_mag
        elif accel_slip and wheel_mag > self.config.max_wheel_delta_per_cycle_m * 0.5:
            # Only flag accel if wheels also show elevated deltas (avoids IMU-only false positives)
            slip_detected = True
            trigger = "accel"
            magnitude = accel_mag
        elif jerk_slip and wheel_mag > self.config.max_wheel_delta_per_cycle_m * 0.5:
            slip_detected = True
            trigger = "jerk"
            magnitude = jerk_mag

        if slip_detected:
            self._cooldown_remaining = self.config.slip_cooldown_cycles
            self._total_slip_count += 1

            event = SlipEvent(
                timestamp=now,
                trigger=trigger,
                magnitude=magnitude,
                pre_slip_pose=pre_slip_pose,
            )
            self._event_log.append(event)
            if len(self._event_log) > 100:
                self._event_log.pop(0)

            # Inject recovery pose via vision measurement path _FCC_
            recovery_pose = self._imu_dead_reckon(pre_slip_pose, dt)
            self._drivetrain.add_vision_measurement(
                recovery_pose,
                wpilib.Timer.getFPGATimestamp() - 0.02,  # one loop ago
                (0.1, 0.1, 0.05),  # tight std devs — trust IMU over slipping wheels
            )

            self.nt.set("Last Trigger", trigger)
            self.nt.set("Last Magnitude", round(magnitude, 4))

        # Cooldown tick
        if self._cooldown_remaining > 0:
            self._cooldown_remaining -= 1
        else:
            # Reset IMU velocity when cooldown expires to prevent drift accumulation _FCC_
            self._imu_velocity_x = 0.0
            self._imu_velocity_y = 0.0

        self.nt.set("Slip Event Count", self._total_slip_count)
        self.nt.set("In Slip Recovery", self._cooldown_remaining > 0)

        return slip_detected

    def _check_wheel_delta(self, state) -> tuple[bool, float]:
        """Gate 1: Check if any single module moved an impossible distance in one cycle.
        A single slipping wheel while others are stationary is the classic FRC slip signature."""
        module_positions = state.module_positions
        max_delta = 0.0

        if not self._positions_initialized:
            self._last_module_positions = [mp.distance for mp in module_positions]
            self._positions_initialized = True
            return False, 0.0

        for i, mp in enumerate(module_positions):
            delta = abs(mp.distance - self._last_module_positions[i])
            if delta > max_delta:
                max_delta = delta
            self._last_module_positions[i] = mp.distance

        return max_delta > self.config.max_wheel_delta_per_cycle_m, max_delta

    def _check_imu_accel(self, dt: float) -> tuple[bool, float]:
        """Gate 2: Check if IMU reports acceleration beyond what's physically possible."""
        self._accel_x_signal.refresh()
        self._accel_y_signal.refresh()

        # Pigeon2 reports in g's — convert to m/s²
        ax = self._accel_x_signal.value * self.GRAVITY
        ay = self._accel_y_signal.value * self.GRAVITY

        accel_mag = math.sqrt(ax * ax + ay * ay)

        # Accumulate IMU velocity for dead reckoning during slip events
        if accel_mag > self.config.imu_accel_noise_floor_mps2:
            self._imu_velocity_x += ax * dt
            self._imu_velocity_y += ay * dt

        self._current_accel_magnitude = accel_mag
        return accel_mag > self.config.max_credible_accel_mps2, accel_mag

    def _check_jerk(self, dt: float) -> tuple[bool, float]:
        """Gate 3: Check if acceleration changed too fast (onset/offset of slip)."""
        if dt <= 0:
            return False, 0.0

        jerk = abs(self._current_accel_magnitude - self._last_accel_magnitude) / dt
        self._last_accel_magnitude = self._current_accel_magnitude

        return jerk > self.config.max_credible_jerk_mps3, jerk

    def _imu_dead_reckon(self, prior_pose: Pose2d, dt: float) -> Pose2d:
        """Short-window IMU dead reckoning for recovery pose during slip.
        Only accurate for 5-15 cycles (~100-300ms). Pigeon yaw is trustworthy
        even during wheel slip since it's inertial, not encoder-based. _FCC_"""
        true_heading = self._pigeon.get_yaw().value  # degrees

        dx = self._imu_velocity_x * dt
        dy = self._imu_velocity_y * dt

        return Pose2d(
            prior_pose.X() + dx,
            prior_pose.Y() + dy,
            Rotation2d.fromDegrees(true_heading),
        )

    @property
    def in_recovery(self) -> bool:
        return self._cooldown_remaining > 0

    @property
    def slip_count(self) -> int:
        return self._total_slip_count

    def get_slip_event_log(self) -> list[SlipEvent]:
        return list(self._event_log)
