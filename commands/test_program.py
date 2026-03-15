"""
Test Program Command for FRC 2026 Prototype Robot

This test program tests all robot subsystems with encoder feedback verification:
- IMU tilt monitoring (pitch and roll measurements)
- LED status display showing test state and tilt warnings
- Motor tests with encoder verification to confirm actual movement
- Movement thresholds set low to avoid false failures
- Displays actual revolutions/movements achieved

Motors tested with encoder feedback:
- Hood: Position control verification (rotations)
- Intake: Deployer position changes (rotations)  
- Transfer: Motor revolution counting during activation
- Shooter: RPM verification during spin-up
- Drivetrain: Small movement test with module positions
"""

import commands2
import wpilib
from wpimath import units
from wpimath.geometry import Rotation2d
import math

from phoenix6.hardware.pigeon2 import Pigeon2

from subsystems.drivetrain.swerve_tuner import TunerConstants
from subsystems.led.LED_helpers import ColorFactories, CANdle_Color
from constants.canbus import kCANId
from constants.led import kLED


class TestProgramCommand(commands2.Command):
    """
    Comprehensive test program that cycles through all robot subsystems
    with encoder feedback verification and low movement thresholds.
    """

    # Test sequence timing (in seconds)
    PHASE_DURATION = 1.5  # Duration of each test phase
    HAZARD_MOTOR_TIME = 0.25  # Maximum time for hazard motors (hood, intake deployer, transfer)
    SAFE_MOTOR_TIME = 1.0  # Maximum time for safe motors (shooter)
    DRIVETRAIN_TIME = 0.5  # Time for drivetrain movement test

    # Tilt thresholds (in degrees)
    TILT_WARNING_THRESHOLD = 5.0  # Yellow warning
    TILT_ERROR_THRESHOLD = 15.0  # Red error
    
    # Movement verification thresholds (set low to avoid false failures)
    MIN_HOOD_MOVEMENT = 0.01  # rotations
    MIN_INTAKE_MOVEMENT = 0.005  # rotations  
    MIN_TRANSFER_MOVEMENT = 0.02  # rotations
    MIN_SHOOTER_RPM = 50  # RPM change
    MIN_DRIVETRAIN_MOVEMENT = 0.01  # rotations per module
    MIN_TURN_ANGLE = 5.0  # degrees per module

    def __init__(self,
                 drivetrain,
                 intake_subsystem,
                 transfer_subsystem,
                 shooter_subsystem,
                 hood_subsystem,
                 led_controller):
        super().__init__()

        self.drivetrain = drivetrain
        self.intake = intake_subsystem
        self.transfer = transfer_subsystem
        self.shooter = shooter_subsystem
        self.hood = hood_subsystem
        self.led = led_controller

        # Create Pigeon2 IMU for tilt monitoring
        self.pigeon = Pigeon2(TunerConstants._pigeon_id)

        # Test state tracking
        self.phase_timer = wpilib.Timer()
        self.motor_timer = wpilib.Timer()
        self.current_phase = 0
        self.total_phases = 10  # Added drivetrain and turn motor tests

        # Tilt measurements
        self.pitch = 0.0
        self.roll = 0.0
        self.tilt = 0.0

        # LED state handlers
        self.tilt_state = None
        self.test_state = None
        
        # Encoder position tracking for movement verification
        self.initial_hood_position = 0.0
        self.initial_intake_position = 0.0
        self.initial_transfer_spindexer_position = 0.0
        self.initial_transfer_up_position = 0.0
        self.initial_shooter_position = 0.0
        self.initial_drivetrain_positions = []
        self.initial_turn_angles = []
        
        # Movement results
        self.hood_movement_result = {"moved": 0.0, "passed": False}
        self.intake_movement_result = {"moved": 0.0, "passed": False}
        self.transfer_movement_result = {"spindexer_moved": 0.0, "up_moved": 0.0, "passed": False}
        self.shooter_movement_result = {"rpm_achieved": 0.0, "passed": False}
        self.drivetrain_movement_result = {"avg_moved": 0.0, "passed": False}
        self.turn_motor_result = {"avg_turned": 0.0, "passed": False}

        # Add subsystem requirements
        self.addRequirements(
            self.drivetrain,
            self.intake,
            self.transfer,
            self.shooter,
            self.hood,
            self.led
        )

    def initialize(self):
        """Called when the command is initially scheduled."""
        wpilib.SmartDashboard.putString("Test Program", "INITIALIZING")
        wpilib.SmartDashboard.putNumber("Test Phase", 0)

        self.current_phase = 0
        self.phase_timer.restart()
        self.motor_timer.restart()

        # Create LED state for initialization (blue)
        self.test_state = self.led.create_state(
            state_key="test_program",
            animation_request=ColorFactories.solid_color(CANdle_Color.BLUE),
            priority=100,
            enable=True
        )

        print("=== Test Program Started ===")
        print(f"Total phases: {self.total_phases}")
        print("Press DISABLE to stop the test at any time")

    def execute(self):
        """Called every 20ms while the command is scheduled."""
        # Read IMU data
        self.pitch = self.pigeon.get_pitch().value
        self.roll = self.pigeon.get_roll().value
        self.tilt = math.sqrt(self.pitch**2 + self.roll**2)

        # Update tilt display
        wpilib.SmartDashboard.putNumber("IMU Pitch", self.pitch)
        wpilib.SmartDashboard.putNumber("IMU Roll", self.roll)
        wpilib.SmartDashboard.putNumber("IMU Tilt", self.tilt)

        # Check if phase is complete
        if self.phase_timer.hasElapsed(self.PHASE_DURATION):
            self.current_phase += 1
            self.phase_timer.restart()
            self.motor_timer.restart()

            if self.current_phase < self.total_phases:
                print(f"\n=== Phase {self.current_phase}/{self.total_phases} ===")

        # Update phase display
        wpilib.SmartDashboard.putNumber("Test Phase", self.current_phase)

        # Execute current test phase
        if self.current_phase == 0:
            self._phase_idle()
        elif self.current_phase == 1:
            self._phase_imu_check()
        elif self.current_phase == 2:
            self._phase_led_test()
        elif self.current_phase == 3:
            self._phase_drivetrain_test()
        elif self.current_phase == 4:
            self._phase_turn_motor_test()
        elif self.current_phase == 5:
            self._phase_shooter_test()
        elif self.current_phase == 6:
            self._phase_hood_test()
        elif self.current_phase == 7:
            self._phase_intake_test()
        elif self.current_phase == 8:
            self._phase_transfer_test()
        elif self.current_phase == 9:
            self._phase_complete()

        # Update LED based on tilt (unless in LED test phase)
        if self.current_phase != 2:
            self._update_tilt_leds()

    def _phase_idle(self):
        """Phase 0: Initial idle phase"""
        wpilib.SmartDashboard.putString("Test Program", "IDLE - Starting Tests")

        # All motors off
        self.shooter.disable()
        self.hood.set_speed(0)
        self.intake.set_deployer_position(0)
        self.intake.set_spinny_speed(0)
        self.transfer.stop()

    def _phase_imu_check(self):
        """Phase 1: IMU tilt measurement check"""
        wpilib.SmartDashboard.putString("Test Program", "IMU CHECK")

        # Print once at start of phase
        if self.phase_timer.get() < 0.1:
            print(f"  Monitoring IMU tilt for {self.PHASE_DURATION}s...")

        # Print periodic updates
        if int(self.phase_timer.get() * 10) % 5 == 0:
            print(f"  Pitch: {self.pitch:.2f}° | Roll: {self.roll:.2f}° | Tilt: {self.tilt:.2f}°")

        # All motors remain off during IMU check
        self.shooter.disable()
        self.hood.set_speed(0)
        self.intake.set_deployer_position(0)
        self.intake.set_spinny_speed(0)
        self.transfer.stop()

    def _phase_led_test(self):
        """Phase 2: Simple LED test - no sensor feedback needed"""
        wpilib.SmartDashboard.putString("Test Program", "LED TEST")

        # Simple color cycle based on time
        time_in_phase = self.phase_timer.get()

        if time_in_phase < 0.3:
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.solid_color(CANdle_Color.RED),
                priority=100,
                enable=True
            )
        elif time_in_phase < 0.6:
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.solid_color(CANdle_Color.GREEN),
                priority=100,
                enable=True
            )
        elif time_in_phase < 0.9:
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.solid_color(CANdle_Color.BLUE),
                priority=100,
                enable=True
            )
        else:
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.rainbow(brightness=1.0, speed=1.0),
                priority=100,
                enable=True
            )

        if time_in_phase < 0.1:
            print("  LED Test: Visual confirmation only - cycling colors")

    def _phase_drivetrain_test(self):
        """Phase 3: Drivetrain encoder movement test"""
        wpilib.SmartDashboard.putString("Test Program", "DRIVETRAIN TEST")

        motor_time = self.motor_timer.get()

        if motor_time < 0.05:  # Capture initial positions
            state = self.drivetrain.get_state()
            self.initial_drivetrain_positions = []
            for i in range(4):  # Assuming 4 swerve modules
                try:
                    module_position = state.robot_state.module_positions[i]
                    distance = module_position.distance
                    self.initial_drivetrain_positions.append(distance)
                    print(f"  Module {i} initial position: {distance:.4f} meters")
                except (IndexError, AttributeError):
                    self.initial_drivetrain_positions.append(0.0)
            print(f"  Drivetrain: Small movement test starting...")

        elif motor_time < self.DRIVETRAIN_TIME:
            # Small forward movement
            self.drivetrain.drive_with_values(velocity_x=0.2, velocity_y=0, rotation_rate=0)

        elif motor_time < self.DRIVETRAIN_TIME + 0.1:  # Stop and measure
            self.drivetrain.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)
            
            # Measure movement
            state = self.drivetrain.get_state()
            total_movement = 0.0
            modules_moved = 0
            
            for i in range(min(4, len(self.initial_drivetrain_positions))):
                try:
                    module_position = state.robot_state.module_positions[i]
                    current_pos = module_position.distance
                    movement = abs(current_pos - self.initial_drivetrain_positions[i])
                    total_movement += movement
                    modules_moved += 1
                    print(f"  Module {i}: {self.initial_drivetrain_positions[i]:.4f} -> {current_pos:.4f} (moved {movement:.4f}m)")
                except (IndexError, AttributeError):
                    pass
            
            avg_movement = total_movement / max(1, modules_moved) if modules_moved > 0 else 0.0
            passed = avg_movement > self.MIN_DRIVETRAIN_MOVEMENT
            
            self.drivetrain_movement_result = {"avg_moved": avg_movement, "passed": passed}
            
            wpilib.SmartDashboard.putNumber("Drivetrain Avg Movement", avg_movement)
            wpilib.SmartDashboard.putBoolean("Drivetrain Test Passed", passed)
            
            print(f"  Drivetrain: Average movement {avg_movement:.4f}m ({'PASS' if passed else 'FAIL'} - min: {self.MIN_DRIVETRAIN_MOVEMENT})")

        else:
            # Ensure stopped
            self.drivetrain.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)

    def _phase_turn_motor_test(self):
        """Phase 4: Turn/steer motor test with angle verification"""
        wpilib.SmartDashboard.putString("Test Program", "TURN MOTOR TEST")

        motor_time = self.motor_timer.get()

        if motor_time < 0.05:  # Capture initial angles
            state = self.drivetrain.get_state()
            self.initial_turn_angles = []
            for i in range(4):  # Assuming 4 swerve modules
                try:
                    module_state = state.robot_state.module_states[i]
                    angle_deg = module_state.angle.degrees()
                    self.initial_turn_angles.append(angle_deg)
                    print(f"  Module {i} initial angle: {angle_deg:.2f} degrees")
                except (IndexError, AttributeError):
                    self.initial_turn_angles.append(0.0)
            print(f"  Turn Motors: Pointing wheels to 45° test starting...")

        elif motor_time < self.DRIVETRAIN_TIME:
            # Point wheels at 45 degree angle
            target_angle = Rotation2d.fromDegrees(45.0)
            self.drivetrain._drivetrain.set_control(
                self.drivetrain._point.with_module_direction(target_angle)
            )
            
        elif motor_time < self.DRIVETRAIN_TIME + 0.1:  # Stop and measure
            self.drivetrain.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)
            
            # Measure angle changes
            state = self.drivetrain.get_state()
            total_angle_change = 0.0
            modules_moved = 0
            
            for i in range(min(4, len(self.initial_turn_angles))):
                try:
                    module_state = state.robot_state.module_states[i]
                    current_angle = module_state.angle.degrees()
                    angle_change = abs(current_angle - self.initial_turn_angles[i])
                    
                    # Handle angle wraparound (e.g. 350° to 10° = 20° change, not 340°)
                    if angle_change > 180:
                        angle_change = 360 - angle_change
                    
                    total_angle_change += angle_change
                    modules_moved += 1
                    print(f"  Module {i}: {self.initial_turn_angles[i]:.2f}° -> {current_angle:.2f}° (turned {angle_change:.2f}°)")
                except (IndexError, AttributeError):
                    pass
            
            avg_angle_change = total_angle_change / max(1, modules_moved) if modules_moved > 0 else 0.0
            passed = avg_angle_change > self.MIN_TURN_ANGLE
            
            self.turn_motor_result = {"avg_turned": avg_angle_change, "passed": passed}
            
            wpilib.SmartDashboard.putNumber("Turn Motor Avg Angle Change", avg_angle_change)
            wpilib.SmartDashboard.putBoolean("Turn Motor Test Passed", passed)
            
            print(f"  Turn Motors: Average angle change {avg_angle_change:.2f}° ({'PASS' if passed else 'FAIL'} - min: {self.MIN_TURN_ANGLE}°)")

        else:
            # Ensure stopped
            self.drivetrain.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)

    def _phase_shooter_test(self):
        """Phase 4: Shooter motor test with RPM verification"""
        wpilib.SmartDashboard.putString("Test Program", "SHOOTER TEST")

        motor_time = self.motor_timer.get()

        if motor_time < 0.05:  # Capture initial RPM
            try:
                self.initial_shooter_position = self.shooter._top_motor.get_velocity().value * 60  # Convert to RPM
                print(f"  Shooter: Initial RPM: {self.initial_shooter_position:.1f}")
            except (AttributeError, Exception) as e:
                self.initial_shooter_position = 0.0
                print(f"  Shooter: Could not read initial RPM: {e}")

        if motor_time < self.SAFE_MOTOR_TIME:
            # Run shooter
            self.shooter.enable()
            if motor_time < 0.1:
                print(f"  Shooter: ENABLED (will run for {self.SAFE_MOTOR_TIME}s)")
                
        elif motor_time < self.SAFE_MOTOR_TIME + 0.1:  # Stop and measure
            self.shooter.disable()
            
            # Measure RPM change
            try:
                current_rpm = self.shooter._top_motor.get_velocity().value * 60  # Convert to RPM
                rpm_achieved = abs(current_rpm - self.initial_shooter_position)
                passed = rpm_achieved > self.MIN_SHOOTER_RPM
                
                self.shooter_movement_result = {"rpm_achieved": rpm_achieved, "passed": passed}
                
                wpilib.SmartDashboard.putNumber("Shooter RPM Change", rpm_achieved)
                wpilib.SmartDashboard.putBoolean("Shooter Test Passed", passed)
                
                print(f"  Shooter: {self.initial_shooter_position:.1f} -> {current_rpm:.1f} RPM (change: {rpm_achieved:.1f} {'PASS' if passed else 'FAIL'} - min: {self.MIN_SHOOTER_RPM})")
            except (AttributeError, Exception) as e:
                print(f"  Shooter: Could not read final RPM: {e}")
                self.shooter_movement_result = {"rpm_achieved": 0.0, "passed": False}
        else:
            self.shooter.disable()

    def _phase_hood_test(self):
        """Phase 5: Hood position test with encoder verification"""
        wpilib.SmartDashboard.putString("Test Program", "HOOD TEST with verification")

        motor_time = self.motor_timer.get()

        if motor_time < 0.05:  # Capture initial position
            try:
                self.initial_hood_position = self.hood.hood_motor.get_position().value
                print(f"  Hood: Initial position: {self.initial_hood_position:.4f} rotations")
            except (AttributeError, Exception) as e:
                self.initial_hood_position = 0.0
                print(f"  Hood: Could not read initial position: {e}")

        if motor_time < self.HAZARD_MOTOR_TIME:
            # Small position movement
            target_position = self.initial_hood_position + 0.05  # 0.05 rotations
            self.hood.set_position(target_position)
            if motor_time < 0.1:
                print(f"  Hood: Moving to position {target_position:.4f} ({self.HAZARD_MOTOR_TIME}s max)")
                
        elif motor_time < self.HAZARD_MOTOR_TIME + 0.2:  # Stop and measure
            self.hood.set_speed(0)
            
            # Measure position change
            try:
                current_position = self.hood.hood_motor.get_position().value
                movement = abs(current_position - self.initial_hood_position)
                passed = movement > self.MIN_HOOD_MOVEMENT
                
                self.hood_movement_result = {"moved": movement, "passed": passed}
                
                wpilib.SmartDashboard.putNumber("Hood Movement", movement)
                wpilib.SmartDashboard.putBoolean("Hood Test Passed", passed)
                
                print(f"  Hood: {self.initial_hood_position:.4f} -> {current_position:.4f} rotations (moved {movement:.4f} {'PASS' if passed else 'FAIL'} - min: {self.MIN_HOOD_MOVEMENT})")
            except AttributeError:
                print("  Hood: Could not read final position")
                self.hood_movement_result = {"moved": 0.0, "passed": False}
        else:
            self.hood.set_speed(0)

    def _phase_intake_test(self):
        """Phase 6: Intake deployer test with encoder verification"""
        wpilib.SmartDashboard.putString("Test Program", "INTAKE TEST with verification")

        motor_time = self.motor_timer.get()

        if motor_time < 0.05:  # Capture initial position
            try:
                self.initial_intake_position = self.intake.left_deployer.get_position().value
                print(f"  Intake: Initial position: {self.initial_intake_position:.4f} rotations")
            except (AttributeError, Exception) as e:
                self.initial_intake_position = 0.0
                print(f"  Intake: Could not read initial position: {e}")

        if motor_time < self.HAZARD_MOTOR_TIME:
            # Small deployment movement
            target_position = self.initial_intake_position + 0.02  # 0.02 rotations
            self.intake.set_deployer_position(target_position)
            # Don't run spinner during test
            self.intake.set_spinny_speed(0)
            if motor_time < 0.1:
                print(f"  Intake: Moving to position {target_position:.4f} ({self.HAZARD_MOTOR_TIME}s max)")
                
        elif motor_time < self.HAZARD_MOTOR_TIME + 0.2:  # Stop and measure
            self.intake.set_deployer_position(self.initial_intake_position)  # Return to initial
            self.intake.set_spinny_speed(0)
            
            # Allow time for movement, then measure
            if motor_time > self.HAZARD_MOTOR_TIME + 0.1:
                try:
                    current_position = self.intake.left_deployer.get_position().value
                    movement = abs(current_position - self.initial_intake_position)
                    passed = movement > self.MIN_INTAKE_MOVEMENT
                    
                    self.intake_movement_result = {"moved": movement, "passed": passed}
                    
                    wpilib.SmartDashboard.putNumber("Intake Movement", movement)
                    wpilib.SmartDashboard.putBoolean("Intake Test Passed", passed)
                    
                    print(f"  Intake: Max movement achieved: {movement:.4f} rotations ({'PASS' if passed else 'FAIL'} - min: {self.MIN_INTAKE_MOVEMENT})")
                except AttributeError:
                    print("  Intake: Could not read final position")
                    self.intake_movement_result = {"moved": 0.0, "passed": False}
        else:
            self.intake.set_deployer_position(self.initial_intake_position)
            self.intake.set_spinny_speed(0)

    def _phase_transfer_test(self):
        """Phase 7: Transfer motor test with encoder verification"""
        wpilib.SmartDashboard.putString("Test Program", "TRANSFER TEST with verification")

        motor_time = self.motor_timer.get()

        if motor_time < 0.05:  # Capture initial positions
            try:
                self.initial_transfer_spindexer_position = self.transfer.spindex_motor.get_position().value
                self.initial_transfer_up_position = self.transfer.up_transfer_motor.get_position().value
                print(f"  Transfer: Initial - Spindexer: {self.initial_transfer_spindexer_position:.4f}, Up: {self.initial_transfer_up_position:.4f} rotations")
            except (AttributeError, Exception) as e:
                self.initial_transfer_spindexer_position = 0.0
                self.initial_transfer_up_position = 0.0
                print(f"  Transfer: Could not read initial positions: {e}")

        if motor_time < self.HAZARD_MOTOR_TIME:
            # Brief transfer activation
            self.transfer.activate()
            if motor_time < 0.1:
                print(f"  Transfer: RUNNING ({self.HAZARD_MOTOR_TIME}s max)")
                
        elif motor_time < self.HAZARD_MOTOR_TIME + 0.1:  # Stop and measure
            self.transfer.stop()
            
            # Measure position changes
            try:
                current_spindexer = self.transfer.spindex_motor.get_position().value
                current_up = self.transfer.up_transfer_motor.get_position().value
                
                spindexer_movement = abs(current_spindexer - self.initial_transfer_spindexer_position)
                up_movement = abs(current_up - self.initial_transfer_up_position)
                
                # Pass if either motor moved sufficiently
                passed = (spindexer_movement > self.MIN_TRANSFER_MOVEMENT) or (up_movement > self.MIN_TRANSFER_MOVEMENT)
                
                self.transfer_movement_result = {
                    "spindexer_moved": spindexer_movement, 
                    "up_moved": up_movement, 
                    "passed": passed
                }
                
                wpilib.SmartDashboard.putNumber("Transfer Spindexer Movement", spindexer_movement)
                wpilib.SmartDashboard.putNumber("Transfer Up Movement", up_movement)
                wpilib.SmartDashboard.putBoolean("Transfer Test Passed", passed)
                
                print(f"  Transfer: Spindexer {spindexer_movement:.4f}, Up {up_movement:.4f} rotations ({'PASS' if passed else 'FAIL'} - min: {self.MIN_TRANSFER_MOVEMENT})")
            except (AttributeError, Exception) as e:
                print(f"  Transfer: Could not read final positions: {e}")
                self.transfer_movement_result = {"spindexer_moved": 0.0, "up_moved": 0.0, "passed": False}
        else:
            self.transfer.stop()

    def _phase_complete(self):
        """Phase 8: Test complete with results summary"""
        wpilib.SmartDashboard.putString("Test Program", "COMPLETE - Check Results")

        # All motors off
        self.drivetrain.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)
        self.shooter.disable()
        self.hood.set_speed(0)
        self.intake.set_deployer_position(0)
        self.intake.set_spinny_speed(0)
        self.transfer.stop()

        # Count passed tests
        tests_passed = 0
        total_tests = 6
        
        if self.drivetrain_movement_result["passed"]:
            tests_passed += 1
        if self.turn_motor_result["passed"]:
            tests_passed += 1
        if self.shooter_movement_result["passed"]:
            tests_passed += 1
        if self.hood_movement_result["passed"]:
            tests_passed += 1
        if self.intake_movement_result["passed"]:
            tests_passed += 1
        if self.transfer_movement_result["passed"]:
            tests_passed += 1

        # Display overall result on LEDs
        if tests_passed == total_tests:
            # All tests passed!
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.rainbow(brightness=1.0, speed=0.5),
                priority=100,
                enable=True
            )
        elif tests_passed >= 4:  # Most tests passed
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.solid_color(CANdle_Color.YELLOW),
                priority=100,
                enable=True
            )
        else:
            # Tests failed!
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.solid_color(CANdle_Color.RED),
                priority=100,
                enable=True
            )

        if self.phase_timer.get() < 0.1:
            print("\n=== Test Program Complete - Results Summary ===")
            print(f"Tests Passed: {tests_passed}/{total_tests}")
            print(f"Final Tilt: {self.tilt:.2f}°")
            print("\n--- Movement Results ---")
            print(f"Drivetrain: {self.drivetrain_movement_result['avg_moved']:.4f}m avg ({'PASS' if self.drivetrain_movement_result['passed'] else 'FAIL'})")
            print(f"Turn Motors: {self.turn_motor_result['avg_turned']:.2f}° avg ({'PASS' if self.turn_motor_result['passed'] else 'FAIL'})")
            print(f"Shooter: {self.shooter_movement_result['rpm_achieved']:.1f} RPM change ({'PASS' if self.shooter_movement_result['passed'] else 'FAIL'})")
            print(f"Hood: {self.hood_movement_result['moved']:.4f} rotations ({'PASS' if self.hood_movement_result['passed'] else 'FAIL'})")
            print(f"Intake: {self.intake_movement_result['moved']:.4f} rotations ({'PASS' if self.intake_movement_result['passed'] else 'FAIL'})")
            print(f"Transfer: S:{self.transfer_movement_result['spindexer_moved']:.4f}, U:{self.transfer_movement_result['up_moved']:.4f} rotations ({'PASS' if self.transfer_movement_result['passed'] else 'FAIL'})")
            print("\nAll motors stopped")
            print("=" * 50)

    def _update_tilt_leds(self):
        """Update LED color based on IMU tilt measurements"""
        if self.tilt > self.TILT_ERROR_THRESHOLD:
            # Red strobe: Excessive tilt
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.strobe(CANdle_Color.RED, speed=0.5),
                priority=100,
                enable=True
            )
        elif self.tilt > self.TILT_WARNING_THRESHOLD:
            # Yellow: Warning tilt
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.solid_color(CANdle_Color.YELLOW),
                priority=100,
                enable=True
            )
        else:
            # Green: Minimal tilt (good)
            self.test_state = self.led.create_state(
                state_key="test_program",
                animation_request=ColorFactories.solid_color(CANdle_Color.GREEN),
                priority=100,
                enable=True
            )

    def isFinished(self) -> bool:
        """Command completes after all phases are done."""
        return self.current_phase >= self.total_phases

    def end(self, interrupted: bool):
        """Called when the command ends."""
        # Ensure all motors are stopped
        self.drivetrain.drive_with_values(velocity_x=0, velocity_y=0, rotation_rate=0)
        self.shooter.disable()
        self.hood.set_speed(0)
        self.intake.set_deployer_position(0)
        self.intake.set_spinny_speed(0)
        self.transfer.stop()

        # Disable test LED state
        if self.test_state:
            self.test_state.disable()

        if interrupted:
            wpilib.SmartDashboard.putString("Test Program", "INTERRUPTED")
            print("\n=== Test Program INTERRUPTED ===")
        else:
            wpilib.SmartDashboard.putString("Test Program", "FINISHED")
            print("=== Test Program FINISHED ===")

        wpilib.SmartDashboard.putNumber("Test Phase", 0)
