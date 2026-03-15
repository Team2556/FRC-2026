# """
# Test Program Command for FRC 2026 Prototype Robot

# This test program safely tests all robot subsystems with the following features:
# - IMU tilt monitoring (pitch and roll measurements)
# - LED status display showing test state and tilt warnings
# - Safe motor tests with time limits for motors that move physical mechanisms
# - Comprehensive subsystem validation

# Motors are categorized as:
# - HAZARD: Hood, Intake Deployers, Transfer motors (limited to brief movements)
# - SAFE: Shooter motors (can run longer for spin-up tests)
# """

# import commands2
# import wpilib
# from wpimath import units
# import math

# from phoenix6.hardware.pigeon2 import Pigeon2

# from subsystems.drivetrain.swerve_tuner import TunerConstants
# from subsystems.led.LED_helpers import ColorFactories, CANdle_Color
# from constants.canbus import kCANId
# from constants.led import kLED


# class TestProgramCommand(commands2.Command):
#     """
#     Comprehensive test program that cycles through all robot subsystems
#     with safety timeouts and visual feedback via LEDs.
#     """

#     # Test sequence timing (in seconds)
#     PHASE_DURATION = 1.5  # Duration of each test phase
#     HAZARD_MOTOR_TIME = 0.25  # Maximum time for hazard motors (hood, intake deployer, transfer)
#     SAFE_MOTOR_TIME = 1.0  # Maximum time for safe motors (shooter)

#     # Tilt thresholds (in degrees)
#     TILT_WARNING_THRESHOLD = 5.0  # Yellow warning
#     TILT_ERROR_THRESHOLD = 15.0  # Red error

#     def __init__(self,
#                  intake_subsystem,
#                  transfer_subsystem,
#                  shooter_subsystem,
#                  hood_subsystem,
#                  led_controller):
#         super().__init__()

#         self.intake = intake_subsystem
#         self.transfer = transfer_subsystem
#         self.shooter = shooter_subsystem
#         self.hood = hood_subsystem
#         self.led = led_controller

#         # Create Pigeon2 IMU for tilt monitoring
#         self.pigeon = Pigeon2(TunerConstants._pigeon_id)

#         # Test state tracking
#         self.phase_timer = wpilib.Timer()
#         self.motor_timer = wpilib.Timer()
#         self.current_phase = 0
#         self.total_phases = 8

#         # Tilt measurements
#         self.pitch = 0.0
#         self.roll = 0.0
#         self.tilt = 0.0

#         # LED state handlers
#         self.tilt_state = None
#         self.test_state = None

#         # Add subsystem requirements
#         self.addRequirements(
#             self.intake,
#             self.transfer,
#             self.shooter,
#             self.hood,
#             self.led
#         )

#     def initialize(self):
#         """Called when the command is initially scheduled."""
#         wpilib.SmartDashboard.putString("Test Program", "INITIALIZING")
#         wpilib.SmartDashboard.putNumber("Test Phase", 0)

#         self.current_phase = 0
#         self.phase_timer.restart()
#         self.motor_timer.restart()

#         # Create LED state for initialization (blue)
#         self.test_state = self.led.create_state(
#             state_key="test_program",
#             animation_request=ColorFactories.solid_color(CANdle_Color.BLUE),
#             priority=100,
#             enable=True
#         )

#         print("=== Test Program Started ===")
#         print(f"Total phases: {self.total_phases}")
#         print("Press DISABLE to stop the test at any time")

#     def execute(self):
#         """Called every 20ms while the command is scheduled."""
#         # Read IMU data
#         self.pitch = self.pigeon.get_pitch().value
#         self.roll = self.pigeon.get_roll().value
#         self.tilt = math.sqrt(self.pitch**2 + self.roll**2)

#         # Update tilt display
#         wpilib.SmartDashboard.putNumber("IMU Pitch", self.pitch)
#         wpilib.SmartDashboard.putNumber("IMU Roll", self.roll)
#         wpilib.SmartDashboard.putNumber("IMU Tilt", self.tilt)

#         # Check if phase is complete
#         if self.phase_timer.hasElapsed(self.PHASE_DURATION):
#             self.current_phase += 1
#             self.phase_timer.restart()
#             self.motor_timer.restart()

#             if self.current_phase < self.total_phases:
#                 print(f"\n=== Phase {self.current_phase}/{self.total_phases} ===")

#         # Update phase display
#         wpilib.SmartDashboard.putNumber("Test Phase", self.current_phase)

#         # Execute current test phase
#         if self.current_phase == 0:
#             self._phase_idle()
#         elif self.current_phase == 1:
#             self._phase_imu_check()
#         elif self.current_phase == 2:
#             self._phase_led_test()
#         elif self.current_phase == 3:
#             self._phase_shooter_test()
#         elif self.current_phase == 4:
#             self._phase_hood_test()
#         elif self.current_phase == 5:
#             self._phase_intake_test()
#         elif self.current_phase == 6:
#             self._phase_transfer_test()
#         elif self.current_phase == 7:
#             self._phase_complete()

#         # Update LED based on tilt (unless in LED test phase)
#         if self.current_phase != 2:
#             self._update_tilt_leds()

#     def _phase_idle(self):
#         """Phase 0: Initial idle phase"""
#         wpilib.SmartDashboard.putString("Test Program", "IDLE - Starting Tests")

#         # All motors off
#         self.shooter.disable()
#         self.hood.set_speed(0)
#         self.intake.set_deployer_position(0)
#         self.intake.set_spinny_speed(0)
#         self.transfer.stop()

#     def _phase_imu_check(self):
#         """Phase 1: IMU tilt measurement check"""
#         wpilib.SmartDashboard.putString("Test Program", "IMU CHECK")

#         # Print once at start of phase
#         if self.phase_timer.get() < 0.1:
#             print(f"  Monitoring IMU tilt for {self.PHASE_DURATION}s...")

#         # Print periodic updates
#         if int(self.phase_timer.get() * 10) % 5 == 0:
#             print(f"  Pitch: {self.pitch:.2f}° | Roll: {self.roll:.2f}° | Tilt: {self.tilt:.2f}°")

#         # All motors remain off during IMU check
#         self.shooter.disable()
#         self.hood.set_speed(0)
#         self.intake.set_deployer_position(0)
#         self.intake.set_spinny_speed(0)
#         self.transfer.stop()

#     def _phase_led_test(self):
#         """Phase 2: LED display test - cycle through colors"""
#         wpilib.SmartDashboard.putString("Test Program", "LED TEST")

#         # Cycle through colors based on time in phase
#         time_in_phase = self.phase_timer.get()

#         if time_in_phase < 0.5:
#             if time_in_phase < 0.05:
#                 print("  LED: RED")
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.solid_color(CANdle_Color.RED),
#                 priority=100,
#                 enable=True
#             )
#         elif time_in_phase < 1.0:
#             if time_in_phase < 0.55:
#                 print("  LED: GREEN")
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.solid_color(CANdle_Color.GREEN),
#                 priority=100,
#                 enable=True
#             )
#         elif time_in_phase < 1.5:
#             if time_in_phase < 1.05:
#                 print("  LED: BLUE")
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.solid_color(CANdle_Color.BLUE),
#                 priority=100,
#                 enable=True
#             )
#         elif time_in_phase < 2.0:
#             if time_in_phase < 1.55:
#                 print("  LED: YELLOW")
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.solid_color(CANdle_Color.YELLOW),
#                 priority=100,
#                 enable=True
#             )
#         else:
#             if time_in_phase < 2.05:
#                 print("  LED: RAINBOW")
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.rainbow(brightness=1.0, speed=1.0),
#                 priority=100,
#                 enable=True
#             )

#     def _phase_shooter_test(self):
#         """Phase 3: Shooter motor test (can run for longer duration)"""
#         wpilib.SmartDashboard.putString("Test Program", "SHOOTER TEST")

#         motor_time = self.motor_timer.get()

#         if motor_time < self.SAFE_MOTOR_TIME:
#             # Run shooter
#             self.shooter.enable()
#             if motor_time < 0.1:
#                 print(f"  Shooter: ENABLED (will run for {self.SAFE_MOTOR_TIME}s)")
#         else:
#             self.shooter.disable()
#             if motor_time < self.SAFE_MOTOR_TIME + 0.1:
#                 print("  Shooter: DISABLED")

#     def _phase_hood_test(self):
#         """Phase 4: Hood motor test (HAZARD - brief movement only)"""
#         wpilib.SmartDashboard.putString("Test Program", "HOOD TEST (HAZARD)")

#         motor_time = self.motor_timer.get()

#         if motor_time < self.HAZARD_MOTOR_TIME:
#             # Brief movement to test hood control
#             self.hood.set_speed(0.1)  # Very slow speed
#             if motor_time < 0.1:
#                 print(f"  Hood: MOVING at 10% speed ({self.HAZARD_MOTOR_TIME}s max)")
#         else:
#             self.hood.set_speed(0)
#             if motor_time < self.HAZARD_MOTOR_TIME + 0.1:
#                 print("  Hood: STOPPED")

#     def _phase_intake_test(self):
#         """Phase 5: Intake deployer test (HAZARD - brief movement only)"""
#         wpilib.SmartDashboard.putString("Test Program", "INTAKE TEST (HAZARD)")

#         motor_time = self.motor_timer.get()

#         if motor_time < self.HAZARD_MOTOR_TIME:
#             # Brief deployment movement
#             self.intake.set_deployer_position(0.1)  # Small position change
#             # Don't run spinner during test
#             self.intake.set_spinny_speed(0)
#             if motor_time < 0.1:
#                 print(f"  Intake: DEPLOYING to 0.1 position ({self.HAZARD_MOTOR_TIME}s max)")
#         else:
#             self.intake.set_deployer_position(0)
#             self.intake.set_spinny_speed(0)
#             if motor_time < self.HAZARD_MOTOR_TIME + 0.1:
#                 print("  Intake: STOPPED and retracted")

#     def _phase_transfer_test(self):
#         """Phase 6: Transfer motor test (HAZARD - brief activation only)"""
#         wpilib.SmartDashboard.putString("Test Program", "TRANSFER TEST (HAZARD)")

#         motor_time = self.motor_timer.get()

#         if motor_time < self.HAZARD_MOTOR_TIME:
#             # Brief transfer activation
#             self.transfer.activate()
#             if motor_time < 0.1:
#                 print(f"  Transfer: RUNNING ({self.HAZARD_MOTOR_TIME}s max)")
#         else:
#             self.transfer.stop()
#             if motor_time < self.HAZARD_MOTOR_TIME + 0.1:
#                 print("  Transfer: STOPPED")

#     def _phase_complete(self):
#         """Phase 7: Test complete"""
#         wpilib.SmartDashboard.putString("Test Program", "COMPLETE")

#         # All motors off
#         self.shooter.disable()
#         self.hood.set_speed(0)
#         self.intake.set_deployer_position(0)
#         self.intake.set_spinny_speed(0)
#         self.transfer.stop()

#         # Display completion on LEDs (green)
#         self.test_state = self.led.create_state(
#             state_key="test_program",
#             animation_request=ColorFactories.solid_color(CANdle_Color.GREEN),
#             priority=100,
#             enable=True
#         )

#         if self.phase_timer.get() < 0.1:
#             print("\n=== Test Program Complete ===")
#             print(f"Final Tilt: {self.tilt:.2f}°")
#             print("All motors stopped")

#     def _update_tilt_leds(self):
#         """Update LED color based on IMU tilt measurements"""
#         if self.tilt > self.TILT_ERROR_THRESHOLD:
#             # Red strobe: Excessive tilt
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.strobe(CANdle_Color.RED, speed=0.5),
#                 priority=100,
#                 enable=True
#             )
#         elif self.tilt > self.TILT_WARNING_THRESHOLD:
#             # Yellow: Warning tilt
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.solid_color(CANdle_Color.YELLOW),
#                 priority=100,
#                 enable=True
#             )
#         else:
#             # Green: Minimal tilt (good)
#             self.test_state = self.led.create_state(
#                 state_key="test_program",
#                 animation_request=ColorFactories.solid_color(CANdle_Color.GREEN),
#                 priority=100,
#                 enable=True
#             )

#     def isFinished(self) -> bool:
#         """Command completes after all phases are done."""
#         return self.current_phase >= self.total_phases

#     def end(self, interrupted: bool):
#         """Called when the command ends."""
#         # Ensure all motors are stopped
#         self.shooter.disable()
#         self.hood.set_speed(0)
#         self.intake.set_deployer_position(0)
#         self.intake.set_spinny_speed(0)
#         self.transfer.stop()

#         # Disable test LED state
#         if self.test_state:
#             self.test_state.disable()

#         if interrupted:
#             wpilib.SmartDashboard.putString("Test Program", "INTERRUPTED")
#             print("\n=== Test Program INTERRUPTED ===")
#         else:
#             wpilib.SmartDashboard.putString("Test Program", "FINISHED")
#             print("=== Test Program FINISHED ===")

#         wpilib.SmartDashboard.putNumber("Test Phase", 0)
