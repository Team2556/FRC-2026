"""
System Test Command for Pit Testing

This command runs a comprehensive test of all robot subsystems.
It should be run with the robot on a stand where wheels don't touch anything,
intake has room to deploy, and shooter/hopper is empty.

The test will:
1. Test each motor subsystem by spinning and monitoring RPM
2. Verify vision system is responding
3. Check for resistance that indicates safety issues (ball in hopper, blocked intake)
4. Report results via SmartDashboard
"""

import commands2
from wpilib import SmartDashboard
import time


class SystemTestCommand(commands2.Command):
    """
    Sequential system test for all robot subsystems.
    Tests motors, vision, and safety systems.
    """

    def __init__(
        self,
        intake_motor,
        shooter_motor,
        spindex_motor,
        transfer_motor1,
        transfer_motor2,
        vision_subsystem,
    ):
        super().__init__()

        self._intake = intake_motor
        self._shooter = shooter_motor
        self._spindex = spindex_motor
        self._transfer1 = transfer_motor1
        self._transfer2 = transfer_motor2
        self._vision = vision_subsystem

        # Add requirements for all subsystems we'll test
        self.addRequirements(
            intake_motor,
            shooter_motor,
            spindex_motor,
            transfer_motor1,
            transfer_motor2,
            vision_subsystem,
        )

        # Test state tracking
        self._test_stage = 0
        self._stage_start_time = 0
        self._test_passed = True
        self._failure_message = ""

        # Test configuration
        self.MOTOR_TEST_DURATION = 2.0  # seconds to run each motor
        self.RPM_TOLERANCE = 0.2  # Allow 20% tolerance from target
        self.MIN_RPM_THRESHOLD = 100  # Minimum RPM to consider motor working
        self.STALL_CHECK_DELAY = 0.5  # seconds before checking for stall

    def initialize(self):
        """Called when the command is first scheduled."""
        self._test_stage = 0
        self._stage_start_time = time.time()
        self._test_passed = True
        self._failure_message = ""

        SmartDashboard.putString("System Test Status", "Starting...")
        SmartDashboard.putString("System Test Stage", "Initialization")
        SmartDashboard.putBoolean("System Test Passed", False)
        SmartDashboard.putString("System Test Failure", "")

    def execute(self):
        """Called periodically while the command is scheduled."""
        current_time = time.time()
        elapsed_time = current_time - self._stage_start_time

        # Stage 0: Test Intake Motor
        if self._test_stage == 0:
            SmartDashboard.putString("System Test Stage", "Testing Intake Motor")
            if elapsed_time < self.MOTOR_TEST_DURATION:
                self._intake.spin()
                # After initial spin-up, check for stall
                if elapsed_time > self.STALL_CHECK_DELAY:
                    rpm = abs(self._intake._motor.get_velocity().value * 60)
                    if rpm < self.MIN_RPM_THRESHOLD:
                        self._test_passed = False
                        self._failure_message = "Intake motor stalled - possible obstruction"
                        self._intake.stop_motor()
            else:
                # Check final RPM before moving on
                rpm = abs(self._intake._motor.get_velocity().value * 60)
                target_rpm = abs(self._intake._RPS * 60)
                if rpm < target_rpm * self.RPM_TOLERANCE:
                    self._test_passed = False
                    self._failure_message = f"Intake motor below target RPM (got {rpm:.0f}, expected ~{target_rpm:.0f})"
                self._intake.stop_motor()
                self._advance_stage()

        # Stage 1: Test Shooter Motor
        elif self._test_stage == 1:
            SmartDashboard.putString("System Test Stage", "Testing Shooter Motor")
            if elapsed_time < self.MOTOR_TEST_DURATION:
                self._shooter.spin()
                if elapsed_time > self.STALL_CHECK_DELAY:
                    rpm = abs(self._shooter._motor.get_velocity().value * 60)
                    if rpm < self.MIN_RPM_THRESHOLD:
                        self._test_passed = False
                        self._failure_message = "Shooter motor stalled - possible ball in shooter"
                        self._shooter.stop_motor()
            else:
                rpm = abs(self._shooter._motor.get_velocity().value * 60)
                target_rpm = abs(self._shooter._RPS * 60)
                if rpm < target_rpm * self.RPM_TOLERANCE:
                    self._test_passed = False
                    self._failure_message = f"Shooter motor below target RPM (got {rpm:.0f}, expected ~{target_rpm:.0f})"
                self._shooter.stop_motor()
                self._advance_stage()

        # Stage 2: Test Spindex Motor
        elif self._test_stage == 2:
            SmartDashboard.putString("System Test Stage", "Testing Spindex Motor")
            if elapsed_time < self.MOTOR_TEST_DURATION:
                self._spindex.spin()
                if elapsed_time > self.STALL_CHECK_DELAY:
                    rpm = abs(self._spindex._motor.get_velocity().value * 60)
                    if rpm < self.MIN_RPM_THRESHOLD:
                        self._test_passed = False
                        self._failure_message = "Spindex motor stalled - possible ball in hopper"
                        self._spindex.stop_motor()
            else:
                rpm = abs(self._spindex._motor.get_velocity().value * 60)
                target_rpm = abs(self._spindex._RPS * 60)
                if rpm < target_rpm * self.RPM_TOLERANCE:
                    self._test_passed = False
                    self._failure_message = f"Spindex motor below target RPM (got {rpm:.0f}, expected ~{target_rpm:.0f})"
                self._spindex.stop_motor()
                self._advance_stage()

        # Stage 3: Test Transfer Motor 1
        elif self._test_stage == 3:
            SmartDashboard.putString("System Test Stage", "Testing Transfer Motor 1")
            if elapsed_time < self.MOTOR_TEST_DURATION:
                self._transfer1.spin()
                if elapsed_time > self.STALL_CHECK_DELAY:
                    rpm = abs(self._transfer1._motor.get_velocity().value * 60)
                    if rpm < self.MIN_RPM_THRESHOLD:
                        self._test_passed = False
                        self._failure_message = "Transfer motor 1 stalled"
                        self._transfer1.stop_motor()
            else:
                rpm = abs(self._transfer1._motor.get_velocity().value * 60)
                target_rpm = abs(self._transfer1._RPS * 60)
                if rpm < target_rpm * self.RPM_TOLERANCE:
                    self._test_passed = False
                    self._failure_message = f"Transfer motor 1 below target RPM (got {rpm:.0f}, expected ~{target_rpm:.0f})"
                self._transfer1.stop_motor()
                self._advance_stage()

        # Stage 4: Test Transfer Motor 2
        elif self._test_stage == 4:
            SmartDashboard.putString("System Test Stage", "Testing Transfer Motor 2")
            if elapsed_time < self.MOTOR_TEST_DURATION:
                self._transfer2.spin()
                if elapsed_time > self.STALL_CHECK_DELAY:
                    rpm = abs(self._transfer2._motor.get_velocity().value * 60)
                    if rpm < self.MIN_RPM_THRESHOLD:
                        self._test_passed = False
                        self._failure_message = "Transfer motor 2 stalled"
                        self._transfer2.stop_motor()
            else:
                rpm = abs(self._transfer2._motor.get_velocity().value * 60)
                target_rpm = abs(self._transfer2._RPS * 60)
                if rpm < target_rpm * self.RPM_TOLERANCE:
                    self._test_passed = False
                    self._failure_message = f"Transfer motor 2 below target RPM (got {rpm:.0f}, expected ~{target_rpm:.0f})"
                self._transfer2.stop_motor()
                self._advance_stage()

        # Stage 5: Test Vision System
        elif self._test_stage == 5:
            SmartDashboard.putString("System Test Stage", "Testing Vision System")
            # Give vision a moment to respond
            if elapsed_time > 1.0:
                # Check if vision subsystem is alive by checking SmartDashboard values
                robot_unflatness = SmartDashboard.getNumber("Robot Unflatness", -1)
                if robot_unflatness == -1:
                    self._test_passed = False
                    self._failure_message = "Vision system not responding"
                self._advance_stage()

        # Stage 6: Complete
        elif self._test_stage == 6:
            SmartDashboard.putString("System Test Stage", "Test Complete")

    def _advance_stage(self):
        """Move to next test stage."""
        self._test_stage += 1
        self._stage_start_time = time.time()

    def isFinished(self):
        """Return True when the command should end."""
        # End if test failed
        if not self._test_passed:
            SmartDashboard.putBoolean("System Test Passed", False)
            SmartDashboard.putString("System Test Failure", self._failure_message)
            SmartDashboard.putString("System Test Status", f"FAILED: {self._failure_message}")
            return True

        # End if all stages complete
        if self._test_stage >= 6:
            SmartDashboard.putBoolean("System Test Passed", True)
            SmartDashboard.putString("System Test Status", "PASSED - All systems nominal")
            return True

        return False

    def end(self, interrupted):
        """Called once when the command ends."""
        # Ensure all motors are stopped
        self._intake.stop_motor()
        self._shooter.stop_motor()
        self._spindex.stop_motor()
        self._transfer1.stop_motor()
        self._transfer2.stop_motor()

        if interrupted:
            SmartDashboard.putString("System Test Status", "INTERRUPTED")
            SmartDashboard.putBoolean("System Test Passed", False)
