from wpilib import SmartDashboard, DigitalInput

import commands2

import phoenix6
from phoenix6.controls import Follower
from phoenix6 import signals

from util.editable_pid import EditablePID
from util.nt_util import NTTable

from constants.intake import kIntakeSpinner, kIntakeDeployer
from constants.canbus import kCANId


class IntakeSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.left_deployer = phoenix6.hardware.TalonFXS(kCANId.intake.LEFT_PIVOT, "rio")
        # defines the  left deplpyer motor, "rio" is for saying it is on the native roboRIO CAN bus
        self.right_deployer = phoenix6.hardware.TalonFXS(
            kCANId.intake.RIGHT_PIVOT, "rio"
        )
        # defines the right deployer motor
        self.spinny_motor = phoenix6.hardware.TalonFX(kCANId.intake.SPINNER, "rio")
        # defines the motor that spins the bar/actual picker upper thing

        self.deployer_cfg = kIntakeDeployer._CONFIG
        # gets all .CONFIG values of the deployer for intake
        self.spinny_cfg = kIntakeSpinner._CONFIG
        # gets all .CONFIG values of the spinner for intake

        self.spinny_cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        # when the spiny motor's output is neutral it coasts

        self.left_deployer.configurator.apply(self.deployer_cfg)
        # self.right_deployer.configurator.apply(self.deployer_cfg) # might need this, sets the 'config info' for the motor
        self.spinny_motor.configurator.apply(self.spinny_cfg)
        # sets config info for motor
        self.right_deployer.set_control(
            # tells Right motor to follow left usinf the parameters--> device ID of the leader(is defined as the CAN ID in line 14, TalonFX rcognizes the first parameter as the device_id)
            Follower(
                self.left_deployer.device_id,
                motor_alignment=signals.spn_enums.MotorAlignmentValue.OPPOSED,
            )
        )

        self.deployer_position_voltage = phoenix6.controls.PositionVoltage(
            position=0, slot=0
        )
        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self.state = "undeployed"

        self.nt = NTTable("Intake")
        self.nt.float("Deployer Position", 0.0)
        self.nt.float("Target Deployer Position", kIntakeDeployer.DEPLOYED_POSITION)
        self.nt.float("Ideal Deployer Position", 0.0)
        self.nt.int("Current Slot", 0)
        self.nt.float("Spinny RPM", 0.0)
        self.nt.float("Target Spinny RPM", kIntakeSpinner.TARGET_RPM)
        self.nt.float("Ideal Spinny RPM", 0.0)
        self.nt.string("State", self.state)

        self.deploy_editable_pid = EditablePID(
            "Intake/DeployerPID", self.left_deployer, self.deployer_cfg, use_slot1=True
        )
        self.spinny_editable_pid = EditablePID(
            "Intake/SpinnyPID", self.spinny_motor, self.spinny_cfg
        )

    def set_deployer_position(self, pos):
        self.left_deployer.set_control(self.deployer_position_voltage.with_position(pos))
        self.nt.set("Ideal Deployer Position", pos)
    
    def set_internal_deployer_position(self, pos):
        self.left_deployer.set_position(pos)
        self.nt.set("Ideal Deployer Position", 0)
    
    def change_deployer_slot(self, slot=0):
        self.left_deployer.set_control(self.deployer_position_voltage.with_slot(slot))
        self.nt.set("Current Slot", slot)
    
    def set_spinny_speed(self, rpm):
        self.spinny_motor.set_control(self.velocity_voltage.with_velocity(rpm / 60))
        self.nt.set("Ideal Spinny RPM", rpm)
    
    def periodic(self):
        if self.left_deployer.get_reverse_limit().value is signals.ReverseLimitValue.CLOSED_TO_GROUND:
            self.set_internal_deployer_position(0)
        
        self.nt.set("Deployer Position", self.left_deployer.get_position().value)
        self.nt.set("Spinny RPM", self.spinny_motor.get_velocity().value * 60)
        self.nt.set("State", self.state)
        kIntakeDeployer.DEPLOYED_POSITION = self.nt.get("Target Deployer Position")
        kIntakeSpinner.TARGET_RPM = self.nt.get("Target Spinny RPM")

        # Commented out to prevent periodic config application warnings
        # self.deploy_editable_pid.periodic()
        # self.spinny_editable_pid.periodic()
