import commands2
import phoenix6
from phoenix6.controls import Follower
from phoenix6 import signals

from wpilib import SmartDashboard, DigitalInput
from constants.intake import kIntakeSpinner, kIntakeDeployer
from util.editable_pid import EditablePID

class IntakeSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.left_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.LEFT_CAN_ID, "rio")
        # defines the  left deplpyer motor, "rio" is for saying it is on the native roboRIO CAN bus 
        self.right_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.RIGHT_CAN_ID, "rio")
        # defines the right deployer motor
        self.spinny_motor = phoenix6.hardware.TalonFX(kIntakeSpinner.CAN_ID, "rio")
        #defines the motor that spins the bar/actual picker upper thing
        
        self.deployer_cfg = kIntakeDeployer._CONFIG
        #gets all .CONFIG values of the deployer for intake
        self.spinny_cfg = kIntakeSpinner._CONFIG 
        # gets all .CONFIG values of the spinner for intake
        
        self.spinny_cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        # when the spiny motor's output is neutral it coasts

        self.left_deployer.configurator.apply(self.deployer_cfg)
        # self.right_deployer.configurator.apply(self.deployer_cfg) # might need this, sets the 'config info' for the motor
        self.spinny_motor.configurator.apply(self.spinny_cfg)
        # sets config info for motor
        self.right_deployer.set_control(
            #tells Right motor to follow left usinf the parameters--> device ID of the leader(is defined as the CAN ID in line 14, TalonFX rcognizes the first parameter as the device_id) and the 
            Follower(
                self.left_deployer.device_id,                                 
                motor_alignment = signals.spn_enums.MotorAlignmentValue.OPPOSED
            )
        )
        
        self.deployer_position_voltage = phoenix6.controls.PositionVoltage(position=0, slot=0)
        #
        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)
        
        self.deploy_editable_pid = EditablePID("Intake/Deployer", self.left_deployer, self.deployer_cfg, use_slot1=True)
        self.spinny_editable_pid = EditablePID("Intake/Spinny", self.spinny_motor, self.spinny_cfg)
        SmartDashboard.putNumber("IntakeLeftDeployer position", self.left_deployer.get_position().value)
        SmartDashboard.putNumber("Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION)
        SmartDashboard.putNumber("Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION)
        SmartDashboard.putNumber("Intake/Spinny Speed", kIntakeSpinner.TARGET_RPS)
        
    def set_deployer_positon(self, pos):
        self.left_deployer.set_control(self.deployer_position_voltage.with_position(pos))
        
    def deploy(self):   
        self.spinny_motor.set_control(self.velocity_voltage.with_velocity(kIntakeSpinner.TARGET_RPS))
         
        self.left_deployer.set_control(
            self.deployer_position_voltage
            .with_position(kIntakeDeployer.DEPLOYED_POSITION)
            .with_slot(0)
        )
    
    def undeploy(self):
        self.spinny_motor.disable()
        
        self.left_deployer.set_control(
            self.deployer_position_voltage
            .with_position(kIntakeDeployer.INITIAL_POSITION)
            .with_slot(0)
        )
    
    def periodic(self):
        # This only detects left limit switches for now but it still should work ideally
        
        SmartDashboard.putString("Intake/State", kIntakeDeployer.STATE)
        kIntakeDeployer.DEPLOYED_POSITION = SmartDashboard.getNumber("Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION)
        kIntakeDeployer.INITIAL_POSITION = SmartDashboard.getNumber("Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION)
        kIntakeSpinner.TARGET_RPS = SmartDashboard.getNumber("Intake/Spinny Speed", kIntakeSpinner.TARGET_RPS)
        
        self.deploy_editable_pid.periodic()
        self.spinny_editable_pid.periodic()
        

        