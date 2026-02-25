# import commands2
# import phoenix6
# from phoenix6.controls import Follower
# from phoenix6 import signals

# from wpilib import SmartDashboard, DigitalInput
# from constants.intake import kIntakeSpinner, kIntakeDeployer
# from util.editable_pid import EditablePID

# class IntakeSubsystem(commands2.Subsystem):
#     def __init__(self):
#         super().__init__()

#         self.left_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.LEFT_CAN_ID, "rio")
#         self.right_deployer = phoenix6.hardware.TalonFX(kIntakeDeployer.RIGHT_CAN_ID, "rio")
#         self.spinny_motor = phoenix6.hardware.TalonFX(kIntakeSpinner.CAN_ID, "rio")
        
#         self.deployer_cfg = kIntakeDeployer._CONFIG
#         self.spinny_cfg = kIntakeSpinner._CONFIG 
#         self.spinny_cfg.motor_output.neutral_mode = signals.NeutralModeValue.COAST

#         self.left_deployer.configurator.apply(self.deployer_cfg)
#         # self.right_deployer.configurator.apply(self.deployer_cfg) # might need this
#         self.spinny_motor.configurator.apply(self.spinny_cfg)
        
#         self.right_deployer.set_control(
#             Follower(
#                 self.left_deployer.device_id,                                 
#                 motor_alignment = signals.spn_enums.MotorAlignmentValue.OPPOSED
#             )
#         )
        
#         self.deployer_position_voltage = phoenix6.controls.PositionVoltage(position=0, slot=0)
#         self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)
        
#         self.state : str = "undeployed"
        
#         self.deploy_editable_pid = EditablePID("Intake/Deployer", self.left_deployer, self.deployer_cfg, use_slot1=True)
#         self.spinny_editable_pid = EditablePID("Intake/Spinny", self.spinny_motor, self.spinny_cfg)
        
#         SmartDashboard.putNumber("Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION)
#         SmartDashboard.putNumber("Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION)
#         SmartDashboard.putNumber("Intake/Spinny Speed", kIntakeSpinner.TARGET_RPS)
    
#     def deploy(self):
#         self.spinny_motor.set_control(self.velocity_voltage.with_velocity(kIntakeSpinner.TARGET_RPS))
        
#         self.left_deployer.set_control(
#             self.deployer_position_voltage
#             .with_position(kIntakeDeployer.DEPLOYED_POSITION)
#             .with_slot(0)
#         )
#         self.state = "deploying"
    
#     def undeploy(self):
#         self.spinny_motor.disable()
        
#         self.left_deployer.set_control(
#             self.deployer_position_voltage
#             .with_position(kIntakeDeployer.INITIAL_POSITION)
#             .with_slot(0)
#         )
#         self.state = "undeploying"
    
#     def periodic(self):
#         # This only detects left limit switches for now but it still should work ideally
        
#         forward_limit = self.left_deployer.get_forward_limit()
#         if forward_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and self.state == "deploying":
#             self.left_deployer.set_control(self.deployer_position_voltage.with_slot(1))
#             self.state = "deployed"
        
#         reverse_limit = self.left_deployer.get_reverse_limit()
#         if reverse_limit.value is signals.ForwardLimitValue.CLOSED_TO_GROUND and self.state == "undeploying":
#             self.left_deployer.set_control(self.deployer_position_voltage.with_slot(1))
#             self.state = "undeployed"
            
#         SmartDashboard.putString("Intake/State", self.state)
#         kIntakeDeployer.INITIAL_POSITION = SmartDashboard.getNumber("Intake/Deploy Initial Position", kIntakeDeployer.INITIAL_POSITION)
#         kIntakeDeployer.DEPLOYED_POSITION = SmartDashboard.getNumber("Intake/Deploy Active Position", kIntakeDeployer.DEPLOYED_POSITION)
#         kIntakeSpinner.TARGET_RPS = SmartDashboard.getNumber("Intake/Spinny Speed", kIntakeSpinner.TARGET_RPS)
        
#         self.deploy_editable_pid.periodic()
#         self.spinny_editable_pid.periodic()
        

        