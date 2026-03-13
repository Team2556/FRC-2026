from phoenix6.configs import TalonFXConfiguration, SlotConfigs
from phoenix6.hardware import TalonFX
from wpilib import SmartDashboard

class EditablePID():
    def __init__(self, name, talon_motor : TalonFX, cfg : TalonFXConfiguration, use_slot0 = True, use_slot1 = False, use_slot2 = False):
        '''
        Takes a TalonFXConfiguration and puts the PID slots stuff to SmartDashboard.
        (because this capability would be used a lot so it's a "component" now)
        Also you have to call the periodic() function on this class.
        '''
        
        self.name = name
        self.talon_motor = talon_motor
        self.cfg = cfg
        
        self.use_slot0 = use_slot0
        self.use_slot1 = use_slot1
        self.use_slot2 = use_slot2
        
        if self.use_slot0:
            self.cfg.slot0.k_p != SmartDashboard.putNumber(f"{self.name}/slot 0 k_p", 0)
            self.cfg.slot0.k_i != SmartDashboard.putNumber(f"{self.name}/slot 0 k_i", 0)
            self.cfg.slot0.k_d != SmartDashboard.putNumber(f"{self.name}/slot 0 k_d", 0)
        if self.use_slot1:
            self.cfg.slot1.k_p != SmartDashboard.putNumber(f"{self.name}/slot 1 k_p", 0)
            self.cfg.slot1.k_i != SmartDashboard.putNumber(f"{self.name}/slot 1 k_i", 0)
            self.cfg.slot1.k_d != SmartDashboard.putNumber(f"{self.name}/slot 1 k_d", 0)
        if self.use_slot2:
            self.cfg.slot2.k_p != SmartDashboard.putNumber(f"{self.name}/slot 2 k_p", 0)
            self.cfg.slot2.k_i != SmartDashboard.putNumber(f"{self.name}/slot 2 k_i", 0)
            self.cfg.slot2.k_d != SmartDashboard.putNumber(f"{self.name}/slot 2 k_d", 0)

    def periodic(self):
        value_changed = (
               (self.cfg.slot0.k_p != SmartDashboard.getNumber(f"{self.name}/slot 0 k_p", 0))
            or (self.cfg.slot0.k_i != SmartDashboard.getNumber(f"{self.name}/slot 0 k_i", 0))
            or (self.cfg.slot0.k_d != SmartDashboard.getNumber(f"{self.name}/slot 0 k_d", 0))
            or (self.cfg.slot1.k_p != SmartDashboard.getNumber(f"{self.name}/slot 1 k_p", 0))
            or (self.cfg.slot1.k_i != SmartDashboard.getNumber(f"{self.name}/slot 1 k_i", 0))
            or (self.cfg.slot1.k_d != SmartDashboard.getNumber(f"{self.name}/slot 1 k_d", 0))
            or (self.cfg.slot2.k_p != SmartDashboard.getNumber(f"{self.name}/slot 2 k_p", 0))
            or (self.cfg.slot2.k_i != SmartDashboard.getNumber(f"{self.name}/slot 2 k_i", 0))
            or (self.cfg.slot2.k_d != SmartDashboard.getNumber(f"{self.name}/slot 2 k_d", 0))
        )
        
        if value_changed:
            if self.use_slot0:
                self.cfg.slot0.k_p = SmartDashboard.getNumber(f"{self.name}/slot 0 k_p", 0)
                self.cfg.slot0.k_i = SmartDashboard.getNumber(f"{self.name}/slot 0 k_i", 0)
                self.cfg.slot0.k_d = SmartDashboard.getNumber(f"{self.name}/slot 0 k_d", 0)
            if self.use_slot1:
                self.cfg.slot1.k_p = SmartDashboard.getNumber(f"{self.name}/slot 1 k_p", 0)
                self.cfg.slot1.k_i = SmartDashboard.getNumber(f"{self.name}/slot 1 k_i", 0)
                self.cfg.slot1.k_d = SmartDashboard.getNumber(f"{self.name}/slot 1 k_d", 0)
            if self.use_slot2:
                self.cfg.slot2.k_p = SmartDashboard.getNumber(f"{self.name}/slot 2 k_p", 0)
                self.cfg.slot2.k_i = SmartDashboard.getNumber(f"{self.name}/slot 2 k_i", 0)
                self.cfg.slot2.k_d = SmartDashboard.getNumber(f"{self.name}/slot 2 k_d", 0)
            self.talon_motor.configurator.apply(self.cfg)
    
    def with_slot1(self):
        self.use_slot1 = True
        return self
    
    def with_slot2(self):
        self.use_slot2s = True
        return self