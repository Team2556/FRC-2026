from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from util.nt_util import NTTable


class EditablePID:
    def __init__(self, name: str, talon_motor: TalonFX, cfg: TalonFXConfiguration, use_slot0: bool = True, use_slot1: bool = False, use_slot2: bool = False):
        self.name = name
        self.talon_motor = talon_motor
        self.cfg = cfg

        self.use_slot0 = use_slot0
        self.use_slot1 = use_slot1
        self.use_slot2 = use_slot2

        self.nt = NTTable(self.name).get_subtable("EditablePID")
        
        # Track last applied values to avoid unnecessary config updates
        self.last_applied_values = {}

        if self.use_slot0:
            self.nt.float("slot0/k_p", self.cfg.slot0.k_p)
            self.nt.float("slot0/k_i", self.cfg.slot0.k_i)
            self.nt.float("slot0/k_d", self.cfg.slot0.k_d)
            self.last_applied_values['slot0'] = {
                'k_p': self.cfg.slot0.k_p,
                'k_i': self.cfg.slot0.k_i, 
                'k_d': self.cfg.slot0.k_d
            }
        if self.use_slot1:
            self.nt.float("slot1/k_p", self.cfg.slot1.k_p)
            self.nt.float("slot1/k_i", self.cfg.slot1.k_i)
            self.nt.float("slot1/k_d", self.cfg.slot1.k_d)
            self.last_applied_values['slot1'] = {
                'k_p': self.cfg.slot1.k_p,
                'k_i': self.cfg.slot1.k_i,
                'k_d': self.cfg.slot1.k_d
            }
        if self.use_slot2:
            self.nt.float("slot2/k_p", self.cfg.slot2.k_p)
            self.nt.float("slot2/k_i", self.cfg.slot2.k_i)
            self.nt.float("slot2/k_d", self.cfg.slot2.k_d)
            self.last_applied_values['slot2'] = {
                'k_p': self.cfg.slot2.k_p,
                'k_i': self.cfg.slot2.k_i,
                'k_d': self.cfg.slot2.k_d
            }

    def periodic(self):
        """Only apply configuration when PID values have actually changed from NetworkTables"""
        values_changed = False
        
        # Check slot0 changes
        if self.use_slot0:
            new_kp = self.nt.get("slot0/k_p")
            new_ki = self.nt.get("slot0/k_i") 
            new_kd = self.nt.get("slot0/k_d")
            
            if (new_kp != self.last_applied_values['slot0']['k_p'] or
                new_ki != self.last_applied_values['slot0']['k_i'] or 
                new_kd != self.last_applied_values['slot0']['k_d']):
                
                self.cfg.slot0.k_p = new_kp
                self.cfg.slot0.k_i = new_ki
                self.cfg.slot0.k_d = new_kd
                
                self.last_applied_values['slot0']['k_p'] = new_kp
                self.last_applied_values['slot0']['k_i'] = new_ki  
                self.last_applied_values['slot0']['k_d'] = new_kd
                values_changed = True
        
        # Check slot1 changes
        if self.use_slot1:
            new_kp = self.nt.get("slot1/k_p")
            new_ki = self.nt.get("slot1/k_i")
            new_kd = self.nt.get("slot1/k_d")
            
            if (new_kp != self.last_applied_values['slot1']['k_p'] or
                new_ki != self.last_applied_values['slot1']['k_i'] or
                new_kd != self.last_applied_values['slot1']['k_d']):
                
                self.cfg.slot1.k_p = new_kp
                self.cfg.slot1.k_i = new_ki
                self.cfg.slot1.k_d = new_kd
                
                self.last_applied_values['slot1']['k_p'] = new_kp
                self.last_applied_values['slot1']['k_i'] = new_ki
                self.last_applied_values['slot1']['k_d'] = new_kd
                values_changed = True
                
        # Check slot2 changes  
        if self.use_slot2:
            new_kp = self.nt.get("slot2/k_p")
            new_ki = self.nt.get("slot2/k_i")
            new_kd = self.nt.get("slot2/k_d")
            
            if (new_kp != self.last_applied_values['slot2']['k_p'] or
                new_ki != self.last_applied_values['slot2']['k_i'] or
                new_kd != self.last_applied_values['slot2']['k_d']):
                
                self.cfg.slot2.k_p = new_kp
                self.cfg.slot2.k_i = new_ki
                self.cfg.slot2.k_d = new_kd
                
                self.last_applied_values['slot2']['k_p'] = new_kp
                self.last_applied_values['slot2']['k_i'] = new_ki  
                self.last_applied_values['slot2']['k_d'] = new_kd
                values_changed = True

        # Only apply configuration if values actually changed
        if values_changed:
            self.talon_motor.configurator.apply(self.cfg)
            print(f"[{self.name}] Applied PID configuration update")

    def with_slot1(self) -> "EditablePID":
        self.use_slot1 = True
        return self

    def with_slot2(self) -> "EditablePID":
        self.use_slot2 = True
        return self