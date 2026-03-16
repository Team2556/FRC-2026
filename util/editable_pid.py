import wpilib
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
        # Debounce: track last apply time to stay below Phoenix 6's 1-second frequent-call threshold
        self._last_apply_time = -10.0

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
        tolerance = 0.01  # Large dead-band to rule out floating-point noise
        
        # Check slot0 changes
        if self.use_slot0:
            new_kp = self.nt.get("slot0/k_p")
            new_ki = self.nt.get("slot0/k_i") 
            new_kd = self.nt.get("slot0/k_d")
            
            if (abs(new_kp - self.last_applied_values['slot0']['k_p']) > tolerance or
                abs(new_ki - self.last_applied_values['slot0']['k_i']) > tolerance or 
                abs(new_kd - self.last_applied_values['slot0']['k_d']) > tolerance):
                
                self.cfg.slot0.k_p = new_kp
                self.cfg.slot0.k_i = new_ki
                self.cfg.slot0.k_d = new_kd
                
                self.last_applied_values['slot0']['k_p'] = new_kp
                self.last_applied_values['slot0']['k_i'] = new_ki  
                self.last_applied_values['slot0']['k_d'] = new_kd
                values_changed = True
                print(f"[{self.name}] Slot0 PID changed: kP={new_kp}, kI={new_ki}, kD={new_kd}")
        
        # Check slot1 changes
        if self.use_slot1:
            new_kp = self.nt.get("slot1/k_p")
            new_ki = self.nt.get("slot1/k_i")
            new_kd = self.nt.get("slot1/k_d")
            
            if (abs(new_kp - self.last_applied_values['slot1']['k_p']) > tolerance or
                abs(new_ki - self.last_applied_values['slot1']['k_i']) > tolerance or
                abs(new_kd - self.last_applied_values['slot1']['k_d']) > tolerance):
                
                self.cfg.slot1.k_p = new_kp
                self.cfg.slot1.k_i = new_ki
                self.cfg.slot1.k_d = new_kd
                
                self.last_applied_values['slot1']['k_p'] = new_kp
                self.last_applied_values['slot1']['k_i'] = new_ki
                self.last_applied_values['slot1']['k_d'] = new_kd
                values_changed = True
                print(f"[{self.name}] Slot1 PID changed: kP={new_kp}, kI={new_ki}, kD={new_kd}")
                
        # Check slot2 changes  
        if self.use_slot2:
            new_kp = self.nt.get("slot2/k_p")
            new_ki = self.nt.get("slot2/k_i")
            new_kd = self.nt.get("slot2/k_d")
            
            if (abs(new_kp - self.last_applied_values['slot2']['k_p']) > tolerance or
                abs(new_ki - self.last_applied_values['slot2']['k_i']) > tolerance or
                abs(new_kd - self.last_applied_values['slot2']['k_d']) > tolerance):
                
                self.cfg.slot2.k_p = new_kp
                self.cfg.slot2.k_i = new_ki
                self.cfg.slot2.k_d = new_kd
                
                self.last_applied_values['slot2']['k_p'] = new_kp
                self.last_applied_values['slot2']['k_i'] = new_ki  
                self.last_applied_values['slot2']['k_d'] = new_kd
                values_changed = True
                print(f"[{self.name}] Slot2 PID changed: kP={new_kp}, kI={new_ki}, kD={new_kd}")

        # 1.5s debounce avoids Phoenix 6 "frequent config" warning; FMS gate prevents field mishaps _FCC_
        if values_changed:
            if wpilib.DriverStation.isFMSAttached():
                print(f"[{self.name}] FMS attached - PID updates disabled")
            else:
                now = wpilib.Timer.getFPGATimestamp()
                if now - self._last_apply_time >= 1.5:
                    self.talon_motor.configurator.apply(self.cfg, 0.050)
                    self._last_apply_time = now
                    print(f"[{self.name}] *** APPLIED PID CONFIGURATION TO MOTOR ***")

    def force_apply(self):
        """Bypass debounce and force-push current dashboard values to motor. Dev use only. _FCC_"""
        if wpilib.RobotBase.isSimulation() or wpilib.DriverStation.isFMSAttached():
            return
        if self.use_slot0:
            self.cfg.slot0.k_p = self.nt.get("slot0/k_p")
            self.cfg.slot0.k_i = self.nt.get("slot0/k_i")
            self.cfg.slot0.k_d = self.nt.get("slot0/k_d")
            self.last_applied_values['slot0'] = {'k_p': self.cfg.slot0.k_p, 'k_i': self.cfg.slot0.k_i, 'k_d': self.cfg.slot0.k_d}
        if self.use_slot1:
            self.cfg.slot1.k_p = self.nt.get("slot1/k_p")
            self.cfg.slot1.k_i = self.nt.get("slot1/k_i")
            self.cfg.slot1.k_d = self.nt.get("slot1/k_d")
            self.last_applied_values['slot1'] = {'k_p': self.cfg.slot1.k_p, 'k_i': self.cfg.slot1.k_i, 'k_d': self.cfg.slot1.k_d}
        if self.use_slot2:
            self.cfg.slot2.k_p = self.nt.get("slot2/k_p")
            self.cfg.slot2.k_i = self.nt.get("slot2/k_i")
            self.cfg.slot2.k_d = self.nt.get("slot2/k_d")
            self.last_applied_values['slot2'] = {'k_p': self.cfg.slot2.k_p, 'k_i': self.cfg.slot2.k_i, 'k_d': self.cfg.slot2.k_d}
        self.talon_motor.configurator.apply(self.cfg, 0.050)
        self._last_apply_time = wpilib.Timer.getFPGATimestamp()
        print(f"[{self.name}] FORCE APPLIED PID")

    def with_slot1(self) -> "EditablePID":
        self.use_slot1 = True
        return self

    def with_slot2(self) -> "EditablePID":
        self.use_slot2 = True
        return self