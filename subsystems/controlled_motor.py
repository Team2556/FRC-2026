import commands2
import phoenix6

from wpilib import SmartDashboard

class ControlledTalonMotor(commands2.Subsystem):
    def __init__(self, name, id, p, i, d, rps):
        super().__init__()
        self._motor = phoenix6.hardware.TalonFX(id, "rio")
        
        self.name = name

        self.cfg = phoenix6.configs.TalonFXConfiguration()
        SmartDashboard.putNumber(f"{self.name} k_p", p)
        SmartDashboard.putNumber(f"{self.name} k_i", i)
        SmartDashboard.putNumber(f"{self.name} k_d", d)

        self.cfg.slot0.k_p = p
        self.cfg.slot0.k_i = i
        self.cfg.slot0.k_d = d

        self._motor.configurator.apply(self.cfg)
        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self._RPS = rps
        SmartDashboard.putNumber(f"{self.name} Target RPS", rps)
        
        SmartDashboard.putBoolean(f"{self.name} Working", False)

    def spin(self):
        self._motor.set_control(self.velocity_voltage.with_velocity(self._RPS))
        SmartDashboard.putBoolean(f"{self.name} Working", True)

    def stop_motor(self):
        self._motor.set(0)
        SmartDashboard.putBoolean(f"{self.name} Working", False)

    def periodic(self):
        SmartDashboard.putNumber(f"{self.name} RPS", self._motor.get_velocity().value)
        self._RPS = SmartDashboard.getNumber(f"{self.name} Target RPS", 0)

        value_changed = (
            (self.cfg.slot0.k_p != SmartDashboard.getNumber(f"{self.name} k_p", 0))
            or (self.cfg.slot0.k_i != SmartDashboard.getNumber(f"{self.name} k_i", 0))
            or (self.cfg.slot0.k_d != SmartDashboard.getNumber(f"{self.name} k_d", 0))
        )

        if value_changed:
            self.cfg.slot0.k_p = SmartDashboard.getNumber(f"{self.name} k_p", 0)
            self.cfg.slot0.k_i = SmartDashboard.getNumber(f"{self.name} k_i", 0)
            self.cfg.slot0.k_d = SmartDashboard.getNumber(f"{self.name} k_d", 0)
            self._motor.configurator.apply(self.cfg)