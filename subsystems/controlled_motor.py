import commands2
import phoenix6

from wpilib import SmartDashboard


class ControlledMotor(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self._motor = phoenix6.hardware.TalonFX(22, "rio")

        self.cfg = phoenix6.configs.TalonFXConfiguration()
        SmartDashboard.putNumber("Motor k_p", 1)
        SmartDashboard.putNumber("Motor k_i", 0)
        SmartDashboard.putNumber("Motor k_d", 0)

        self.cfg.slot0.k_p = 0
        self.cfg.slot0.k_i = 5
        self.cfg.slot0.k_d = 0

        self._motor.configurator.apply(self.cfg)
        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self._RPS = 60
        SmartDashboard.putNumber("Motor Target RPS", 60)

    def spin(self):
        self._motor.set_control(self.velocity_voltage.with_velocity(self._RPS))

    def stop_motor(self):
        self._motor.set(0)

    def periodic(self):
        SmartDashboard.putNumber("Motor RPS", self._motor.get_velocity().value)
        self._RPS = SmartDashboard.getNumber("Motor Target RPS", 60)

        value_changed = (
            (self.cfg.slot0.k_p != SmartDashboard.getNumber("Motor k_p", 1))
            or (self.cfg.slot0.k_i != SmartDashboard.getNumber("Motor k_i", 0))
            or (self.cfg.slot0.k_d != SmartDashboard.getNumber("Motor k_d", 0))
        )

        if value_changed:
            self.cfg.slot0.k_p = SmartDashboard.getNumber("Motor k_p", 1)
            self.cfg.slot0.k_i = SmartDashboard.getNumber("Motor k_i", 0)
            self.cfg.slot0.k_d = SmartDashboard.getNumber("Motor k_d", 0)
            self._motor.configurator.apply(self.cfg)
