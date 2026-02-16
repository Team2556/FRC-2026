import commands2
import phoenix6

from wpilib import SmartDashboard


class ControlledTalonMotor(commands2.Subsystem):
    def __init__(
        self,
        name: str,
        id: int,
        config: phoenix6.configs.TalonFXConfiguration,
        target_rpm: float,
        enable_smartdashboard=False,
    ):
        super().__init__()

        self._motor = phoenix6.hardware.TalonFX(id, "rio")
        self.name = name
        self.cfg = config

        self._motor.configurator.apply(self.cfg)
        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self._RPS = target_rpm / 60

        self.enable_smartdashboard = enable_smartdashboard
        if self.enable_smartdashboard:
            SmartDashboard.putNumber(f"{self.name} Target RPM", target_rpm)
            SmartDashboard.putBoolean(f"{self.name} Working", False)

    def spin(self):
        self._motor.set_control(self.velocity_voltage.with_velocity(self._RPS))

        if self.enable_smartdashboard:
            SmartDashboard.putBoolean(f"{self.name} Working", True)

    def stop_motor(self):
        self._motor.set(0)
        if self.enable_smartdashboard:
            SmartDashboard.putBoolean(f"{self.name} Working", False)

    def periodic(self):
        SmartDashboard.putNumber(
            f"{self.name} RPM", self._motor.get_velocity().value * 60
        )

        if self.enable_smartdashboard:
            self._RPS = SmartDashboard.getNumber(f"{self.name} Target RPM", 0) / 60

            value_changed = (
                (self.cfg.slot0.k_p != SmartDashboard.getNumber(f"{self.name} k_p", 0))
                or (
                    self.cfg.slot0.k_i
                    != SmartDashboard.getNumber(f"{self.name} k_i", 0)
                )
                or (
                    self.cfg.slot0.k_d
                    != SmartDashboard.getNumber(f"{self.name} k_d", 0)
                )
            )

            if value_changed:
                self.cfg.slot0.k_p = SmartDashboard.getNumber(f"{self.name} k_p", 0)
                self.cfg.slot0.k_i = SmartDashboard.getNumber(f"{self.name} k_i", 0)
                self.cfg.slot0.k_d = SmartDashboard.getNumber(f"{self.name} k_d", 0)
                self._motor.configurator.apply(self.cfg)
