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
        coast_when_neutral=False
    ):
        super().__init__()

        self._motor = phoenix6.hardware.TalonFX(id, "rio")
        self.name = name
        self.cfg = config

        for _ in range(5):
            status = self._motor.configurator.apply(self.cfg)
            if status.is_ok(): break
            
        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self._RPS = target_rpm / 60

        self.enable_smartdashboard = enable_smartdashboard
        
        if self.enable_smartdashboard:
            SmartDashboard.putNumber(f"Controlled Motors/{self.name}/ k_p", self.cfg.slot0.k_p)
            SmartDashboard.putNumber(f"Controlled Motors/{self.name}/ k_i", self.cfg.slot0.k_i)
            SmartDashboard.putNumber(f"Controlled Motors/{self.name}/ k_d", self.cfg.slot0.k_d)
            SmartDashboard.putNumber(f"Controlled Motors/{self.name}/ Target RPM", target_rpm)
            SmartDashboard.putBoolean(f"Controlled Motors/{self.name}/ Working", False)
        
        if coast_when_neutral:
            self._motor.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        else:
            self._motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

    def spin(self):
        self._motor.set_control(self.velocity_voltage.with_velocity(self._RPS))
        # self._motor.set(self._RPS / 100)

        if self.enable_smartdashboard:
            SmartDashboard.putBoolean(f"Controlled Motors/{self.name}/ Working", True)

    def stop_motor(self):
        self._motor.set(0)
        if self.enable_smartdashboard:
            SmartDashboard.putBoolean(f"Controlled Motors/{self.name}/ Working", False)
    
    def get_rpm(self):
        '''Returns the RPM of the mtor'''
        motor_rps = self._motor.get_velocity().value
        return motor_rps * 60

    def periodic(self):
        
        SmartDashboard.putNumber(
            f"Controlled Motors/{self.name}{"/" if self.enable_smartdashboard else ""} RPM", self._motor.get_velocity().value * 60
        )

        if self.enable_smartdashboard:
            self._RPS = SmartDashboard.getNumber(f"Controlled Motors/{self.name}/ Target RPM", 0) / 60

            value_changed = (
                (self.cfg.slot0.k_p != SmartDashboard.getNumber(f"Controlled Motors/{self.name} k_p", 0))
                or (
                    self.cfg.slot0.k_i
                    != SmartDashboard.getNumber(f"Controlled Motors/{self.name}/ k_i", 0)
                )
                or (
                    self.cfg.slot0.k_d
                    != SmartDashboard.getNumber(f"Controlled Motors/{self.name}/ k_d", 0)
                )
            )

            if value_changed:
                self.cfg.slot0.k_p = SmartDashboard.getNumber(f"Controlled Motors/{self.name}/ k_p", 0)
                self.cfg.slot0.k_i = SmartDashboard.getNumber(f"Controlled Motors/{self.name}/ k_i", 0)
                self.cfg.slot0.k_d = SmartDashboard.getNumber(f"Controlled Motors/{self.name}/ k_d", 0)
                self._motor.configurator.apply(self.cfg)
