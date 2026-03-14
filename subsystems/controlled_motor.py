import commands2
import phoenix6
from util.nt_util import NTTable


class ControlledTalonMotor(commands2.Subsystem):
    def __init__(
        self,
        name: str,
        id: int,
        config: phoenix6.configs.TalonFXConfiguration,
        target_rpm: float,
        enable_smartdashboard=False,
        coast_when_neutral=False,
    ):
        super().__init__()
        self._motor = phoenix6.hardware.TalonFX(id, "rio")
        self.name = name
        self.cfg = config
        self.enable_smartdashboard = enable_smartdashboard

        for _ in range(5):
            status = self._motor.configurator.apply(self.cfg)
            if status.is_ok():
                break

        self.velocity_voltage = phoenix6.controls.VelocityVoltage(velocity=0, slot=0)

        self.nt = NTTable(self.name).get_subtable("Controlled Motor")
        self.nt.float("k_p", self.cfg.slot0.k_p)
        self.nt.float("k_i", self.cfg.slot0.k_i)
        self.nt.float("k_d", self.cfg.slot0.k_d)
        self.nt.float("RPM", target_rpm)
        self.nt.float("Target RPM", target_rpm)
        self.nt.bool("Working", False)

        self._RPS = target_rpm / 60

        if coast_when_neutral:
            self._motor.setNeutralMode(phoenix6.signals.NeutralModeValue.COAST)
        else:
            self._motor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)

    def spin(self):
        self._motor.set_control(self.velocity_voltage.with_velocity(self._RPS))
        self.nt.set("Working", True)

    def stop_motor(self):
        self._motor.set(0)
        self.nt.set("Working", False)

    def get_rpm(self):
        return self._motor.get_velocity().value * 60

    def periodic(self):
        self.nt.set("RPM", self._motor.get_velocity().value * 60)
        self._RPS = self.nt.get("Target RPM") / 60

        value_changed = (
            self.cfg.slot0.k_p != self.nt.get("k_p")
            or self.cfg.slot0.k_i != self.nt.get("k_i")
            or self.cfg.slot0.k_d != self.nt.get("k_d")
        )

        if value_changed:
            self.cfg.slot0.k_p = self.nt.get("k_p")
            self.cfg.slot0.k_i = self.nt.get("k_i")
            self.cfg.slot0.k_d = self.nt.get("k_d")
            self._motor.configurator.apply(self.cfg)
