import commands2

from subsystems.shooter.dual_shooter import DualMotorShooter


class EnableShooter(commands2.Command):
    def __init__(self, shooter_subsystem: DualMotorShooter) -> None:
        self._shooter = shooter_subsystem
        self.addRequirements(self._shooter)

    def initialize(self):
        self._shooter.enable()

    def end(self, interrupted):
        self._shooter.disable()


class DisableShooter(commands2.Command):
    def __init__(self, shooter_subsystem: DualMotorShooter) -> None:
        self._shooter = shooter_subsystem
        self.addRequirements(self._shooter)

    def initialize(self):
        self._shooter.disable()
