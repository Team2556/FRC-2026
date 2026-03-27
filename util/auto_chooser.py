from commands2 import Command

from wpilib import SendableChooser

from util.nt_util import NTTable


class AutoChooser:
    def __init__(self, autos: dict[str, Command]):
        self.autos = autos
        self._nt = NTTable("Autonomous")

    def make_dropdown(self):
        self.chooser = SendableChooser()

        first = True
        for name, command in self.autos.items():
            if first:
                self.chooser.setDefaultOption(name, command)
                first = False
            else:
                self.chooser.addOption(name, command)

        self._nt.sendable("Selector", self.chooser)

    def update(self) -> None:
        """Call every loop to sync the dashboard selection with the robot.

        SendableChooser writes its current selection back to the 'active' NT
        entry only when the builder is updated.  Without this, dashboard
        changes (e.g. from Elastic) are never echoed back and getSelected()
        never sees the new value.
        """
        self._nt.update_sendables()

    def choose_auto(self):
        return self.chooser.getSelected()
