import commands2

from subsystems.trasnfer.transfer_subsystem import TransferSubsystem
from subsystems.shooter.dual_shooter import DualMotorShooter


class RunTransferCommand(commands2.Command):
    def __init__(self, transfer_sybsystem: TransferSubsystem):
        super().__init__()
        self.transfer_sybsystem = transfer_sybsystem
        self.addRequirements(transfer_sybsystem)

    def execute(self):
        self.transfer_sybsystem.activate()

    def end(self, interrupted):
        self.transfer_sybsystem.stop()
        
class ReverseTransferCommand(commands2.Command):
    def __init__(self, transfer_sybsystem: TransferSubsystem):
        super().__init__()
        self.transfer_sybsystem = transfer_sybsystem
        self.addRequirements(transfer_sybsystem)

    def execute(self):
        self.transfer_sybsystem.reverse()

    def end(self, interrupted):
        self.transfer_sybsystem.stop()

class SpindexOnlyCommand(commands2.Command):
    def __init__(self, transfer_sybsystem: TransferSubsystem):
        super().__init__()
        self.transfer_sybsystem = transfer_sybsystem
        self.addRequirements(transfer_sybsystem)

    def execute(self):
        self.transfer_sybsystem.reverse()

    def end(self, interrupted):
        self.transfer_sybsystem.stop()