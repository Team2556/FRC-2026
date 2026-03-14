import commands2

from subsystems.transfer_subsystem import TransferSubsystem

class RunTransferCommand(commands2.Command):
    def __init__(self, transfer_sybsystem: TransferSubsystem):
        super().__init__()
        self.transfer_sybsystem = transfer_sybsystem
        self.addRequirements(transfer_sybsystem)
        
    def initialize(self):
        self.transfer_sybsystem.activate()

    def end(self, interrupted):
        self.transfer_sybsystem.stop()