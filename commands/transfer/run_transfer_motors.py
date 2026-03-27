import commands2

from subsystems.trasnfer.transfer_subsystem import TransferSubsystem


class RunTransferCommand(commands2.Command):
    def __init__(self, transfer_sybsystem: TransferSubsystem):
        super().__init__()
        self.transfer_sybsystem = transfer_sybsystem
        self.addRequirements(transfer_sybsystem)

    def initialize(self):
        pass

    def execute(self):
        self.transfer_sybsystem.activate()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.transfer_sybsystem.stop()


class ReverseTransferCommand(commands2.Command):
    def __init__(self, transfer_sybsystem: TransferSubsystem):
        super().__init__()
        self.transfer_sybsystem = transfer_sybsystem
        self.addRequirements(transfer_sybsystem)

    def initialize(self):
        pass

    def execute(self):
        self.transfer_sybsystem.reverse()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.transfer_sybsystem.stop()


class SpindexOnlyCommand(commands2.Command):
    def __init__(self, transfer_sybsystem: TransferSubsystem):
        super().__init__()
        self.transfer_sybsystem = transfer_sybsystem
        self.addRequirements(transfer_sybsystem)

    def initialize(self):
        pass

    def execute(self):
        self.transfer_sybsystem.spindex_only()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self.transfer_sybsystem.stop()
