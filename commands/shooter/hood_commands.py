from wpilib import SmartDashboard

from commands2 import Command

from util.nt_util import NTTable
from util.robot_zone_checker import RobotZoneChecker

from subsystems.shooter.shooter_hood import ShooterHood
from subsystems.drivetrain.drivetrain import SwerveDriveTrain

from constants.shooter import kHoodMotor
from constants.shooter import kShooterData


class ResetShooterHood(Command):
    def __init__(self, shooter_hood: ShooterHood):
        self.shooter_hood = shooter_hood

        self.nt = NTTable("Shooter").get_subtable("Hood")
        self.nt.bool("Resetting")

        self.addRequirements(self.shooter_hood)

    def initialize(self):
        self.shooter_hood.set_speed(kHoodMotor.RESET_HOME_SPEED)

        self.nt.set("Resetting", True)

    def isFinished(self):
        return self.shooter_hood.is_hard_stopped()

    def end(self, interrupted):
        self.shooter_hood.set_speed(0)

        self.nt.set("Resetting", False)
