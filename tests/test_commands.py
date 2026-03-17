"""Tests that major commands can be scheduled, ticked, and cancelled cleanly."""

import commands2
import pytest


@pytest.fixture(autouse=True)
def scheduler_cleanup():
    scheduler = commands2.CommandScheduler.getInstance()
    scheduler.cancelAll()
    scheduler.clearComposedCommands()
    yield
    scheduler.cancelAll()
    scheduler.clearComposedCommands()


@pytest.fixture
def container():
    from robotcontainer import RobotContainer

    return RobotContainer()


def _schedule_tick_cancel(cmd, ticks=5):
    """Helper: schedule a command, run N ticks, then cancel."""
    scheduler = commands2.CommandScheduler.getInstance()
    cmd.schedule()
    for _ in range(ticks):
        scheduler.run()
    cmd.cancel()
    scheduler.run()


def test_intake_deploy(container):
    from commands.intake.intake_commands import IntakeCommandDeploy

    cmd = IntakeCommandDeploy(container.intake_subsystem)
    _schedule_tick_cancel(cmd)


def test_intake_undeploy(container):
    from commands.intake.intake_commands import IntakeCommandUndeploy

    cmd = IntakeCommandUndeploy(container.intake_subsystem)
    _schedule_tick_cancel(cmd)


def test_intake_force_retract(container):
    from commands.intake.intake_commands import IntakeForceRetract

    cmd = IntakeForceRetract(container.intake_subsystem)
    _schedule_tick_cancel(cmd)


def test_intake_manual_forward(container):
    from commands.intake.intake_commands import IntakeCommandManualForward

    cmd = IntakeCommandManualForward(container.intake_subsystem)
    _schedule_tick_cancel(cmd)


def test_intake_manual_reverse(container):
    from commands.intake.intake_commands import IntakeCommandManualReverse

    cmd = IntakeCommandManualReverse(container.intake_subsystem)
    _schedule_tick_cancel(cmd)


def test_enable_shooter(container):
    from commands.shooter.shooter_commands import EnableShooter

    cmd = EnableShooter(container.shooter_subsystem)
    _schedule_tick_cancel(cmd)


def test_disable_shooter(container):
    from commands.shooter.shooter_commands import DisableShooter

    cmd = DisableShooter(container.shooter_subsystem)
    _schedule_tick_cancel(cmd)


def test_run_transfer(container):
    from commands.transfer.run_transfer_motors import RunTransferCommand

    cmd = RunTransferCommand(container.transfer_subsystem, container.shooter_subsystem)
    _schedule_tick_cancel(cmd)


def test_controller_drive(container):
    from commands.drive.drive_commands import ControllerDrive

    cmd = ControllerDrive(container._drivetrain, container._controller_1)
    _schedule_tick_cancel(cmd)


def test_reset_shooter_hood(container):
    from commands.shooter.hood_commands import ResetShooterHood

    cmd = ResetShooterHood(container.hood_subsystem)
    _schedule_tick_cancel(cmd)
