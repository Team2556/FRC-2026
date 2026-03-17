"""Tests that the robot boots without crashing and all subsystems initialize."""

import commands2
import pytest


@pytest.fixture(autouse=True)
def scheduler_cleanup():
    """Reset the command scheduler before and after each test."""
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


def test_robotcontainer_instantiates(container):
    """RobotContainer should instantiate without exceptions."""
    assert container is not None


def test_all_subsystems_exist(container):
    """All expected subsystems should be non-None after init."""
    assert container._drivetrain is not None
    assert container.intake_subsystem is not None
    assert container.transfer_subsystem is not None
    assert container.shooter_subsystem is not None
    assert container.hood_subsystem is not None
    assert container.mono_vision is not None
    assert container.LED_controller is not None


def test_default_commands_set(container):
    """Subsystems with default commands should have them assigned."""
    assert container._drivetrain.getDefaultCommand() is not None
    assert container.mono_vision.getDefaultCommand() is not None
    assert container.shooter_subsystem.getDefaultCommand() is not None


def test_scheduler_runs_without_error(container):
    """The command scheduler should tick several cycles without crashing."""
    scheduler = commands2.CommandScheduler.getInstance()
    for _ in range(10):
        scheduler.run()
