"""Tests for the autonomous command."""

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


def test_get_autonomous_command(container):
    """getAutonomousCommand() should return without crashing.

    NOTE: Currently returns None on main — this test will need updating
    once a real auto is wired up. For now we just verify it doesn't throw.
    """
    auto_cmd = container.getAutonomousCommand()
    # When a real auto exists, uncomment the lines below:
    # assert auto_cmd is not None
    # auto_cmd.schedule()
    # scheduler = commands2.CommandScheduler.getInstance()
    # for _ in range(10):
    #     scheduler.run()
    # auto_cmd.cancel()
    # scheduler.run()


def test_auto_chooser_exists(container):
    """The auto chooser should be initialized."""
    assert container.auto_chooser is not None
