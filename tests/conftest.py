import pathlib
import sys

project_root = pathlib.Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# wpilib.getDeployDirectory() uses __main__.__file__ to locate the deploy folder.
# When running via `python -m pytest`, __main__ is the pytest package, so the
# path is wrong. Patch it to the correct project deploy directory before
# importing the robot (which triggers class-level AprilTag layout loading).
import wpilib
_real_get_deploy = wpilib.getDeployDirectory
wpilib.getDeployDirectory = lambda: str(project_root / "deploy")

from robot import MyRobot

wpilib.getDeployDirectory = _real_get_deploy

from pyfrc.test_support.pytest_plugin import PyFrcPlugin


def pytest_configure(config):
    robot_file = project_root / "robot.py"
    config.pluginmanager.register(PyFrcPlugin(MyRobot, robot_file, False))
