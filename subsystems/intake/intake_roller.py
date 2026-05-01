import wpilib
import commands2
import phoenix6
import phoenix6.controls
from phoenix6 import signals
from phoenix6.controls import VelocityVoltage, NeutralOut

from util.editable_pid import EditablePID
from util.nt_util import NTTable

from constants.intake import kIntakeRoller
from constants.canbus import kCANId


class IntakeRoller(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.roller_motor = phoenix6.hardware.TalonFX(kCANId.intake.ROLLER, "rio")

        self.roller_cfg = kIntakeRoller._CONFIG
        self.roller_cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE

        if not wpilib.RobotBase.isSimulation():
            self.roller_motor.configurator.apply(self.roller_cfg)

        self._vel_req = VelocityVoltage(velocity=0, slot=0)
        self._coast = NeutralOut()

        self._enabled = False
        self._target_rpm = 0.0

        self.nt = NTTable("Intake")
        self.nt.float("Roller RPM", 0.0)
        self.nt.float("Target Roller RPM", 0.0)
        self.nt.string("Roller Command", "_none")

        self.roller_editable_pid = EditablePID(
            "Intake/RollerPID", self.roller_motor, self.roller_cfg
        )

    def set_target_rpm(self, rpm: float) -> None:
        self._target_rpm = rpm
        self._enabled = True

    def stop_roller(self) -> None:
        self._enabled = False

    def periodic(self) -> None:
        if not self._enabled:
            self.roller_motor.set_control(self._coast)
        else:
            self.roller_motor.set_control(
                self._vel_req.with_velocity(self._target_rpm / 60)
            )
        
        # try:
        #     self.nt.set("Roller Command", self.getCurrentCommand().getName())
        # except:
        #     pass

        # self.nt.set("Roller RPM", self.roller_motor.get_velocity().value * 60)
        # self.nt.set("Target Roller RPM", self._target_rpm if self._enabled else 0.0)
