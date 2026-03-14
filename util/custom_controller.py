from commands2.button import CommandXboxController, CommandPS4Controller

from util.math_helpers import clamp


class XboxController(CommandXboxController):
    """
    Custom XboxController class with extra features
    """

    def __init__(self, port):
        """
        Creates a new custom xbox controller object for command based
        :param port: Driverstation port for XBOX controller
        """
        super().__init__(port)

        self._deadband = 0
        self._mult = 1

    def with_deadband(self, deadband):
        """
        Sets the deadband of the controller,
        a zone where controller movements will not be used
        """
        self._deadband = deadband
        return self

    def with_mult(self, mult):
        """
        Simple multiplier to all joystick values
        """
        self._mult = mult
        return self

    def _apply(self, val):
        """
        Applies the custom controller settings
        """
        if abs(val) <= self._deadband:
            return 0

        # _val = val**3
        _val = val
        return clamp(_val, -1, 1)

    def getLeftX(self):
        _val = super().getLeftX()
        return self._apply(val=_val)

    def getLeftY(self):
        _val = super().getLeftY()
        return self._apply(val=_val)

    def getRightX(self):
        _val = super().getRightX()
        return self._apply(val=_val)

    def getRightY(self):
        _val = super().getRightY()
        return self._apply(val=_val)

# class PS4Controller(CommandPS4Controller):
    