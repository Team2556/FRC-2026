from commands2.button import CommandXboxController


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

        self.maxSmoothingExponent = 4.0

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
        _val = val
        _sign = 1.0 if _val > 0 else -1.0

        # Apply Deadband
        if abs(_val) <= self._deadband:
            return 0
        _val = (
            _sign * (abs(_val) - self._deadband) / (1.0 - self._deadband)
        )  # Remap the joystick values excluding deadband zone

        # Apply Smoothing
        _val = _sign ** 3

        # Apply Mult
        _val = _val * self._mult

        return _val

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
