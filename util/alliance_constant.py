from typing import Generic, TypeVar
from wpilib import DriverStation

TYPE = TypeVar("type")


class AllianceConstantStruct(Generic[TYPE]):
    __slots__ = ("_red", "_blue")

    def __init__(self, red: TYPE, blue: TYPE):
        self._red = red
        self._blue = blue

    def get(self) -> TYPE:
        match DriverStation.getAlliance():
            case DriverStation.Alliance.kRed:
                return self._red
            case DriverStation.Alliance.kBlue:
                return self._blue
            case _:
                return None

def AllianceConstant(red: TYPE, blue: TYPE) -> TYPE:
    '''Creates a constant structure with two values for RED and BLUE. 
    It will automitcally choose the correct constant based on the DriverStation Alliance. 
    It excepts any type and can have mismatch types'''
    _struct = AllianceConstantStruct(red, blue)
    return _struct.get


test_constant = AllianceConstant(2, 1)
