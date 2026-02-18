import math


class kMath:
    INCH_FT = 12
    """inches / foot"""
    CM_INCH = 2.54
    """centimeters / inch"""
    CM_M = 100
    """centimeters / meter"""
    M_INCH = CM_INCH / CM_M
    """meters / inch"""
    M_FT = M_INCH * INCH_FT
    """meters / foot"""
    RAD_REV = 2 * math.pi
    """radians / revolution"""
    DEG_REV = 360
    """degrees / revolution"""
    RAD_DEG = RAD_REV / DEG_REV
    """radians / degree"""
    mS_S = 1000 / 1
    """milliseconds / second"""
    S_MIN = 60 / 1
    """seconds / minute"""
    RPM_ANGULARVEL = (1 / RAD_REV) * S_MIN
    """RPM / (radians / second)"""
    GRAVITY = 9.802
    """m / s / s"""
    KG_LB = 0.454
    """kg/lb"""