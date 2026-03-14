import math


class kMath:
    InchesPerFoot = 12
    """inches / foot"""
    CentimetersPerInch = 2.54
    """centimeters / inch"""
    CentimetersPerMeter = 100
    """centimeters / meter"""
    MetersPerInch = CentimetersPerInch / CentimetersPerMeter
    """meters / inch"""
    MetersPer_Foot = MetersPerInch * InchesPerFoot
    """meters / foot"""
    RadiansPerRevolution = 2 * math.pi
    """radians / revolution"""
    DegreesPerRevolution = 360
    """degrees / revolution"""
    RadiansPerDegree = RadiansPerRevolution / DegreesPerRevolution
    """radians / degree"""
    MilisecondPerSecond = 1000 / 1
    """milliseconds / second"""
    SecondPerMinute = 60 / 1
    """seconds / minute"""
    RPMPerAngularVelocity = (1 / RadiansPerRevolution) * SecondPerMinute
    """RPM / (radians / second)"""
    AngularVelocityPerRPS = RadiansPerRevolution
    """(radians / second) / RPS"""
    GRAVITY = 9.802
    """m / s / s"""
    KilogramsToPounds = 0.454
    """kg/lb"""
