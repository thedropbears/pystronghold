
import pytest
import math

from subsystems.chassis import SwerveModule

def test_swerve_init(wpilib, hal_data):
    swerve = SwerveModule(0, 1)
    # Check that the drive and steer motor controllers have been created
    assert swerve._drive and isinstance(swerve._drive, wpilib.CANTalon)
    assert swerve._steer and isinstance(swerve._steer, wpilib.CANTalon)
    assert hal_data['CAN'][1]['feedback_device_select'] == wpilib.CANTalon.FeedbackDevice.AnalogEncoder
    
    # Test in relative encoder mode
    swerve = SwerveModule(2, 3, False)
    assert hal_data['CAN'][3]['feedback_device_select'] == wpilib.CANTalon.FeedbackDevice.QuadEncoder

def test_swerve_steer():
    epsilon = 0.01 # Tolerance for floating point errors (~0.5 degrees)
    swerve = SwerveModule(0, 1)
    # A call to SwerveModule.steer() with one argument should set the speed to 0
    swerve._speed = 1.0
    swerve.steer(2.0)
    assert swerve._speed == 0.0
    assert swerve._direction == 2.0

    # Rescale values to the range [0, 2*pi)
    swerve._direction = 0.0
    swerve.steer(2.1*math.pi)
    assert abs(swerve._direction - 0.1*math.pi) < epsilon
    swerve.steer(-0.1*math.pi)
    assert abs(swerve._direction - 1.9*math.pi) < epsilon

    # Make sure the swerve module calculates the quickest way to the desired heading
    swerve._direction = 0.0
    swerve.steer(math.pi, 1.0)
    assert swerve._speed == -1.0
    assert abs(swerve._direction) < epsilon
