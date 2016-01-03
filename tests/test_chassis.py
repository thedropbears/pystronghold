
import pytest
import math

TAU = 2*math.pi # :P it had to happen... https://bugs.python.org/issue12345

from subsystems.chassis import SwerveModule
from subsystems.chassis import Chassis

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

def test_chassis():
    epsilon = 0.01 # Tolerance for angular floating point errors (~0.05 degrees)
    chassis = Chassis()

    # vX is out the left side of the robot, vY is out of the front, vZ is upwards, so a +ve rotation is counter-clockwise
    #             vX   vY   vZ   throttle
    chassis.drive(0.0, 0.0, 0.0, 0.0)
    chassis.zero_module_directions()
    for module in chassis._modules:
        assert module._speed == 0.0
        assert abs(module._direction) <= epsilon # make sure that the module has been zeroed

    #test x axis
    chassis.drive(1.0, 0.0, 0.0, 1.0)
    for module in chassis._modules:
        assert module._speed == 1.0
        assert abs(module._direction) <= episilon

    # test y axis
    chassis.drive(0.0, 1.0, 0.0, 1.0)
    for module in chassis._modules:
        # test weather each module is facing in the right direction
        assert (((TAU/4-epsilon)<=module._direction<=(TAU/4+epsilon) and module._speed == -1.0) or ((TAU*3/4-epsilon)<=module._direction<=(TAU*3/4+epsilon) and module._speed == 1.0))

    chassis.zero_module_directions()

    chasssis.drive(0.0, 0.0, 1.0, 1.0)

    assert (((TAU*225/360-epsilon)<=chassis._modules[0]._direction<=(TAU*225/360+epsilon) and chassis.modules[0]._speed == -1.0) or ((TAU*45/360-epsilon)<=chassis._modules[0]._direction<=(TAU*45/360+epsilon) and chassis.modules[0]._speed == 1.0))
    assert (((TAU*315/360-epsilon)<=chassis._modules[1]._direction<=(TAU*315/360+epsilon) and chassis.modules[1]._speed == -1.0) or ((TAU*135/360-epsilon)<=chassis._modules[1]._direction<=(TAU*135/360+epsilon) and chassis.modules[1]._speed == 1.0))
    assert (((TAU*45/360-epsilon)<=chassis._modules[2]._direction<=(TAU*45/360+epsilon) and chassis.modules[2]._speed == -1.0) or ((TAU*225/360-epsilon)<=chassis._modules[2]._direction<=(TAU*225/360+epsilon) and chassis.modules[2]._speed == 1.0))
    assert (((TAU*135/360-epsilon)<=chassis._modules[3]._direction<=(TAU*135/360+epsilon) and chassis.modules[3]._speed == -1.0) or ((TAU*315/360-epsilon)<=chassis._modules[3]._direction<=(TAU*315/360+epsilon) and chassis.modules[3]._speed == 1.0))
