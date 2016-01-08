
import pytest
import math

TAU = 2*math.pi # :P it had to happen... https://bugs.python.org/issue12345

from subsystems.chassis import SwerveModule
from subsystems.chassis import Chassis
from robot_map import RobotMap

def test_swerve_init(wpilib, hal_data):
    swerve = SwerveModule(0, 1)
    # Check that the drive and steer motor controllers have been created
    assert swerve._drive and isinstance(swerve._drive, wpilib.CANTalon)
    assert swerve._steer and isinstance(swerve._steer, wpilib.CANTalon)
    assert hal_data['CAN'][1]['feedback_device'] == wpilib.CANTalon.FeedbackDevice.AnalogEncoder

    # Test in relative encoder mode
    swerve = SwerveModule(2, 3, False)
    assert hal_data['CAN'][3]['feedback_device'] == wpilib.CANTalon.FeedbackDevice.QuadEncoder

def test_swerve_steer():
    epsilon = 0.01 # Tolerance for floating point errors (~0.5 degrees)
    swerve = SwerveModule(0, 1)
    # A call to SwerveModule.steer() with one argument should set the speed to 0
    swerve._speed = 1.0
    swerve.steer(0.1)
    assert swerve._speed == 0.0
    assert abs(swerve._direction - 0.1) < epsilon

    # Rescale values to the range [0, 2*pi)
    swerve._direction = 0.0
    swerve.steer(2.1*math.pi)
    assert abs(swerve._direction - 0.1*math.pi) < epsilon
    swerve._direction = 0.0
    swerve.steer(-2.1*math.pi)
    assert abs(swerve._direction - -0.1*math.pi) < epsilon

    # Make sure the swerve module calculates the quickest way to the desired heading
    swerve._direction = 0.0
    swerve.steer(math.pi, 1.0)
    assert swerve._speed == -1.0
    assert abs(swerve._direction) < epsilon

def test_chassis(robot):
    epsilon = 0.01 # Tolerance for angular floating point errors (~0.05 degrees)
    chassis = Chassis(robot)

    # vX is out the left side of the robot, vY is out of the front, vZ is upwards, so a +ve rotation is counter-clockwise
    #             vX   vY   vZ   throttle
    chassis.drive(0.0, 0.0, 0.0, 0.0)
    for module in chassis._modules:
        assert module._speed == 0.0
        assert abs(module._direction) <= epsilon # make sure that the module has been zeroed
        module._direction = 0.0
        module._speed = 0.0

    #test x axis
    chassis.drive(1.0, 0.0, 0.0, 1.0)
    for module in chassis._modules:
        assert module._direction == 0.0
        module._direction = 0.0
        module._speed = 0.0

    # test y axis
    chassis.drive(0.0, 1.0, 0.0, 1.0)
    for module in chassis._modules:
        # test weather each module is facing in the right direction
        assert TAU/4.0 == module._direction
        module._direction = 0.0
        module._speed = 0.0

    vz_a = math.atan2(-RobotMap.robot_width, RobotMap.robot_length) #the angle that module a will go to if we spin on spot
    vz_b = math.atan2(RobotMap.robot_width, RobotMap.robot_length)
    vz_c = math.atan2(-RobotMap.robot_width, RobotMap.robot_length)
    vz_d = math.atan2(RobotMap.robot_width, RobotMap.robot_length)

    vectors = [vz_a, vz_b, vz_c, vz_d]

    chassis.drive(0.0, 0.0, 1.0, 1.0)

    for module, vector in zip(chassis._modules, vectors):
        assert abs(module._direction - vector) < epsilon
        module._direction = 0.0
        module._speed = 0.0

    chassis.drive(1.0, 1.0, 0.0, 1.0)

    """
    for module in chassis._modules:
        assert module._direction == TAU/8.0

    chassis.drive(1.0, 0.0, 1.0, 1.0)

    for module, vector in zip(chassis._modules, vectors):
        assert abs(module._direction-vector/2.0) < epsilon
        module._direction = 0.0
        module._speed = 0.0"""

def test_angular_displacement():
    module = SwerveModule(0, 1, absoluteEncoder=False)

    current_position = 0.0
    desired_position = math.pi/4.0
    opposite_desired_position = -3.0*math.pi/4.0

    assert module.angularDisplacement(desired_position, opposite_desired_position, current_position) == math.pi/4.0
