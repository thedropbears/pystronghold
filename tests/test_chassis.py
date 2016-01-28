
import math

from subsystems.chassis import SwerveModule
from subsystems.chassis import Chassis
from subsystems import chassis

epsilon = 0.01  # Tolerance for floating point errors (~0.5 degrees)

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
    swerve = SwerveModule(0, 1)
    # A call to SwerveModule.steer() with one argument should set the speed to 0
    swerve._drive.set(1.0)
    swerve.steer(0.1)
    assert swerve.speed == 0.0
    assert abs(swerve.direction - 0.1) < epsilon

    # Rescale values to the range [0, 2*pi)
    reset_module(swerve)
    swerve.steer(2.1 * math.pi)
    assert abs(swerve.direction - 0.1 * math.pi) < epsilon
    reset_module(swerve)
    swerve.steer(-2.1 * math.pi)
    assert abs(swerve.direction - -0.1 * math.pi) < epsilon

    # Make sure the swerve module calculates the quickest way to the desired heading
    reset_module(swerve)
    swerve.steer(math.pi, 1.0)
    assert swerve.speed == -1.0
    assert abs(swerve.direction) < epsilon

def reset_module(module):
        # Set controller setpoints
        module._drive.set(0.0)
        module._steer.set(0.0)
        # Steer with no speed will drive to absolute position
        module.steer(0.0)

def reset_chassis(chassis):
    for module in chassis._modules:
        reset_module(module)

def test_chassis(robot):
    epsilon = 0.01  # Tolerance for angular floating point errors (~0.05 degrees)
    robot.robotInit()
    chassis = robot.chassis

    # vX is out the left side of the robot, vY is out of the front, vZ is upwards, so a +ve rotation is counter-clockwise
    #             vX   vY   vZ   throttle
    chassis.drive(0.0, 0.0, 0.0, 0.0)
    for module in chassis._modules:
        assert module.speed == 0.0
        assert abs(module.direction) <= epsilon  # make sure that the module has been zeroed
    reset_chassis(chassis)

    # test x axis
    chassis.drive(1.0, 0.0, 0.0, 1.0)
    for module in chassis._modules:
        assert module.direction == 0.0
    reset_chassis(chassis)

    # test y axis
    chassis.drive(0.0, 1.0, 0.0, 1.0)
    for module in chassis._modules:
        # test weather each module is facing in the right direction
        assert math.pi / 2.0 == module.direction
    reset_chassis(chassis)

    vz_a = math.atan2(-Chassis.length, Chassis.width)  # the angle that module a will go to if we spin on spot
    vz_b = math.atan2(Chassis.length, Chassis.width)
    vz_c = math.atan2(-Chassis.length, Chassis.width)
    vz_d = math.atan2(Chassis.length, Chassis.width)

    vectors = [vz_a, vz_b, vz_c, vz_d]

    chassis.drive(0.0, 0.0, 1.0, 1.0)

    for module, vector in zip(chassis._modules, vectors):
        assert abs(module.direction - vector) < epsilon
    reset_chassis(chassis)

    chassis.drive(1.0, 1.0, 0.0, 1.0)
    for module in chassis._modules:
        assert module.direction == math.pi / 4.0

    reset_chassis(chassis)

def test_angular_displacement():
    assert abs(chassis.min_angular_displacement(0.0, math.pi / 4.0) - math.pi / 4.0) < epsilon
    assert abs(chassis.min_angular_displacement(0.0, math.pi * 3.0 / 4.0) - -math.pi / 4.0) < epsilon

def test_retain_wheel_direction(robot):
    # When the joystick is returned to the centre, keep the last direction that the wheels were pointing
    robot.robotInit()
    chassis = robot.chassis
    for module in chassis._modules:
        module.steer(math.pi / 4.0)
    chassis.drive(0.0, 0.0, 0.0, 1.0)
    for module in chassis._modules:
        assert module.direction == math.pi / 4.0
    # Should not matter what the throttle is, even if it is zero
    chassis.drive(0.0, 0.0, 0.0, 0.0)
    for module in chassis._modules:
        assert module.direction == math.pi / 4.0

