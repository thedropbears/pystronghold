
import math
import pytest

from components.chassis import SwerveModule
from components.chassis import Chassis, constrain_angle, field_orient
from components import chassis

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

def test_dont_unwind_module(hal_data):
    # If the modules are "wound up" (ie have rotated full revolutions)
    # they should still go to the nearest correct angle
    # Set module pointing forward but with two full revolutions
    swerve = SwerveModule(0, 1)
    hal_data['CAN'][1]['pos'] = 2048
    swerve._steer.set(2048)
    swerve.steer(0.0, 1.0)
    assert abs(swerve.direction - 4.0 * math.pi) < epsilon
    swerve.steer(math.pi / 4.0, 1.0)
    assert abs(swerve.direction - 4.25 * math.pi) < epsilon

def reset_module(module):
        # Set controller setpoints
        module._drive.set(0.0)
        module._steer.set(0.0)
        # Steer with no speed will drive to absolute position
        module.steer(0.0)
        module._offset = 0.0

def reset_chassis(chassis):
    for module in chassis._modules.values():
        reset_module(module)

def test_chassis(robot, wpilib):
    epsilon = 0.01  # Tolerance for angular floating point errors (~0.05 degrees)
    chassis = Chassis()

    # vX is out the left side of the robot, vY is out of the front, vZ is upwards, so a +ve rotation is counter-clockwise
    #             vX   vY   vZ   throttle
    reset_chassis(chassis)
    # test x axis
    chassis.drive(1.0, 0.0, 0.0, 1.0)
    for name, module in chassis._modules.items():
        assert abs(constrain_angle(module.direction)) % math.pi  < epsilon
    reset_chassis(chassis)

    # test y axis
    chassis.drive(0.0, 1.0, 0.0, 1.0)
    for name, module in chassis._modules.items():
        # test weather each module is facing in the right direction
        assert abs(constrain_angle(math.pi / 2.0 - module.direction)) < epsilon
    reset_chassis(chassis)

    vz = {'a': math.atan2(-Chassis.length, Chassis.width),  # the angle that module a will go to if we spin on spot
          'b': math.atan2(Chassis.length, Chassis.width),
          'c': math.atan2(-Chassis.length, Chassis.width),
          'd': math.atan2(Chassis.length, Chassis.width)
          }

    chassis.drive(0.0, 0.0, 1.0, 1.0)

    for name, module in chassis._modules.items():
        assert abs(constrain_angle(module.direction - vz[name])) < epsilon
    reset_chassis(chassis)

    chassis.drive(1.0, 1.0, 0.0, 1.0)
    for module in chassis._modules.values():
        assert abs(constrain_angle(module.direction - math.pi / 4.0)) < epsilon
    reset_chassis(chassis)

def test_no_throttle(robot):
    epsilon = 0.01  # Tolerance for angular floating point errors (~0.05 degrees)
    chassis = Chassis()
    reset_chassis(chassis)
    # None for throttle should point the modules in the absolute direction
    # for diagnostic purposes
    chassis.drive(-1.0, -1.0, 0.0, None)
    for module in chassis._modules.values():
        assert abs(constrain_angle(module.direction + 3.0 / 4.0 * math.pi)) < epsilon
    reset_chassis(chassis)


def test_angular_displacement():
    assert abs(chassis.min_angular_displacement(0.0, math.pi / 4.0) - math.pi / 4.0) < epsilon
    assert abs(chassis.min_angular_displacement(0.0, math.pi * 3.0 / 4.0) - -math.pi / 4.0) < epsilon
    # With wrap around:
    assert abs(chassis.min_angular_displacement(4.0 * math.pi, math.pi / 4.0) - math.pi / 4.0) < epsilon
    assert abs(chassis.min_angular_displacement(4.0 * math.pi, math.pi * 3.0 / 4.0) - -math.pi / 4.0) < epsilon


def test_retain_wheel_direction():
    # When the joystick is returned to the centre, keep the last direction that the wheels were pointing
    chassis = Chassis()
    for name, module in chassis._modules.items():
        module.steer(math.pi / 4.0)
    chassis.drive(0.0, 0.0, 0.0, 1.0)
    for name, module in chassis._modules.items():
        assert abs(constrain_angle(module.direction - math.pi / 4.0)) < epsilon
    # Should not matter what the throttle is, even if it is zero
    chassis.drive(0.0, 0.0, 0.0, 0.0)
    for name, module in chassis._modules.items():
        assert abs(constrain_angle(module.direction - math.pi / 4.0)) < epsilon

def test_toggle_field_oriented():
    chassis = Chassis()
    field_o = chassis.field_oriented
    chassis.toggle_field_oriented()
    assert chassis.field_oriented is not field_o
    chassis.toggle_field_oriented()
    assert chassis.field_oriented is field_o

def test_toggle_track_vision():
    chassis = Chassis()
    vision = chassis.track_vision
    chassis.toggle_vision_tracking()
    assert chassis.track_vision is not vision
    chassis.toggle_vision_tracking()
    assert chassis.track_vision is vision

def test_toggle_range_holding():
    chassis = Chassis()
    chassis.range_setpoint = 0.0
    chassis.toggle_range_holding(2.0)
    assert chassis.range_setpoint == 2.0
    chassis.toggle_range_holding(2.0)
    assert chassis.range_setpoint == 0.0

def test_field_orient_calc():
    vx, vy = field_orient(1.0, 1.0, -1.0 / 4.0 * math.pi)
    assert abs(vx - 0.0) < epsilon
    assert abs(vy - 2.0 ** 0.5) < epsilon
    vx, vy = field_orient(1.0, 1.0, 1.0 / 4.0 * math.pi)
    assert abs(vx - 2.0 ** 0.5) < epsilon
    assert abs(vy - 0.0) < epsilon


