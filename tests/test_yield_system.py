from robot import *
from robot_map import RobotMap

import time

drive_fwd_seconds = 2
move_forward_seconds_button = 7

def test_command_system(robot, control, fake_time, hal_data):

    # test that the command system works and runs
    robot.omni_driving = False
    control.set_operator_control(enabled=True)
    hal_data["joysticks"][0]["buttons"][move_forward_seconds_button] = True
    control.run_test(lambda tm: tm < drive_fwd_seconds)
    assert len(robot.running) == 1
    for mod in robot.chassis._modules:
        assert mod._speed != 0.0
        assert mod._direction == 0.0

def test_disabled(robot, control, fake_time):
    robot.running = {}
    # run disabled for 15 seconds
    control.set_autonomous(enabled=False)
    # run for 15 fake seconds
    control.run_test(lambda tm: tm < 15)
    assert len(robot.running) == 0
    # ensure we ran for 15 seconds of fake time
    assert int(fake_time.get()) == 15

def test_omni_drive_disable(robot, control, fake_time, hal_data):

    # test that omni drive does run when its flag is true
    robot.running = {}
    robot.omni_driving= False
    control.set_operator_control(enabled = True)
    hal_data["joysticks"][0]["buttons"][move_forward_seconds_button] = True
    control.run_test(lambda tm: tm < 2)
    assert len(robot.running) == 1

def test_omni_drive(robot, control, fake_time, hal_data):
    epsilon = 0.05
    robot.running = {}
    robot.omni_driving = True
    control.set_operator_control(enabled = True)
    # robot's x axis to 1.0
    hal_data['joysticks'][0][1] = -1.0
    # robot's throttle to 1.0
    hal_data['joysticks'][0][3] = -1.0
    control.run_test(lambda tm: tm < drive_fwd_seconds)
    for module in robot.chassis._modules:
        assert (module._speed - 1.0) < epsilon

def test_autonomous_start(robot, control, fake_time, hal_data):
    robot.auto_tasks = move_forward_auto
    robot.omni_driving = False
    control.set_autonomous(enabled = True)
    control.run_test(lambda tm: tm < drive_fwd_seconds/2.0)
    assert len(robot.running) >= 1

def test_autonomous_end(robot, control, fake_time, hal_data):
    robot.auto_tasks = move_forward_auto
    robot.omni_driving = False
    control.set_autonomous(enabled = True)
    control.run_test(lambda tm: tm < drive_fwd_seconds+1)
    assert len(robot.running)
