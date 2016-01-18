from robot import *
from robot_map import RobotMap

def test_disabled(robot, control, fake_time):
    robot.robotInit()

    robot.running = {}
    # run disabled for 15 seconds
    control.set_autonomous(enabled=False)
    robot.disabledInit()
    for i in range(10):
        robot.disabledPeriodic()
    # run for 15 fake seconds
    #control.run_test(lambda tm: tm < 15)
    assert len(robot.running) == 0
    # ensure we ran for 15 seconds of fake time
    #assert int(fake_time.get()) == 15

def test_teleop(robot, control, fake_time, hal_data):
    robot.robotInit()

    # test that omni drive does run when its flag is true
    robot.running = {}
    robot.omni_driving= False
    control.set_operator_control(enabled = True)
    robot.teleopInit()
    for i in range(10):
        hal_data["joysticks"][0]["buttons"][11] = True
        robot.teleopPeriodic()
    # run for 15 fake seconds
    #control.run_test(lambda tm: tm < 15)
    assert len(robot.running) == 0
    # ensure we ran for 15 seconds of fake time
    #assert int(fake_time.get()) == 15

    # test that omni drive does not run when its flag is false
    robot.running = {}
    robot.omni_driving= True
    control.set_operator_control(enabled = True)
    for i in range(10):
        robot.teleopPeriodic()
    # run for 15 fake seconds
    #control.run_test(lambda tm: tm < 15)
    assert len(robot.running) == 1
    # ensure we ran for 15 seconds of fake time
    #assert int(fake_time.get()) == 15

def test_omni_drive(robot, control, fake_time, hal_data):
    robot.robotInit()
    epsilon = 0.05
    for module in robot.chassis._modules:
        module._direction = 0.0
        module._speed = 0.0
    robot.running = {}
    robot.omni_driving = True
    control.set_operator_control(enabled = True)
    # robot's x axis to 1.0
    hal_data['joysticks'][0][1] = -1.0
    # robot's throttle to 1.0
    hal_data['joysticks'][0][3] = -1.0
    robot.teleopInit()
    for i in range(10):
        robot.teleopPeriodic()
    #control.run_test(lambda tm: tm < 15)
    # ensure that the robot is going forwards
    for module in robot.chassis._modules:
        assert (module._speed - 1.0) < epsilon
    #assert int(fake_time.get()) - 15
