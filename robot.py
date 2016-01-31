#!/usr/bin/env python3

import wpilib
import time

from subsystems import Chassis
from subsystems import Vision
from subsystems import BNO055
from subsystems import RangeFinder
from subsystems import Shooter
from oi import OI

from robot_map import RobotMap
import logging
import multiprocessing
import math

def omni_drive(robot):
    while robot.omni_driving:
        if robot.oi.joystick.getPOV() == -1:
            robot.chassis.drive(robot.oi.getJoystickY(),
                                robot.oi.getJoystickX(),
                                robot.oi.getJoystickZ(),
                                robot.oi.getThrottle()
                                )
        else:
            robot.chassis.drive(math.cos(robot.oi.joystick.getPOV()*math.pi/180.0),
                                -math.sin(robot.oi.joystick.getPOV()*math.pi/180.0),
                                0.0,
                                None
                                )
        yield

def move_forward_time(robot):
    robot.omni_driving = False
    tm = time.time()
    while time.time() - tm < 2:  # Drive for 2 seconds
        robot.chassis.drive(1.0, 0.0, 0.0, 1.0)
        yield

def strafe_with_vision(robot):
    robot.omni_driving = False
    x_offset = 0.0
    alpha = 0.7
    while True:
        with robot.vision_lock:
            if robot.vision_array[4] == 1:  # New value available
                if robot.vision_array[3] == 0.0:
                    robot.chassis.drive(0.0, 1.0, 0.0, 0.0)
                    yield
                else:
                    x_offset = alpha * robot.vision_array[0] + (1.0 - alpha) * x_offset
                    robot.chassis.drive(0.0, x_offset, 0.0, 0.5)
                robot.vision_array[4] = 0  # Reset the "new value" flag
        yield

def move_with_rangefinder(robot):
    robot.omni_driving = False
    x_offset_cm = 0.0
    x_offset_scaled = 0.0
    desired_dist = 200  # cm
    max_throttle = 0.3
    alpha = 0.7
    robot.logger.info("Rangefinder command init")
    while True:
        dist = robot.range_finder.getDistance()
        x_offset_cm = dist - desired_dist
        x_offset_scaled = x_offset_cm / desired_dist * max_throttle * alpha + x_offset_scaled * (1 - alpha)
        if x_offset_scaled >= max_throttle:
            x_offset_scaled = max_throttle
        elif x_offset_scaled <= -max_throttle:
            x_offset_scaled = -max_throttle
        robot.logger.info(-x_offset_scaled)
        robot.chassis.drive(-x_offset_scaled, 0.0, 0.0, 1.0)
        yield

def move_with_rangefinder_and_vision(robot):
    robot.omni_driving = False
    robot.field_oriented = False
    range_finder_offset_cm = 0.0
    range_finder_offset_scaled = 0.0
    desired_dist = 200  # cm
    max_throttle = 0.3
    range_alpha = 0.7
    vision_offset = 0.0
    vision_alpha = 0.7
    while True:
        dist = robot.range_finder.getDistance()
        range_finder_offset_cm = dist - desired_dist
        range_finder_offset_scaled = range_finder_offset_cm / desired_dist * max_throttle * range_alpha + range_finder_offset_scaled * (1 - range_alpha)
        if range_finder_offset_scaled >= max_throttle:
            range_finder_offset_scaled = max_throttle
        elif range_finder_offset_scaled <= -max_throttle:
            range_finder_offset_scaled = -max_throttle
        with robot.vision_lock:
            if robot.vision_array[3] == 0.0:
                robot.chassis.drive(-range_finder_offset_scaled, 0.0, 0.0, 1.0)
            else:
                vision_offset = vision_alpha * robot.vision_array[0] * max_throttle + (1.0 - vision_alpha) * vision_offset
                robot.chassis.drive(-range_finder_offset_scaled, vision_offset, 0.0, 1.0)
            robot.vision_array[4] = 0.0
        yield

def drive_motors(robot):
    robot.omni_driving = False
    robot.chassis.drive(0.0, 0.0, 0.0, 0.0)
    while not robot.omni_driving:
        robot.drive_motors.drive(robot.oi.getThrottle() * 2.0 - 1.0)
        robot.logger.info(robot.oi.getThrottle() * 2.0 - 1.0)
        yield

def toggle_field_oriented(robot):
    robot.field_oriented = not robot.field_oriented
    while robot.oi.joystick.getRawButton(robot.oi.field_orient_toggle_button):  # wait for 1 sec before toggling again
        yield

def reset_gyro(robot):
    robot.bno055.resetHeading()
    while robot.oi.joystick.getRawButton(robot.oi.gyro_reset_button):  # wait for 1 sec before toggling again
        yield

taskmap = {2:reset_gyro,
           3:toggle_field_oriented,
           7:move_forward_time,
           8:drive_motors,
           9:strafe_with_vision,
           10:move_with_rangefinder,
           11:move_with_rangefinder_and_vision}


move_forward_auto = [[move_forward_time]]

class StrongholdRobot(wpilib.IterativeRobot):

    logger = logging.getLogger("robot")
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.running = {}
        self.omni_driving = True
        self.field_oriented = True
        self.omni_drive = omni_drive
        self.shooter = Shooter(self)
        self.oi = OI(self)
        self.range_finder = RangeFinder()
        self.bno055 = BNO055()
        self.chassis = Chassis(self)
        self.auto_tasks = move_forward_auto  # [[list, of, tasks, to_go, through, sequentially], [and, this, list, will, run, in, parallel]
        self.current_auto_tasks = []
        self.vision_array = multiprocessing.Array("d", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.vision_terminate_event = multiprocessing.Event()
        self.vision_lock = multiprocessing.Lock()
        self.vision = Vision(self.vision_array, self.vision_terminate_event, self.vision_lock)

    def disabledInit(self):
        self.vision_terminate_event.clear()

    def disabledPeriodic(self):
        """This function is called periodically when disabled."""
        self.running = {}
        self.vision_terminate_event.clear()
        # self.logger.info("Euler: %f,%f,%f" % tuple(self.bno055.getAngles()))
        # self.logger.info("Rangefinder: " + str(self.range_finder.getDistance()))

    def autonomousInit(self):
        self.running = {}
        self.current_auto_tasks = self.auto_tasks
        self.vision_terminate_event.set()
        try:
            self.vision.start()
        except:
            pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # clear empty task sequences
        for i in range(len(self.auto_tasks)):
            if len(self.auto_tasks[i]) == 0:
                self.auto_tasks[i].pop()
        for command_sequence in self.current_auto_tasks:
            if command_sequence[0] not in self.running:
                ifunc = command_sequence[0](self).__next__
                self.running[command_sequence[0]] = ifunc
        done = []
        for key, ifunc in self.running.items():
            try:
                ifunc()
            except StopIteration:
                done.append(key)
        for key in done:
            self.running.pop(key)
            # remove the done task from our list of auto commands
            for command_sequence in self.current_auto_tasks:
                if key == command_sequence[0]:
                    command_sequence.pop(0)

    def teleopInit(self):
        self.running = {}
        self.omni_driving = True
        self.vision_terminate_event.set()
        try:
            self.vision.start()
        except:
            pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        for button, task in taskmap.items():
            # if the button for the task is pressed and the task is not already running
            if self.oi.joystick.getRawButton(button) and task not in self.running:
                ifunc = task(self).__next__
                self.running[task] = ifunc
        if self.omni_driving and self.omni_drive not in self.running:
            ifunc = self.omni_drive(self).__next__
            self.running[self.omni_drive] = ifunc
        done = []
        for key, ifunc in self.running.items():
            try:
                ifunc()
            except StopIteration:
                done.append(key)
        for key in done:
            self.running.pop(key)
        # self.logger.info("Teleop periodic vision: " + str(self.vision_array[0]))

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(StrongholdRobot)
