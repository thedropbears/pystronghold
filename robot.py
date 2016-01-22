#!/usr/bin/env python3

import wpilib
import time
from wpilib import command

from subsystems import Chassis
from subsystems import Vision
from subsystems import DriveMotors
from oi import OI

from robot_map import RobotMap
import logging
from multiprocessing import Array, Event

def omni_drive(robot):
    while robot.omni_driving:
        robot.chassis.drive(robot.oi.getJoystickY(), robot.oi.getJoystickX(), robot.oi.getJoystickZ(), robot.oi.getThrottle())
        yield

def move_forward_time(robot):
    robot.omni_driving = False
    tm = time.time()
    while time.time()-tm < RobotMap.move_forward_seconds:
        robot.chassis.drive(1.0, 0.0, 0.0, 1.0)
        yield

def strafe_with_vision(robot):
    robot.omni_driving = False
    while True:
        if robot.vision_array[0] < -0.05:
            robot.chassis.drive(0.0, 0.1, 0.0, 0.5)
        elif robot.vision_array[0] > 0.05:
            robot.chassis.drive(0.0, -0.1, 0.0, 0.5)
        yield


def drive_motors(robot):
    robot.omni_driving = False
    robot.chassis.drive(0.0, 0.0, 0.0, 0.0)
    while not robot.omni_driving:
        robot.drive_motors.drive(robot.oi.getThrottle()*2.0-1.0)
        robot.logger.info(robot.oi.getThrottle()*2.0-1.0)
        logging.getLogger("robotpy").info(robot.oi.getThrottle()*2.0-1.0)
        yield

taskmap = {RobotMap.move_forward_seconds_button:move_forward_time, 9:strafe_with_vision, 8:drive_motors}

move_forward_auto = [[move_forward_time]]

class StrongholdRobot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.logger = logging.getLogger("robotpy")
        self.running = {}
        self.omni_driving = True
        self.omni_drive = omni_drive
        self.drive_motors = DriveMotors(self)
        self.oi = OI(self)
        self.chassis = Chassis(self)
        self.auto_tasks = move_forward_auto # [[list, of, tasks, to_go, through, sequentially], [and, this, list, will, run, in, parallel]
        self.current_auto_tasks = []
        self.vision_array = Array("d", [0.0, 0.0, 0.0, 0.0])
        self.vision_terminate_event = Event()
        self.vision = Vision(self.vision_array, self.vision_terminate_event)

    def disabledInit(self):
        self.vision_terminate_event.clear()

    def disabledPeriodic(self):
        """This function is called periodically when disabled."""
        self.running = {}
        self.vision_terminate_event.clear()

    def autonomousInit(self):
        self.running = {}
        self.current_auto_tasks = self.auto_tasks
        self.vision_terminate_event.set()
        if not self.vision.is_alive:
            self.vision.start()

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
        if not self.vision.is_alive:
            self.vision.start()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        for button, task in taskmap.items():
            #if the button for the task is pressed and the task is not already running
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
        self.logger.info(self.vision_array[0])

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(StrongholdRobot)
