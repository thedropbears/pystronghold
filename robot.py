#!/usr/bin/env python3

import wpilib
import time
from wpilib import command

#from subsystems.example_subsystem import ExampleSubsystem
from subsystems import Chassis
#from commands.example_command import ExampleCommand
from oi import OI

import logging

def omni_drive(robot):
    while robot.omni_driving:
        robot.chassis.drive(robot.oi.getJoystickY(), robot.oi.getJoystickX(), robot.oi.getJoystickZ(), robot.oi.getThrottle())
        yield

def log_message(robot):
    time = time.time()
    robot.logger.info("log message run")
    print("logged")
    #only run every 3 seconds
    while time.time()-time < 3.0:
        yield


class StrongholdRobot(wpilib.IterativeRobot):

    #example_subsystem = ExampleSubsystem()

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.running = {}
        self.omni_driving = True
        self.omni_drive = omni_drive
        self.oi = OI(self)
        self.chassis = Chassis(self)
        self.logger = logging.getLogger("robotpy")
        #Create the command used for the autonomous period
        #self.autonomous_command = ExampleCommand(self)
        self.taskmap = {11:log_message}
        self.auto_tasks = [] # [[list, of, tasks, to_go, through, sequentially], [and, this, list, will, run, in, parallel]
        self.current_auto_tasks = []

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        """This function is called periodically when disabled."""
        self.running = {}

    def autonomousInit(self):
        #Schedule the autonomous command
        #self.autonomous_command.start()
        self.running = {}
        self.current_auto_tasks = self.auto_tasks

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # clear empty task sequences
        for i in range(len(self.auto_tasks)):
            if len(self.auto_tasks[i]) == 0:
                self.auto_tasks[i].pop()
        for command_sequence in self.auto_tasks:
            if command_sequence[0] not in self.running:
                ifunc = command_sequence[0](self).__next__
                self.running[command_sequence[0]] = ifunc
        done = []
        for key, ifunc in self.running.items():
            try:
                ifunc(self)
            except StopIteration:
                done.append(key)
        for key in done:
            self.running.pop(key)
            # remove the done task from our list of auto commands
            for command_sequence in self.running:
                if command_sequence[0] == done:
                    command_seqeuence[0].pop()

    def teleopInit(self):
        self.running = {}

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        for button, task in self.taskmap.items():
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

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(StrongholdRobot)
