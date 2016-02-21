
from components.chassis import Chassis
from components import shooter
from components import intake

from networktables import NetworkTable

import logging

import math

class States:
    through_low_bar = 1
    strafing = 2
    goal_tracking = 3
    firing = 4

class LowBarTower:
    LEFT = 1
    CENTRE = 2
    RIGHT = 3

    chassis = Chassis
    shooter = shooter.Shooter
    intake = intake.Intake

    #strafe_vx = 2.4
    #strafe_vy = 3.8
    strafe_vx = 1.2
    strafe_vy = 1.2
    strafe_distance = math.sqrt(strafe_vx**2 + strafe_vy**2)

    @property
    def distance(self):
        distances = 0.0
        for module in self.chassis._modules.values():
            distances += abs(module._drive.getEncPosition())/module.drive_counts_per_metre
        return distances/4.0

    def zero_encoders(self):
        for module in self.chassis._modules.values():
            module._drive.setPosition(0.0)

    def on_enable(self):
        """Set up the autonomous routine"""
        # Reset the IMU
        self.chassis.bno055.resetHeading()
        self.chassis.set_heading_setpoint(self.chassis.bno055.getAngle())
        self.chassis.heading_hold = True
        self.chassis.autonomous = True
        self.state = States.through_low_bar
        self.shooter.change_state(shooter.States.off)
        self.intake.stop()
        self.zero_encoders()
        self.sd = NetworkTable.getTable('SmartDashboard')

    def on_disable(self):
        """Cleanup after auto routine"""
        self.chassis.range_setpoint = 0.0
        self.chassis.track_vision = False
        self.shooter.change_state(shooter.States.off)
        self.intake.state = intake.States.no_ball

    def on_iteration(self, tm):
        self.sd.putDouble('auto_state', self.state)
        if self.state == States.through_low_bar:
            self.chassis.inputs[:] = (1.0, 0.0, 0.0, 0.7)
            if self.distance > 3.4:
                self.zero_encoders()
                self.state = States.strafing
        if self.state == States.strafing:
            self.chassis.inputs[:] = (self.strafe_vx, -self.strafe_vy, 0.0, 0.7)
            if self.distance > self.strafe_distance:
                self.state = States.goal_tracking
                self.chassis.inputs[:] = (0.0, 0.0, 0.0, 1.0)
        if self.state == States.goal_tracking:
            # TODO - the trig to track the middle of the goal
            self.chassis.range_setpoint = 1.4 #m
            self.chassis.track_vision = True
            if self.chassis.range_pid.onTarget() and self.chassis.vision_pid.onTarget():
                self.state = States.firing
                self.chassis.range_setpoint = 0.0
                self.chassis.track_vision = False
        if self.state == States.firing:
            self.shooter.change_state(shooter.States.shooting)
            self.intake.state = intake.States.fire
            self.chassis.field_oriented = True

class LowBarCentreTower(LowBarTower):
    MODE_NAME = "Low bar, CENTRE tower"
    DEFAULT = True

    target = LowBarTower.CENTRE


class LowBarLeftTower(LowBarTower):
    MODE_NAME = "Low bar, LEFT tower"
    DEFAULT = False

    target = LowBarTower.LEFT


class LowBarRightTower(LowBarTower):
    MODE_NAME = "Low bar, RIGHT tower"
    DEFAULT = False

    target = LowBarTower.RIGHT

