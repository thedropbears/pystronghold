
from components.chassis import Chassis, constrain_angle
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
    spinning = 5
    range_finding = 6

class LowBarTower:
    chassis = Chassis
    shooter = shooter.Shooter
    intake = intake.Intake

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
        self.chassis.field_oriented = False
        self.state = States.through_low_bar
        self.shooter.change_state(shooter.States.off)
        self.intake.stop()
        self.zero_encoders()
        self.vision_counts = 0

    def on_disable(self):
        """Cleanup after auto routine"""
        self.chassis.range_setpoint = 0.0
        self.chassis.track_vision = False
        self.shooter.change_state(shooter.States.off)
        self.intake.state = intake.States.no_ball

    def _generate_on_iteration(self, delta_x, delta_y, delta_heading=0.0):
        '''Drive forward the same amount, then move by delta_x and delta_y
        to the position where the vision and range finder take over.
        Final change in heading is specified too.'''
        strafe_distance = (delta_x ** 2 + delta_y ** 2) ** 0.5
        # Rescale velocity components to get a combined magnitude of 1
        vx = delta_x / strafe_distance
        vy = delta_y / strafe_distance
        def on_iteration(tm):
            dr_throttle = 0.5  # dead reckoning
            if self.state == States.through_low_bar:
                self.chassis.inputs[:] = (-1.0, 0.0, 0.0, dr_throttle)
                if self.distance > 3.4:  # This is always the same
                    self.chassis.heading_hold_pid.set(constrain_angle(self.chassis.heading_hold_pid.getSetpoint() + delta_heading))
                    self.state = States.spinning
            if self.state == States.spinning:
                if self.chassis.heading_hold_pid.onTarget():
                    self.zero_encoders()
                    self.state = States.strafing
            if self.state == States.strafing:
                final_throttle = self.chassis.range_pid.maximumOutput
                # scale throttle smoothly between dead reckoning throttle and range max throttle
                throttle = (final_throttle - dr_throttle) * self.distance / strafe_distance + dr_throttle
                self.chassis.inputs[:] = (vx, vy, 0.0, throttle)
                if self.distance > strafe_distance:
                    self.state = States.range_finding
                    self.chassis.range_pid.reset()
                    self.chassis.range_setpoint = 1.4  # m
                    self.chassis.inputs[:] = (0.0, 0.0, 0.0, 0.0)
            if self.state == States.range_finding:
                if self.chassis.range_pid.onTarget():
                    self.chassis.vision_pid.reset()
                    self.state = States.goal_tracking
            if self.state == States.goal_tracking:
                self.shooter.change_state(shooter.States.shooting)
                self.chassis.range_setpoint = 1.4  # m
                self.chassis.track_vision = True
                if self.chassis.vision_pid.onTarget():
                    self.vision_counts += 1
                else:
                    self.vision_counts = 0
                if self.chassis.range_pid.onTarget() and self.vision_counts >= 5:
                    self.state = States.firing
                    self.chassis.range_setpoint = 0.0
                    self.chassis.track_vision = False
            if self.state == States.firing:
                if self.shooter.state == shooter.States.shooting:
                    self.intake.state = intake.States.fire
                    self.chassis.field_oriented = True
        return on_iteration

class LowBarCentreTower(LowBarTower):
    MODE_NAME = "Low bar, CENTRE tower"
    DEFAULT = True

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        self.on_iteration = self._generate_on_iteration(1.2, -1.0, 0.0)


class LowBarLeftTower(LowBarTower):
    MODE_NAME = "Low bar, LEFT tower"
    DEFAULT = False

    def __init__(self):
        self.on_iteration = self._generate_on_iteration(1.4, -0.5, -math.pi / 3.0)


class LowBarRightTower(LowBarTower):
    MODE_NAME = "Low bar, RIGHT tower"
    DEFAULT = False

    def __init__(self):
        self.on_iteration = self._generate_on_iteration(1.4, -1.5, math.pi / 3.0)

