
from components.chassis import Chassis
from components import shooter
from components import intake

import logging

class States:
    initialisation = 0
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

    def on_enable(self):
        """Set up the autonomous routine"""
        # Reset the IMU
        self.chassis.bno055.resetHeading()
        self.chassis.heading_hold = False
        self.chassis.autonomous = True
        self.state = States.goal_tracking

    def on_disable(self):
        """Cleanup after auto routine"""
        self.chassis.range_setpoint = 0.0
        self.chassis.track_vision = False
        self.shooter.change_state(shooter.States.off)
        self.intake.state = intake.States.no_ball

    def on_iteration(self, tm):
        """Run the autonomous routine, based on a state machine"""
        """if self.state == States.initialisation:
            self.chassis.start_odometry(5, 0.0)
            self.state = States.through_low_bar
        if self.state == States.through_low_bar:
            if self.chassis.odometry_on_target():
                self.state = states.strafing
                self.chassis.start_odometry(3, 45.0) # set the odometry to go to near the batter
        if self.state == States.strafing:
            if self.chassis.odometry_on_target():
                self.state = states.goal_tracking"""
        logging.getLogger('auto').info(self.state)
        if self.state == States.goal_tracking:
            # TODO - the trig to track the middle of the goal
            self.chassis.range_setpoint = 2.45 #m
            self.chassis.track_vision = True
            if abs(self.chassis.range_setpoint - self.chassis.range_finder.getDistance()) < 0.05 and abs(self.chassis.vy) < 0.1:
                self.state = States.firing
        if self.state == States.firing:
            self.shooter.change_state(shooter.States.firing)
            self.intake.state = states.fire

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

