
from components.chassis import Chassis, constrain_angle
from components import shooter
from components import intake
from components import defeater
from components import bno055
from wpilib import CANTalon
import logging

import math, time


class States:
    init = 0
    through_obstacle = 1
    strafing = 2
    goal_tracking = 3
    firing = 4
    spinning = 5
    range_finding = 6


class ObstacleHighGoal:
    chassis = Chassis
    shooter = shooter.Shooter
    intake = intake.Intake
    defeater = defeater.Defeater
    defeater_motor = CANTalon
    bno055 = bno055.BNO055

    def __init__(self, delta_x, delta_y, delta_heading=0.0, portcullis=False):
        self.straight = 5.0
        self.delta_x = delta_x
        self.delta_y = delta_y
        self.delta_heading = delta_heading
        self.portcullis = portcullis
        self.strafe_distance = (delta_x ** 2 + delta_y ** 2) ** 0.5
        # Rescale velocity components to get a combined magnitude of 1
        self.vx = delta_x / self.strafe_distance
        self.vy = delta_y / self.strafe_distance
        self.logger = logging.getLogger("auto")

    def on_enable(self):
        """Set up the autonomous routine"""
        # Reset the IMU
        self.bno055.resetHeading()
        self.chassis.set_heading_setpoint(self.bno055.getAngle())
        self.chassis.heading_hold_pid.reset()
        self.chassis.heading_hold = True
        self.chassis.field_oriented = True
        self.chassis.drive(1, 0, 0, 0.0001)
        self.state = States.init
        self.shooter.change_state(shooter.States.off)
        self.intake.stop()
        self.chassis.zero_encoders()
        self.vision_counts = 0
        self.timeout = 0
        self.start_time = time.time()

    def on_disable(self):
        """Cleanup after auto routine"""
        self.chassis.range_setpoint = 0.0
        self.chassis.track_vision = False
        self.shooter.change_state(shooter.States.off)
        self.intake.state = intake.States.no_ball

    def on_iteration(self, tm):
        '''Drive forward the same amount, then move by delta_x and delta_y
        to the position where the vision and range finder take over.
        Final change in heading is specified too.'''
        if self.state == States.init:
            if self.portcullis:
                self.defeater_motor.set(-0.5)
            if not self.chassis.onTarget():
                return
            self.chassis.field_displace(self.straight, 0.0)
            self.state = States.through_obstacle
            self.logger.info("SETPOINT: " + str(self.chassis.distance_pid.getSetpoint()))
        if self.state == States.through_obstacle and self.chassis.distance_pid.onTarget():
            # Let the distance PID do its magic...
            # Turn off the distance PID, and spin to the right angle
            self.logger.info("Obstacle finished, distance: " + str(self.chassis.distance))
            self.chassis.distance_pid.disable()
            self.chassis.heading_hold_pid.setSetpoint(constrain_angle(self.chassis.heading_hold_pid.getSetpoint() + self.delta_heading))
            self.defeater_motor.set(0.3)
            self.state = States.spinning
        if self.state == States.spinning and self.chassis.heading_hold_pid.onTarget():
            # Turn on the distance PID for the next displacement
            self.chassis.field_displace(self.delta_x, self.delta_y)
            self.state = States.strafing
        if self.state == States.strafing and self.chassis.distance_pid.onTarget():
            # Dead reckoning is done - engage the rangefinder
            # Leave the distance PID running as it will read the rf for us
            self.logger.info("Strafing finished, distance: " + str(self.chassis.distance))
            self.state = States.range_finding
            self.chassis.distance_pid.setSetpoint(0.0)
            self.chassis.distance_pid.reset()
            self.chassis.range_setpoint = self.chassis.correct_range  # m
            self.chassis.distance_pid.enable()
        if self.state == States.range_finding and self.chassis.on_range_target(): #self.chassis.distance_pid.onTarget():
            # Range is good, now turn on the vision tracking
            self.chassis.track_vision = True
            self.chassis.distance_pid.setSetpoint(0.0)
            self.chassis.distance_pid.reset()
            self.state = States.goal_tracking
            self.logger.info("On range, distance: " + str(self.chassis.distance))
            self.shooter.change_state(shooter.States.shooting)
            self.chassis.distance_pid.enable()
        if (self.state == States.goal_tracking and self.chassis.on_vision_target()) or ((time.time() - self.start_time) > 12): #self.chassis.distance_pid.onTarget():
            # We made it to the target point, so fire away!
            self.shooter.change_state(shooter.States.shooting)
            self.intake.state = intake.States.fire
            self.state = States.firing
            self.chassis.range_setpoint = 0.0
            self.chassis.track_vision = False
        if self.state == States.firing and self.shooter.state == shooter.States.shooting:
            self.shooter.change_state(shooter.States.shooting)
            self.intake.state = intake.States.fire
            self.chassis.field_oriented = True

class LowBarCentreTower(ObstacleHighGoal):
    MODE_NAME = "Low bar, CENTRE tower"
    DEFAULT = True

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        #super().__init__(2.0, -1.8, 0.0)
        #should be correct for real field
        super().__init__(1.2, -3.8, 0.0)

class LowBarCentreTowerTest(ObstacleHighGoal):
    MODE_NAME = "TEST MODE: Low bar, CENTRE tower"
    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        #super().__init__(2.0, -1.8, 0.0)
        #should be correct for real field
        super().__init__(0.8, -0.2, 0.0)


class LowBarLeftTower(ObstacleHighGoal):
    MODE_NAME = "Low bar, LEFT tower"

    def __init__(self):
        super().__init__(1.4, -0.5, -math.pi / 3.0)


class LowBarRightTower(ObstacleHighGoal):
    MODE_NAME = "Low bar, RIGHT tower"

    def __init__(self):
        super().__init__(1.4, -1.5, math.pi / 3.0)


class Portcullis2CentreTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 2, CENTRE tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis2LeftTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 2, LEFT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis2RightTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 2, RIGHT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis3CentreTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 3, CENTRE tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis3LeftTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 3, LEFT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis3RightTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 3, RIGHT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis4CentreTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 4, CENTRE tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis4LeftTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 4, LEFT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis4RightTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 4, RIGHT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis5CentreTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 5, CENTRE tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis5LeftTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 5, LEFT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)


class Portcullis5RightTower(ObstacleHighGoal):
    MODE_NAME = "Portcullis position 5, RIGHT tower"

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, 0.0, 0.0, True)
