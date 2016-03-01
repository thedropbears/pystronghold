
from components.chassis import Chassis, constrain_angle
from components import shooter
from components import intake
from components import defeater
from wpilib import CANTalon

import math


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

    def __init__(self, delta_x, delta_y, delta_heading=0.0, portcullis=False):
        self.straight = 3.4
        self.delta_x = delta_x
        self.delta_y = delta_y
        self.delta_heading = delta_heading
        self.portcullis = portcullis
        self.strafe_distance = (delta_x ** 2 + delta_y ** 2) ** 0.5
        # Rescale velocity components to get a combined magnitude of 1
        self.vx = delta_x / self.strafe_distance
        self.vy = delta_y / self.strafe_distance

    def on_enable(self):
        """Set up the autonomous routine"""
        # Reset the IMU
        self.chassis.bno055.resetHeading()
        self.chassis.set_heading_setpoint(self.chassis.bno055.getAngle())
        self.chassis.heading_hold = True
        self.chassis.field_oriented = True
        self.chassis.drive(1, 0, 0, 0.0001)
        self.state = States.init
        self.shooter.change_state(shooter.States.off)
        self.intake.stop()
        self.chassis.zero_encoders()
        self.vision_counts = 0
        self.timeout = 0

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
        dr_throttle = 0.4  # dead reckoning
        if self.state == States.init:
            if self.portcullis:
                self.defeater_motor.set(-0.5)
            if not self.chassis.onTarget():
                return
            if not abs(self.chassis.distance) < 0.01:
                self.chassis.zero_encoders()
                return
            self.state = States.through_obstacle
        if self.state == States.through_obstacle:
            self.chassis.inputs[:] = (1.0, 0.0, 0.0, dr_throttle)
            if self.chassis.distance > self.straight:  # This is always the same
                self.chassis.heading_hold_pid.setSetpoint(constrain_angle(self.chassis.heading_hold_pid.getSetpoint() + self.delta_heading))
                self.chassis.inputs[:] = (0.0, 0.0, 0.0, 0.0)
                self.state = States.spinning
        if self.state == States.spinning:
            self.defeater_motor.set(0.3)
            if self.chassis.heading_hold_pid.onTarget():
                if not abs(self.chassis.distance) < 0.01:
                    self.chassis.zero_encoders()
                    return
                self.state = States.strafing
        if self.state == States.strafing:
            final_throttle = self.chassis.range_pid.maximumOutput
            # scale throttle smoothly between dead reckoning throttle and range max throttle
            throttle = (final_throttle - dr_throttle) * self.chassis.distance / self.strafe_distance + dr_throttle
            self.chassis.inputs[:] = (self.vx, self.vy, 0.0, throttle)
            if self.chassis.distance > self.strafe_distance:
                self.state = States.range_finding
                self.chassis.range_pid.reset()
                self.chassis.range_setpoint = 1.4  # m
                self.chassis.inputs[:] = (0.0, 0.0, 0.0, 0.0)
        if self.state == States.range_finding:
            if self.chassis.range_pid.onTarget():
                self.chassis.vision_pid.reset()
                self.state = States.goal_tracking
        if self.state == States.goal_tracking:
            # self.shooter.change_state(shooter.States.shooting)
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


class LowBarCentreTower(ObstacleHighGoal):
    MODE_NAME = "Low bar, CENTRE tower"
    DEFAULT = True

    def __init__(self):
        # Barker field: delta_x = 2.4, delta_y = -3.8
        super().__init__(1.2, -1.0, 0.0)


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
