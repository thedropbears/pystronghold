
from wpilib import CANTalon

from robot_map import RobotMap

import math

import logging

class Chassis():
    length = 498.0  # mm
    width = 600.0  # mm

    motor_dist = math.sqrt((width / 2) ** 2 + (length / 2) ** 2)  # distance of motors from the center of the robot

    #                    x component                   y component
    vz_components = {'x': (width / 2) / motor_dist, 'y': (length / 2) / motor_dist}  # multiply both by vz and the

    # the number that you need to multiply the vz components by to get them in the appropriate directions
    #                   vx   vy
    module_params = {'a': {'args': {'drive':8, 'steer':10, 'absolute':True,
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':187},
                           'vz': {'x':-vz_components['x'], 'y': vz_components['y']}},
                     'b': {'args': {'drive':6, 'steer':7, 'absolute':True,
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':246},
                           'vz': {'x':-vz_components['x'], 'y':-vz_components['y']}},
                     'c': {'args': {'drive':3, 'steer':4, 'absolute':True,
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':257},
                           'vz': {'x': vz_components['x'], 'y':-vz_components['y']}},
                     'd': {'args': {'drive':1, 'steer':12, 'absolute':True,
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':873},
                           'vz': {'x': vz_components['x'], 'y': vz_components['y']}}
                     }


    def __init__(self, robot):

        super().__init__()

        self.robot = robot
        #  A - D
        #  |   |
        #  B - C
        self._modules = {}
        for name, params in Chassis.module_params.items():
            self._modules[name] = SwerveModule(**(params['args']))

    def drive(self, vX, vY, vZ, throttle):
        if self.robot.field_oriented and throttle is not None:
            vX, vY = self.robot.oi.fieldOrient(vX, vY, self.robot.bno055.getHeading())
        motor_vectors = {}
        for name, params in Chassis.module_params.items():
            motor_vectors[name] = {'x': vX + vZ * params['vz']['x'],
                                   'y': vY + vZ * params['vz']['y']
                                   }
        # convert the vectors to polar coordinates
        polar_vectors = {}
        max_mag = 1.0
        for name, motor_vector in motor_vectors.items():
            polar_vectors[name] = {'dir': math.atan2(motor_vector['y'],
                                                     motor_vector['x']
                                                     ),
                                   'mag': math.sqrt(motor_vector['x'] ** 2
                                                    + motor_vector['y'] ** 2
                                                    )
                                   }
            if abs(polar_vectors[name]['mag']) > max_mag:
                max_mag = polar_vectors[name]['mag']

        for name in polar_vectors.keys():
            polar_vectors[name]['mag'] /= max_mag
            if throttle is None:
                polar_vectors[name]['mag'] = None
                continue
            polar_vectors[name]['mag'] *= throttle

        for name, polar_vector in polar_vectors.items():
            self._modules[name].steer(polar_vector['dir'], polar_vector['mag'])


class SwerveModule():
    def __init__(self, drive, steer,
                 absolute=True, reverse_drive=False,
                 reverse_steer=False, zero_reading=0):
        # Initialise private motor controllers
        self._drive = CANTalon(drive)
        self.reverse_drive = reverse_drive
        self._steer = CANTalon(steer)

        # Set up the motor controllers
        # Different depending on whether we are using absolute encoders or not
        if absolute:
            self.counts_per_radian = 1024.0 / (2.0 * math.pi)
            self._steer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder)
            self._steer.changeControlMode(CANTalon.ControlMode.Position)
            self._steer.reverseSensor(reverse_steer)
            self._steer.reverseOutput(not reverse_steer)
            # Read the current encoder position
            self._steer.setPID(20.0, 0.0, 0.0)  # PID values for abs
            self._offset = zero_reading - 256.0
            if reverse_steer:
                self._offset = -self._offset
        else:
            self._steer.changeControlMode(CANTalon.ControlMode.Position)
            self._steer.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self._steer.setPID(6.0, 0.0, 0.0)  # PID values for rel
            self.counts_per_radian = 497.0 * (40.0 / 48.0) * 4.0 / (2.0 * math.pi)
            self._offset = 0

    @property
    def direction(self):
        # Read the current direction from the controller setpoint
        setpoint = self._steer.getSetpoint()
        return float(setpoint - self._offset) / self.counts_per_radian

    @property
    def speed(self):
        # Read the current speed from the controller setpoint
        setpoint = self._drive.getSetpoint()
        return float(setpoint)

    def steer(self, direction, speed=None):
        # Set the speed and direction of the swerve module
        # Always choose the direction that minimises movement,
        # even if this means reversing the drive motor
        if speed is None:
            # Force the modules to the direction specified - don't
            # go to the closest one and reverse.
            delta = constrain_angle(direction - self.direction)  # rescale to +/-pi
            self._steer.set((self.direction + delta) * self.counts_per_radian + self._offset)
            self._drive.set(0.0)
            return

        if abs(speed) > 0.05:
            direction = constrain_angle(direction)  # rescale to +/-pi
            current_heading = constrain_angle(self.direction)

            delta = min_angular_displacement(current_heading, direction)

            if self.reverse_drive:
                speed = -speed
            if abs(constrain_angle(self.direction) - direction) < math.pi / 6.0:
                self._drive.set(speed)
            else:
                self._drive.set(-speed)
            self._steer.set((self.direction + delta) * self.counts_per_radian + self._offset)
        else:
            self._drive.set(0.0)

def constrain_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def min_angular_displacement(current, target):
    target = constrain_angle(target)
    opp_target = constrain_angle(target + math.pi)
    current = constrain_angle(current)
    diff = constrain_angle(target - current)
    opp_diff = constrain_angle(opp_target - current)

    if abs(diff) < abs(opp_diff):
        return diff
    return opp_diff


