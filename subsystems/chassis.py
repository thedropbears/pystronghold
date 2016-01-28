
from wpilib import CANTalon

from robot_map import RobotMap

import math

class Chassis():
    # (drive_id, steer_id)
    module_motors = {'a': {'drive':8, 'steer':10, 'absolute':True, 'reverse_drive':False, 'reverse_steer':True, 'zero_reading':256},
                     'b': {'drive':6, 'steer':7, 'absolute':True, 'reverse_drive':True, 'reverse_steer':True, 'zero_reading':256},
                     'c': {'drive':3, 'steer':4, 'absolute':True, 'reverse_drive':True, 'reverse_steer':True, 'zero_reading':256},
                     'd': {'drive':1, 'steer':12, 'absolute':True, 'reverse_drive':False, 'reverse_steer':True, 'zero_reading':256}
                     }

    length = 498.0  # mm
    width = 600.0  # mm

    motor_dist = math.sqrt((width / 2) ** 2 + (length / 2) ** 2)  # distance of motors from the center of the robot

    #                    x component                   y component
    vz_components = ((width / 2) / motor_dist, (length / 2) / motor_dist)  # multiply both by vz and the

    # the number that you need to multiply the vz components by to get them in the appropriate directions
    #                   vx   vy
    motor_vz_scaling = [(-vz_components[0], vz_components[1]),
                        (-vz_components[0], -vz_components[1]),
                        (vz_components[0], -vz_components[1]),
                        (vz_components[0], vz_components[1])]
    
    
    def __init__(self, robot):

        super().__init__()

        self.robot = robot
        # we want to create four swervemodules here
        # the numbers here need to be replaced with constants from robotmap
        #  A - D
        #  |   |
        #  B - C
        self._modules = [SwerveModule(**module_motor_params)
                         for _, module_motor_params in Chassis.module_motors.items()
                         ]

    def drive(self, vX, vY, vZ, throttle):
        if self.robot.field_oriented:
            vX, vY = self.robot.oi.fieldOrient(vX, vY, self.robot.bno055.getHeading())
        motor_vectors = []
        for scaling in Chassis.motor_vz_scaling:
            motor_vectors.append((vX + vZ * scaling[0], vY + vZ * scaling[1]))
        # convert the vectors to polar coordinates
        polar = []
        max_mag = 1.0
        for motor_vector in motor_vectors:
            #                      direction                                                             magnitude
            polar_vector = [math.atan2(motor_vector[1], motor_vector[0]), math.sqrt(motor_vector[0] ** 2 + motor_vector[1] ** 2)]
            if abs(polar_vector[1]) > max_mag:
                max_mag = polar_vector[1]
            polar.append(polar_vector)

        for polar_vector in polar:
            polar_vector[1] /= max_mag
            polar_vector[1] *= throttle

        for module, polar_vector in zip(self._modules, polar):
            module.steer(polar_vector[0], polar_vector[1])


class SwerveModule():
    def __init__(self, drive, steer,
                 absolute=True, reverse_drive=False,
                 reverse_steer=False, zero_reading=0):
        # Initialise private motor controllers
        self._drive = CANTalon(drive)
        self.reverse_drive = reverse_drive
        self._steer = CANTalon(steer)
        # Private members to store the setpoints
        self._speed = 0.0
        self._direction = 0.0
        # Always in radians. Right hand rule applies - Z is up!

        # Set up the motor controllers
        # Different depending on whether we are using absolute encoders or not
        if absolute:
            self.counts_per_radian = 1024.0 / (2.0 * math.pi)
            self._steer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder)
            self._steer.changeControlMode(CANTalon.ControlMode.Position)
            self._steer.reverseSensor(reverse_steer)
            self._steer.reverseOutput(not reverse_steer)
            # Read the current encoder position
            self._steer.setPID(6.0, 0.0, 0.0)  # PID values for abs
            self._offset = zero_reading - 256
            self._direction = float(self._steer.get() - self._offset) / self.counts_per_radian
            import logging
            logging.getLogger("Swerve").info("CAN id: %i Counts: %i Direction: %f"
                                             % (steer, self._steer.get() - self._offset, self._direction))
        else:
            self._steer.changeControlMode(CANTalon.ControlMode.Position)
            self._steer.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self._steer.setPID(6.0, 0.0, 0.0)  # PID values for rel
            self._steer.setPosition(0.0)
            self.counts_per_radian = 497.0 * (40.0 / 48.0) * 4.0 / (2.0 * math.pi)
            self._offset = 0


    def steer(self, direction, speed=None):
        # Set the speed and direction of the swerve module
        # Always choose the direction that minimises movement,
        # even if this means reversing the drive motor
        if speed is None:
            # Force the modules to the direction specified - don't
            # go to the closest one and reverse.
            direction = constrain_angle(direction)  # rescale to +/-pi
            current_heading = constrain_angle(self._direction)
            delta = direction - current_heading
            if delta > math.pi:
                delta -= 2.0 * math.pi
            elif delta < -math.pi:
                delta += 2.0 * math.pi
            self._direction += delta
            self._steer.set(self._direction * self.counts_per_radian + self._offset)
            self._drive.set(0.0)
            self._speed = 0.0
            return
            
        if speed != 0.0:
            direction = constrain_angle(direction)  # rescale to +/-pi
            current_heading = constrain_angle(self._direction)
    
            delta = min_angular_displacement(current_heading, direction)
    
            self._direction += delta
            if self.reverse_drive:
                speed = -speed
            if abs(constrain_angle(self._direction) - direction) < math.pi / 6.0:
                self._drive.set(speed)
                self._speed = speed
            else:
                self._drive.set(-speed)
                self._speed = -speed
            self._steer.set(self._direction * self.counts_per_radian + self._offset)

    def getSpeed(self):
        return self._speed

    def getDirection(self):
        return self._direction

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


