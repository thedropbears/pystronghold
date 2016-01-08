
from wpilib.command import Subsystem

from wpilib import CANTalon

from robot_map import RobotMap

from commands.omni_drive import OmniDrive

import math

TAU = 2*math.pi

class Chassis(Subsystem):

    def __init__(self, robot):

        super().__init__()

        self.robot = robot
        # we want to create four swervemodules here
        # the numbers here need to be replaced with constants from robotmap
        #  A - D
        #  |   |
        #  B - C
        self._modules = [SwerveModule(RobotMap.module_a_move_motor_id , RobotMap.module_a_rotation_motor_id ),
                SwerveModule(RobotMap.module_b_move_motor_id, RobotMap.module_b_rotation_motor_id),
                SwerveModule(RobotMap.module_c_move_motor_id, RobotMap.module_c_rotation_motor_id),
                SwerveModule(RobotMap.module_d_move_motor_id, RobotMap.module_d_rotation_motor_id)]


    #Put methods for controlling this subsystem here.
    # Call these from Commands.

    def initDefaultCommand(self):
        #Set the default command for a subsystem here.
        self.setDefaultCommand(OmniDrive(self.robot))

    def drive(self, vX, vY, vZ, throttle):

        mA_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_a_vz_scaling[0],
                vY+vZ*RobotMap.vz_components[1]*RobotMap.motor_a_vz_scaling[1]]
        mB_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_b_vz_scaling[0],
                vY+vZ*RobotMap.vz_components[1]*RobotMap.motor_b_vz_scaling[1]]
        mC_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_c_vz_scaling[0],
                vY+vZ*RobotMap.vz_components[1]*RobotMap.motor_c_vz_scaling[1]]
        mD_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_d_vz_scaling[0],
                vY+vZ*RobotMap.vz_components[1]*RobotMap.motor_d_vz_scaling[1]]

        vectors = [mA_vector, mB_vector, mC_vector, mD_vector]

        # convert the vectors to pollar coordinates
        polar = []
        max_mag= 1.0
        for motor_vector in vectors:
            #                      direction                                                             magnitude
            polar_vector = [math.atan2(motor_vector[1], motor_vector[0]), math.sqrt(motor_vector[0]**2+motor_vector[1]**2)]
            if abs(polar_vector[1]) > max_mag:
                max_mag= polar_vector[1]
            polar.append(polar_vector)

        for polar_vector in polar:
            polar_vector[1]/= max_mag

        for module, polar_vector in zip(self._modules, polar):
            module.steer(polar_vector[0], polar_vector[1])



class SwerveModule():
    def __init__(self, driveCanTalonId, steerCanTalonId, absoluteEncoder = True):
        # Initialise private motor controllers
        self._drive = CANTalon(driveCanTalonId)
        self._steer = CANTalon(steerCanTalonId)
        self.absoluteEncoder = absoluteEncoder
        # Set up the motor controllers
        # Different depending on whether we are using absolute encoders or not
        if absoluteEncoder:
            self._steer.changeControlMode(CANTalon.ControlMode.Position)
            self._steer.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder)
        else:
            self._steer.changeControlMode(CANTalon.ControlMode.Position)
            self._steer.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)

        # Private members to store the setpoints
        self._speed = 0.0
        self._direction = 0.0
        self._opposite_direction = TAU/2
        # Always in radians. Right hand rule applies - Z is up!
        # Rescale values to the range [0, 2*pi) m8 its tau not 2pi

    def steer(self, direction, speed = 0):
        # Set the speed and direction of the swerve module
        # Always choose the direction that minimises movement,
        # even if this means reversing the drive motor
        direction = math.atan2(math.sin(direction), math.cos(direction)) # rescale to +/-pi
        opposite_direction = math.atan2(math.sin(direction+TAU/2.0), math.cos(direction+TAU/2.0)) # rescale to +/-pi
        heading = math.atan2(math.sin(self._direction), math.cos(self._direction))

        delta = self.angularDisplacement(direction, opposite_direction, heading)

        self._direction += delta
        if abs(math.atan2(math.sin(self._direction), math.cos(self._direction))-direction)<math.pi/6.0:
            self._drive.set(speed)
            self._speed = speed
        else:
            self._drive.set(-speed)
            self._speed = -speed
        if self.absoluteEncoder:
            self._steer.set(self._direction*RobotMap.module_rotation_volts_per_revolution)
        else:
            self._steer.set(self._direction*RobotMap.module_rotation_counts_per_revolution)


    def angularDisplacement(self, first, second, radians):

        first_diff = first-radians
        second_diff = second-radians

        first_diff = math.atan2(math.sin(first_diff), math.cos(first_diff))
        second_diff = math.atan2(math.sin(second_diff), math.cos(second_diff))

        if abs(first_diff) < abs(second_diff):
            return first_diff
        return second_diff


    def getSpeed(self):
        return self._speed

    def getDirection(self):
        return self._direction

    def wrapRadians(self, radians):
        if radians >= 0.0:
            return radians%TAU
        return TAU+(radians%TAU)
