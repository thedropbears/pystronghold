
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

        mA_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_a_vz_scaling[0], vX+vZ*RobotMap.vz_components[1]*RobotMap.motor_a_vz_scaling[1]]
        mB_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_b_vz_scaling[0], vX+vZ*RobotMap.vz_components[1]*RobotMap.motor_b_vz_scaling[1]]
        mC_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_c_vz_scaling[0], vX+vZ*RobotMap.vz_components[1]*RobotMap.motor_c_vz_scaling[1]]
        mD_vector = [vX+vZ*RobotMap.vz_components[0]*RobotMap.motor_d_vz_scaling[0], vX+vZ*RobotMap.vz_components[1]*RobotMap.motor_d_vz_scaling[1]]

        vectors = [mA_vector, mB_vector, mC_vector, mD_vector]

        # convert the vectors to pollar coordinates
        polar = []
        max_magnitutde = 1.0
        for motor_vector in vectors:
            #                      direction                                                             magnitude
            polar_vector = [math.atan2(motor_vector[1], motor_vector[0]), math.sqrt(motor_vector[0]**2+motor_vector[1]**2)]
            if abs(polar_vector[1]) > max_magnitude:
                max_magnitude = polar_vector[1]
            polar.apppend(polar_vector)

        for polar_vector in polar:
            polar_vector[1]/= max_magnitude

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
            pass
        else:
            self._steer.changeControlMode(CANTalon.controlMode.Position)

        # Private members to store the setpoints
        self._speed = 0.0
        self._direction = 0.0
        # Always in radians. Right hand rule applies - Z is up!
        # Rescale values to the range [0, 2*pi) m8 its tau not 2pi

    def steer(self, direction, speed = 0):
        # Set the speed and direction of the swerve module
        # Always choose the direction that minimises movement,
        # even if this means reversing the drive motor
        self._direction = self.wrapRadians(direction)
        self._speed = speed

        ticks_to_set_to = 0.0 # the position that we want to set the steer motor to

        if not self.absoluteEncoder:
            direction_ticks = self._direction/TAU * RobotMap.module_rotation_counts_per_revolution
            opposite_direction_ticks = self.wrapRadians(self._direction-TAU/2)*RobotMap.module_rotation_counts_per_revolution
            current_direction = self.wrapRadians(self._steer.get() / RobotMap.module_rotation_counts_per_revolution)
            opposite_current_direction = self.wrapRadians(current_direction+TAU/2)
            if not (current_direction - RobotMap.module_angular_tol < current_direction < current_direction + RobotMap.module_angular_tol or
                    opposite_current_direction - RobotMap.module_angular_tol < opposite_current_direction < opposite_current_direction + RobotMap.module_angular_tol):
                # we need to rotate to the correct direction

                # first, find out which way we need to rotate to
                if isCloser(current_direction, opposite_current_direction, self._direction):
                    # if the current direction is closer than opposite it to the desired direction
                    ticks_to_set_to = direction_ticks
                else:
                    ticks_to_set_to = oppostite_direction_ticks

                # now we need to figure out where the closest amount of ticks are that we can use (e.g. if the module has rotated several times over,
                # then we want our setpoint to be an equivalent number in the hundreds rather than somehing in the first rotation
            elif current_direction - RobotMap.module_angular_tol < current_direction < current_direction + RobotMap.module_angular_tol:
                self._drive.set(self._speed)
            else:
                self._drive.set(self._speed*-1)

    def isCloser(first, second, radians):
        pass

    def getSpeed(self):
        return self._speed

    def getDirection(self):
        return self._direction

    def wrapRadians(self, radians):
        if radians >= 0.0:
            return radians%TAU
        return -radians%TAU
