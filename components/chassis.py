
import math

from wpilib import CANTalon

from .bno055 import BNO055
from .vision import Vision
from .bno055 import BNO055
from .range_finder import RangeFinder

class Chassis:
    length = 498.0  # mm
    width = 600.0  # mm

    motor_dist = math.sqrt((width / 2) ** 2 + (length / 2) ** 2)  # distance of motors from the center of the robot

    #                    x component                   y component
    vz_components = {'x': (width / 2) / motor_dist, 'y': (length / 2) / motor_dist}  # multiply both by vz and the

    # the number that you need to multiply the vz components by to get them in the appropriate directions
    #                   vx   vy
    module_params = {'a': {'args': {'drive':8, 'steer':10, 'absolute':True,
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':332,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},
                           'vz': {'x':-vz_components['x'], 'y': vz_components['y']}},
                     'b': {'args': {'drive':6, 'steer':7, 'absolute':True,
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':162,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},
                           'vz': {'x':-vz_components['x'], 'y':-vz_components['y']}},
                     'c': {'args': {'drive':3, 'steer':11, 'absolute':True,
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':320,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},
                           'vz': {'x': vz_components['x'], 'y':-vz_components['y']}},
                     'd': {'args': {'drive':4, 'steer':12, 'absolute':True,
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':606,
                                    'drive_encoder':True, 'reverse_drive_encoder':False},
                           'vz': {'x': vz_components['x'], 'y': vz_components['y']}}
                     }

    # Use the magic here!
    bno055 = BNO055
    vision = Vision
    range_finder = RangeFinder

    def __init__(self):
        super().__init__()

        #  A - D
        #  |   |
        #  B - C
        self._modules = {}
        for name, params in Chassis.module_params.items():
            self._modules[name] = SwerveModule(**(params['args']))
        self.field_oriented = False
        self.inputs = [0.0, 0.0, 0.0, None]
        self.vx = self.vy = self.vz = 0.0
        self.throttle = None
        self.track_vision = False
        self.range_setpoint = None
        import robot
        self.rescale_js = robot.rescale_js

    def toggle_field_oriented(self):
        self.field_oriented = not self.field_oriented

    def toggle_vision_tracking(self):
        self.track_vision = not self.track_vision

    def toggle_range_holding(self, setpoint):
        if self.range_setpoint == 0.0:
            self.range_setpoint = setpoint
        else:
            self.range_setpoint = 0.0

    def drive(self, vX, vY, vZ, throttle):
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

    def execute(self):
        if self.field_oriented and self.inputs[3] is not None:
            self.inputs[0:2] = field_orient(self.inputs[0], self.inputs[1], self.bno055.getHeading())
        # Are we holding a range
        if self.range_setpoint:
            self.field_oriented = False
            self.throttle = 1.0
            self.vx = self.rescale_js(self.range_finder - self.range_setpoint, rate=0.3)
        else:
            self.vx = self.inputs[0]
            self.throttle = self.inputs[3]
        # Are we strafing to get the vision target in the centre
        if self.track_vision:
            self.field_oriented = False
            self.throttle = 1.0
            vision_data = self.vision.get()
            if vision_data:  # Data is available and new
                self.vy = self.rescale_js(vision_data[0], rate=0.3)
        else:
            self.vy = self.inputs[1]
            self.throttle = self.inputs[3]
        # TODO - use the gyro to hold heading here
        self.vz = self.inputs[2]
        self.drive(self.vx, self.vy, self.vz, self.throttle)


class SwerveModule():
    def __init__(self, drive, steer,
                 absolute=True, reverse_drive=False,
                 reverse_steer=False, zero_reading=0,
                 drive_encoder=False, reverse_drive_encoder=False):
        # Initialise private motor controllers
        self._drive = CANTalon(drive)
        self.reverse_drive = reverse_drive
        self._steer = CANTalon(steer)
        self.drive_encoder = drive_encoder

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
            # Update the current setpoint to be the current position
            # Stops the unwind problem
            self._steer.set(self._steer.getSetpoint())
        else:
            self._steer.changeControlMode(CANTalon.ControlMode.Position)
            self._steer.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self._steer.setPID(6.0, 0.0, 0.0)  # PID values for rel
            self.counts_per_radian = 497.0 * (40.0 / 48.0) * 4.0 / (2.0 * math.pi)
            self._offset = 0

        if self.drive_encoder:
            self.drive_counts_per_rev = 80*6.67
            self.drive_counts_per_metre = self.drive_counts_per_rev*(math.pi*0.1016)
            self.drive_max_speed = 639
            self._drive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self.changeDriveControlMode(CANTalon.ControlMode.Speed)
            self._drive.reverseSensor(reverse_drive_encoder)
        else:
            self.drive_counts_per_rev = 0.0
            self.drive_max_speed = 1.0
            self.changeDriveControlMode(CANTalon.ControlMode.PercentVbus)

    def changeDriveControlMode(self, control_mode):
        if self._drive.getControlMode is not control_mode:
            if control_mode == CANTalon.ControlMode.Speed:
                self._drive.setPID(1.0, 0.0, 0.0, 1023.0/self.drive_max_speed)
            elif control_mode == CANTalon.ControlMode.Position:
                self._drive.setPID(0.1, 0.0, 0.0, 0.0)
            self._drive.changeControlMode(control_mode)

    def driveToPos(self, position, direction, reset_position):
        if self.drive_encoder:
            # position in meters
            self.changeDriveControlMode(CANTalon.ControlMode.Position)
            if self.reverse_drive:
                position = -position
            if reset_position:
                # re-zero the relative encoder
                self._drive.setSensorPosition(0.0)
            direction = constrain_angle(direction)
            current_heading = constrain_angle(self.direction)

            delta = min_angular_displacement(current_heading, direction)
            self._steer.set((self.direction+delta)*self.counts_per_radian+self._offset)

            if abs(self._steer.getClosedLoopError()*self.counts_per_radian+self._offset) < 0.1:
                # if we are within .1 of a radian of where we want to be
                self._drive.set(position*self.drive_counts_per_metre)

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
        if self.drive_encoder:
            self.changeDriveControlMode(CANTalon.ControlMode.Speed)
        else:
            self.changeDriveControlMode(CANTalon.ControlMode.PercentVbus)
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
                self._drive.set(speed*self.drive_max_speed)
            else:
                self._drive.set(-speed*self.drive_max_speed)
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

def field_orient(vx, vy, heading):
    oriented_vx = vx * math.cos(heading) + vy * math.sin(heading)
    oriented_vy = -vx * math.sin(heading) + vy * math.cos(heading)
    return oriented_vx, oriented_vy
