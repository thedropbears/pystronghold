
import math

from wpilib import CANTalon, PIDController
from wpilib.interfaces import PIDOutput, PIDSource

from .bno055 import BNO055
from .vision import Vision
from .range_finder import RangeFinder


class BlankPIDOutput(PIDOutput):
    def __init__(self):
        self.output = 0.0

    def pidWrite(self, output):
        self.output = output


class Chassis:
    correct_range = 1.65 # m

    length = 498.0  # mm
    width = 600.0  # mm

    vision_scale_factor = 0.3  # units of m/(vision unit)
    distance_pid_abs_error = 0.05  # metres

    motor_dist = math.sqrt((width / 2) ** 2 + (length / 2) ** 2)  # distance of motors from the center of the robot

    #                    x component                   y component
    vz_components = {'x': (width / 2) / motor_dist, 'y': (length / 2) / motor_dist}  # multiply both by vz and the

    # the number that you need to multiply the vz components by to get them in the appropriate directions
    #                   vx   vy
    """module_params = {'a': {'args': {'drive':13, 'steer':14, 'absolute':True,
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':30,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},
                           'vz': {'x':-vz_components['x'], 'y': vz_components['y']}},
                     'b': {'args': {'drive':8, 'steer':9, 'absolute':True,
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':109,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},
                           'vz': {'x':-vz_components['x'], 'y':-vz_components['y']}},
                     'c': {'args': {'drive':2, 'steer':4, 'absolute':True,
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':536,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},
                           'vz': {'x': vz_components['x'], 'y':-vz_components['y']}},
                     'd': {'args': {'drive':3, 'steer':6, 'absolute':True,
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':389,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},
                           'vz': {'x': vz_components['x'], 'y': vz_components['y']}}
                     }"""
    module_params = {'a': {'args': {'drive':13, 'steer':11, 'absolute':False,                                     
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':30,      
                                    'drive_encoder':True, 'reverse_drive_encoder':True},                 
                           'vz': {'x':-vz_components['x'], 'y': vz_components['y']}},               
                     'b': {'args': {'drive':8, 'steer':9, 'absolute':False,                         
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':109,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},             
                           'vz': {'x':-vz_components['x'], 'y':-vz_components['y']}},                
                     'c': {'args': {'drive':2, 'steer':4, 'absolute':False,                          
                                    'reverse_drive':False, 'reverse_steer':True, 'zero_reading':536,
                                    'drive_encoder':True, 'reverse_drive_encoder':True},             
                           'vz': {'x': vz_components['x'], 'y':-vz_components['y']}},                
                     'd': {'args': {'drive':3, 'steer':6, 'absolute':False,                          
                                    'reverse_drive':True, 'reverse_steer':True, 'zero_reading':389, 
                                    'drive_encoder':True, 'reverse_drive_encoder':True},             
                           'vz': {'x': vz_components['x'], 'y': vz_components['y']}}                 
                     }
    # Use the magic here!
    bno055 = BNO055
    vision = Vision
    range_finder = RangeFinder
    heading_hold_pid_output = BlankPIDOutput
    heading_hold_pid = PIDController

    def __init__(self):
        super().__init__()

        #  A - D
        #  |   |
        #  B - C
        self._modules = {}
        for name, params in Chassis.module_params.items():
            self._modules[name] = SwerveModule(**(params['args']))
            self._modules[name]._drive.setVoltageRampRate(50.0)
        self.field_oriented = True
        self.inputs = [0.0, 0.0, 0.0, 0.0]
        self.vx = self.vy = self.vz = 0.0
        self.track_vision = False
        self.range_setpoint = None
        self.heading_hold = True
        self.lock_wheels = False
        self.momentum = False
        import robot
        self.rescale_js = robot.rescale_js

        self.distance_pid_heading = 0.0  # Relative to field
        self.distance_pid_output = BlankPIDOutput()
        # TODO tune the distance PID values
        self.distance_pid = PIDController(1.0, 0.02, 0.0,
                                          self, self.distance_pid_output)
        self.distance_pid.setAbsoluteTolerance(self.distance_pid_abs_error)
        self.distance_pid.setToleranceBuffer(5)
        self.distance_pid.setContinuous(False)
        self.distance_pid.setInputRange(-5.0, 5.0)
        self.distance_pid.setOutputRange(-0.4, 0.4)
        self.distance_pid.setSetpoint(0.0)
        self.reset_distance_pid = False
        self.pid_counter = 0

    def on_enable(self):
        self.bno055.resetHeading()
        self.heading_hold = True
        self.field_oriented = True
        self.heading_hold_pid.setSetpoint(self.bno055.getAngle())
        self.heading_hold_pid.reset()
        # Update the current module steer setpoint to be the current position
        # Stops the unwind problem
        for module in self._modules.values():
            module._steer.set(module._steer.getPosition())

    def onTarget(self):
        for module in self._modules.values():
            if not abs(module._steer.getError()) < 50:
                return False
        return True

    def toggle_field_oriented(self):
        self.field_oriented = not self.field_oriented

    def toggle_vision_tracking(self):
        self.track_vision = not self.track_vision
        if self.track_vision:
            self.zero_encoders()
            self.distance_pid.setSetpoint(0.0)
            self.distance_pid.enable()

    def toggle_range_holding(self, setpoint=1.65):
        if not self.range_setpoint:
            self.range_setpoint = setpoint
            self.zero_encoders()
            self.distance_pid.setSetpoint(0.0)
            self.distance_pid.enable()
        else:
            self.range_setpoint = 0.0

    def zero_encoders(self):
        for module in self._modules.values():
            module.zero_distance()

    def field_displace(self, x, y):
        '''Use the distance PID to displace the robot by x,y
        in field reference frame.'''
        d = math.sqrt((x ** 2 + y ** 2))
        fx, fy = field_orient(x, y, self.bno055.getHeading())
        self.distance_pid_heading = math.atan2(fy, fx)
        self.distance_pid.disable()
        self.zero_encoders()
        self.distance_pid.setSetpoint(d)
        self.distance_pid.reset()
        self.distance_pid.enable()

    def pidGet(self):
        return self.distance

    def getPIDSourceType(self):
        return PIDSource.PIDSourceType.kDisplacement

    @property
    def distance(self):
        distances = 0.0
        for module in self._modules.values():
            distances += abs(module.distance) / module.drive_counts_per_metre
        return distances / 4.0

    def drive(self, vX, vY, vZ, absolute=False):
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
            if absolute:
                polar_vectors[name]['mag'] = None
                continue

        for name, polar_vector in polar_vectors.items():
            self._modules[name].steer(polar_vector['dir'], polar_vector['mag'])

    def execute(self):
        if self.field_oriented and self.inputs[3] is not None:
            self.inputs[0:2] = field_orient(self.inputs[0], self.inputs[1], self.bno055.getHeading())

        # Are we in setpoint displacement mode?
        if self.distance_pid.isEnable():
            if self.distance_pid.onTarget():
                if self.pid_counter > 10:
                    self.reset_distance_pid = False
                    # Let's see if we need to move further
                    x = y = 0.0
                    if self.range_setpoint and not self.on_range_target():
                        x = self.range_finder.pidGet() - self.range_setpoint
                        if x > 0.5:
                            x = 0.5
                        elif x < -0.5:
                            x = -0.5
                    if self.track_vision and not self.on_vision_target():
                        y = self.vision.pidGet() * self.vision_scale_factor
                        if y > self.vision_scale_factor:
                            y = self.vision_scale_factor
                        elif y < -self.vision_scale_factor:
                            y = -self.vision_scale_factor
                    self.distance_pid.disable()
                    self.zero_encoders()
                    self.distance_pid_heading = constrain_angle(math.atan2(y, x)+self.bno055.getAngle())
                    self.distance_pid.setSetpoint(math.sqrt(x**2+y**2))
                    self.distance_pid.reset()
                    self.distance_pid.enable()
                    self.pid_counter = 0
                else:
                    self.pid_counter += 1

            # Keep driving
            self.vx = math.cos(self.distance_pid_heading) * self.distance_pid_output.output
            self.vy = math.sin(self.distance_pid_heading) * self.distance_pid_output.output
        else:
            self.vx = self.inputs[0] * self.inputs[3]  # multiply by throttle
            self.vy = self.inputs[1] * self.inputs[3]  # multiply by throttle

        if self.heading_hold:
            if self.momentum and abs(self.bno055.getHeadingRate()) < 0.005:
                self.momentum = False

            if self.inputs[2] != 0.0:
                self.momentum = True

            if not self.momentum:
                self.heading_hold_pid.enable()
                self.vz = self.heading_hold_pid_output.output
            else:
                self.heading_hold_pid.setSetpoint(self.bno055.getAngle())
                self.vz = self.inputs[2] * self.inputs[3]  # multiply by throttle

        if self.lock_wheels:
            for _, params, module in zip(Chassis.module_params.items(),
                                         self._modules):
                direction = constrain_angle(math.atan2(params['vz']['y'],
                                                       params['vz']['x']) +
                                            math.pi / 2.0)
                module.steer(direction, 0.0)
        else:
            self.drive(self.vx, self.vy, self.vz)

    def toggle_heading_hold(self):
        self.heading_hold = not self.heading_hold

    def set_heading_setpoint(self, setpoint):
        self.heading_hold_pid.setSetpoint(constrain_angle(setpoint))

    def on_range_target(self):
        return abs(self.range_finder.pidGet() - self.range_setpoint) < self.distance_pid_abs_error * 2.0

    def on_vision_target(self):
        return (self.vision.no_vision_counter == 0.0 and
                abs(self.vision.pidGet() * self.vision_scale_factor) < self.distance_pid_abs_error * 2.0)


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
        self._distance_offset = 0  # Offset the drive distance counts

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
            self.counts_per_radian = (497.0 * (40.0 / 48.0) * 4.0 /
                                      (2.0 * math.pi))
            self._offset = self.counts_per_radian*2.0*math.pi/4.0
            self._steer.setPosition(0.0)

        if self.drive_encoder:
            self.drive_counts_per_rev = 80*6.67
            self.drive_counts_per_metre = (self.drive_counts_per_rev /
                                           (math.pi * 0.1016))
            self.drive_max_speed = 570
            self._drive.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self.changeDriveControlMode(CANTalon.ControlMode.Speed)
            self._drive.reverseSensor(reverse_drive_encoder)
        else:
            self.drive_counts_per_rev = 0.0
            self.drive_max_speed = 1.0
            self.changeDriveControlMode(CANTalon.ControlMode.PercentVbus)
        self._drive.setVoltageRampRate(150.0)

    def changeDriveControlMode(self, control_mode):
        if self._drive.getControlMode is not control_mode:
            if control_mode == CANTalon.ControlMode.Speed:
                self._drive.setPID(1.0, 0.00, 0.0, 1023.0 / self.drive_max_speed)
            elif control_mode == CANTalon.ControlMode.Position:
                self._drive.setPID(0.1, 0.0, 0.0, 0.0)
            self._drive.changeControlMode(control_mode)

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

    @property
    def distance(self):
        # Read the current position from the encoder and remove the offset
        return self._drive.getEncPosition() - self._distance_offset

    def zero_distance(self):
        self._distance_offset = self._drive.getEncPosition()

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
            self._steer.set((self.direction + delta) *
                            self.counts_per_radian + self._offset)
            self._drive.set(0.0)
            return

        if abs(speed) > 0.05:
            direction = constrain_angle(direction)  # rescale to +/-pi
            current_heading = constrain_angle(self.direction)

            delta = min_angular_displacement(current_heading, direction)

            if self.reverse_drive:
                speed = -speed
            if abs(constrain_angle(self.direction - direction)) < math.pi / 6.0:
                self._drive.set(speed*self.drive_max_speed)
            else:
                self._drive.set(-speed*self.drive_max_speed)
            self._steer.set((self.direction + delta) *
                            self.counts_per_radian + self._offset)
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
