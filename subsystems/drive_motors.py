
from robot_map import RobotMap

from wpilib import CANTalon

class DriveMotors():
    def __init__(self, robot):
        self.robot = robot
        self.motor_a = CANTalon(RobotMap.drive_motors_motor_a_id)
        self.motor_b = CANTalon(RobotMap.drive_motors_motor_b_id)

    def drive(self, speed):
        self.motor_a.set(speed)
        self.motor_b.set(speed)
