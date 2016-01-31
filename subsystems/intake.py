
from wpilib import CANTalon

from robot_map import RobotMap

class Intake():
    def __init__(self):
        self.intake_motor = CANTalon(2)

    def start(self):
        self.intake_motor.set(1.0)

    def stop(self):
        self.intake_motor.set(0.0)
