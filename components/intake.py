
from wpilib import CANTalon

class Intake:
    intake_motor = CANTalon

    def __init__(self):
        self._speed = 0.0

    def set_speed(self, speed):
        self._speed = speed

    def stop(self):
        self._speed = 0.0

    def toggle(self, speed=1.0):
        if self._speed == 0.0:
            self.set_speed(speed)
        else:
            self.set_speed(0.0)

    def execute(self):
        self.intake_motor.set(self._speed)
