
from wpilib import CANTalon

class Intake:
    intake_motor = CANTalon

    def __init__(self):
        self.running = False

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def toggle(self):
        self.running = not self.running

    def execute(self):
        if self.running:
            self.intake_motor.set(1.0)
        else:
            self.intake_motor.set(0.0)
