
from wpilib import CANTalon

class Defeater:

    defeater_motor = CANTalon

    def up(self):
        self.defeater_motor.set(0.5)

    def down(self):
        self.defeater_motor.set(-0.5)

    def execute(self):
        current = self.defeater_motor.getOutputCurrent()

        if current > 10.0:
            self.defeater_motor.set(0.0)



