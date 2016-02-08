from wpilib import CANTalon

import logging

class Shooter:
#closed-loop controls for shooting mechanism

    shooter_motor = CANTalon
    shoot_encoder_cpr = 4096.0
    max_speed = 37000.0
    shoot_percentage = 0.88

    def __init__(self):
        self._speed = 0.0
        self._changed_state = False
        self.initialised = False

    def set_speed(self, speed):
        self._speed = speed * Shooter.max_speed
        self._changed_state = True

    def stop(self):
        self._speed = 0.0
        self._changed_state = True

    def toggle(self, speed=shoot_percentage):
        if self._speed == 0.0:
            self.set_speed(speed)
        else:
            self.stop()

    def execute(self):
        if not self.initialised:
            self.shooter_motor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self.shooter_motor.changeControlMode(CANTalon.ControlMode.Speed)
            self.shooter_motor.reverseSensor(True)
            self.shooter_motor.setPID(0.075, 0.00075, 0, 1023.0 / Shooter.max_speed, izone=3000)
            #self.shooter_motor.setPID(0.075, 0.000, 0, 1023.0 / Shooter.max_speed)
            self.initialised = True

        if self._changed_state:
            self.shooter_motor.set(-self._speed)
            self._changed_state = False
