from wpilib import CANTalon

from components.chassis import Chassis

import logging

class States:
    off = 0
    shooting = 1
    backdriving = 2

class Shooter:
#closed-loop controls for shooting mechanism

    shooter_motor = CANTalon
    chassis = Chassis
    shoot_encoder_cpr = 4096.0
    max_speed = 36000.0
    #shoot_percentage = 0.99
    shoot_percentage = 0.67

    def __init__(self):
        self._changed_state = True
        self.initialised = False
        self.state = States.off
        self._speed = 0.0

    def start_shoot(self):
        self.state = States.shooting
        self._changed_state = True

    def stop(self):
        self.state = States.off
        self._changed_state = True

    def backdrive(self):
        self.change_state(States.backdriving)

    def toggle(self):
        if self.state == States.off:
            self.start_shoot()
        elif self.state == States.shooting:
            self.stop()

    def change_state(self, state):
        if state != self.state:
            self.state = state
            self._changed_state = True

    def on_enabled(self):
        self.stop()
        self._speed = 0.0

    def on_disabled(self):
        self.state = States.off
        self._changed_state = True

    def execute(self):
        if not self.initialised:
            self.shooter_motor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self.shooter_motor.changeControlMode(CANTalon.ControlMode.Speed)
            self.shooter_motor.reverseSensor(True)
            self.shooter_motor.setPID(0.075, 0.00075, 0, 1023.0 / Shooter.max_speed, izone=3000)
            self.initialised = True

        if self._changed_state:
            if self.state == States.shooting:
                self._speed = -self.shoot_percentage*Shooter.max_speed
            elif self.state == States.off:
                self._speed = 0.0
            elif self.state == States.backdriving:
                self._speed = 0.01*Shooter.max_speed
            self._changed_state = False
            self.shooter_motor.set(self._speed)
