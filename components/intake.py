
from wpilib import CANTalon, PowerDistributionPanel

from components import shooter

import logging
from _collections import deque

class States:
    no_ball = 0
    intaking_free = 1
    intaking_contact = 2
    pinning = 4
    pinned = 5
    fire = 6

class Intake:
    intake_motor = CANTalon
    shooter = shooter.Shooter

    def __init__(self):
        self._speed = 0.0
        self.state = States.no_ball
        self.current_deque = deque([0.0] * 5, 5)  # Used to average currents over last n readings

    def toggle(self):
        if self.state == States.intaking_free:
            self.state = States.no_ball
        else:
            self.state = States.intaking_free

    def execute(self):
        # add next reading on right, will automatically pop on left
        self.current_deque.append(self.intake_motor.getOutputCurrent())
        maxlen = self.current_deque.maxlen
        current_avg = sum(self.current_deque) / maxlen
        current_rate = self.current_deque[maxlen-1]-self.current_deque[maxlen-2]

        if self.state == States.no_ball:
            self._speed = 0.0

        if self.state == States.intaking_free:
            self.intake_motor.setVoltageRampRate(6.0)  # V/s
            self._speed = 1.0
            self.shooter.change_state(shooter.States.backdriving)
            if current_avg > 3.0 and current_rate >= 0.0:  # amps
                self.state = States.intaking_contact

        if self.state == States.intaking_contact:
            self._speed = 1.0
            if current_avg < 3.0 and current_rate <= 0.0:  # amps
                self.state = States.pinning

        if self.state == States.pinning:
            self._speed = -0.4
            if current_avg > 5.0 and current_rate >= 0.0:
                self.state = States.pinned

        if self.state == States.pinned:
            self.shooter.change_state(shooter.States.off)
            self._speed = 0.0

        if self.state == States.fire:
            self.intake_motor.setVoltageRampRate(99999.0)  # Max ramp rate
            self._speed = 1.0

        self.intake_motor.set(-self._speed)
