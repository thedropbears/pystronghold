
from wpilib import CANTalon, PowerDistributionPanel

from components import shooter

import logging

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

    pdp_channel = 15

    def __init__(self):
        self._speed = 0.0
        self.state = States.no_ball
        self.pdp = PowerDistributionPanel()
        self.current_counter = 0.0
        self.spool_up = 0.0
        self.changed_state = True

    def change_state(self, state):
        if state != self.state:
            self.state = state
            self.changed_state = True

    def toggle(self):
        if self.state == States.intaking_free:
            self.state = States.no_ball
        else:
            self.state = States.intaking_free
        self.changed_state = True

    def execute(self):
        current = self.pdp.getCurrent(self.pdp_channel)
        if current != 0.0:
            logging.getLogger("intake").info(self.pdp.getCurrent(self.pdp_channel))
            logging.getLogger("intake").info("State: " + str(self.state))
        if self.state == States.no_ball:
            self._speed = 0.0
        if self.state == States.intaking_free:
            self._speed = 1.0
            self.shooter.change_state(shooter.States.backdriving)
            self.spool_up += 1
            if current > 3.0 and self.spool_up > 20: # amps
                self.current_counter += 1
                if self.current_counter >= 3:
                    self.change_state(States.intaking_contact)
                    self.current_counter = 0
                    self.spool_up = 0
            elif current != 0:
                self.current_counter = 0
        if self.state == States.intaking_contact:
            self._speed = 1.0
            self.shooter.change_state(shooter.States.backdriving)
            if current < 3.0: # amps
                self.current_counter += 1
                if self.current_counter >= 2:
                    self.change_state(States.pinning)
                    self.current_counter = 0
            elif current != 0:
                self.current_counter = 0
        if self.state == States.pinning:
            self._speed = -0.4
            self.shooter.change_state(shooter.states.off)
            if current > 5.0:
                self.current_counter += 1
                if self.current_counter >= 2:
                    self.change_state(States.pinned)
                    self.current_counter = 0
            elif current != 0:
                self.current_counter = 0
        if self.state == States.pinned:
            self._speed = 0.0
        if self.state == States.fire:
            self._speed = 1.0
        if self.changed_state:
            self.intake_motor.set(-self._speed)
            self.changed_state = False
