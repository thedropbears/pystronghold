
from wpilib import CANTalon, PowerDistributionPanel

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

    pdp_channel = 15

    def __init__(self):
        self._speed = 0.0
        self.state = States.no_ball
        self.pdp = PowerDistributionPanel()
        self.current_counter = 0.0
        self.spool_up = 0.0

    def set_speed(self, speed):
        self._speed = speed

    def stop(self):
        self._speed = 0.0

    def toggle(self, speed=1.0):
        if self.state == States.intaking_free:
            self.state = States.no_ball
        else:
            self.state = States.intaking_free

    def execute(self):
        current = self.pdp.getCurrent(self.pdp_channel)
        if current != 0.0:
            logging.getLogger("intake").info(self.pdp.getCurrent(self.pdp_channel))
            logging.getLogger("intake").info("State: " + str(self.state))
        if self.state == States.no_ball:
            self.intake_motor.set(0.0)
        if self.state == States.intaking_free:
            self.intake_motor.set(-1.0)
            self.spool_up += 1
            if current > 3.0 and self.spool_up > 20: # amps
                self.current_counter += 1
                if self.current_counter >= 3:
                    self.state = States.intaking_contact
                    self.current_counter = 0
                    self.spool_up = 0
            elif current != 0:
                self.current_counter = 0
        if self.state == States.intaking_contact:
            self.intake_motor.set(-1.0)
            if current < 3.0: # amps
                self.current_counter += 1
                if self.current_counter >= 2:
                    self.state = States.pinning
                    self.current_counter = 0
            elif current != 0:
                self.current_counter = 0
        if self.state == States.pinning:
            self.intake_motor.set(0.4)
            if current > 5.0:
                self.current_counter += 1
                if self.current_counter >= 2:
                    self.state = States.pinned
                    self.current_counter = 0
            elif current != 0:
                self.current_counter = 0
        if self.state == States.pinned:
            self.intake_motor.set(0.0)
        if self.state == States.fire:
            self.intake_motor.set(-1.0)
