
from wpilib import CANTalon, PowerDistributionPanel

from components import shooter

import csv

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
        self.log_queue = []
        self.log_current = True

    def toggle(self):
        self.log_queue = []
        if self.state == States.intaking_free:
            self.state = States.no_ball
        else:
            self.state = States.intaking_free

    def fire(self):
        self.state = States.fire

    def execute(self):
        # add next reading on right, will automatically pop on left
        self.current_deque.append(self.intake_motor.getOutputCurrent())
        maxlen = self.current_deque.maxlen
        current_avg = sum(self.current_deque) / maxlen
        current_rate = self.current_deque[maxlen-1]-self.current_deque[maxlen-2]
        if self.state != States.no_ball and self.state != States.pinned:
            self.log_queue.append(current_avg)

        if self.state == States.no_ball:
            self._speed = 0.0

        if self.state == States.intaking_free:
            self.shooter.change_state(shooter.States.off)
            self.intake_motor.setVoltageRampRate(6.0)  # V/s
            self._speed = 1.0
            if current_avg > 2.0 and current_rate >= 0.0:  # amps
                self.state = States.intaking_contact

        if self.state == States.intaking_contact:
            self.shooter.change_state(shooter.States.off)
            self._speed = 1.0
            if current_avg < 5 and current_rate <= 0.0:  # amps
                self.state = States.pinning

        if self.state == States.pinning:
            self._speed = -0.35
            self.shooter.change_state(shooter.States.backdriving)
            self.intake_motor.setVoltageRampRate(120.0)  # Max ramp rate
            if current_avg > 4 and current_rate >= 0.0:
                self.state = States.pinned

        if self.state == States.pinned:
            self.shooter.change_state(shooter.States.off)
            self._speed = 0.0
            if self.log_current and self.log_queue:
                csv_file = open("current_log.csv", "wb")
                writer = csv.writer(csv_file)
                writer.writerow(self.log_queue)
                self.log_queue = []

        if self.state == States.fire:
            self.intake_motor.setVoltageRampRate(240.0)  # Max ramp rate
            if abs(self.shooter.shooter_motor.getClosedLoopError())<= 0.02*(self.shooter.max_speed) and self.shooter._speed != 0.0:
                self._speed = 1.0
                logging.getLogger("intake").info("fire away")

        self.intake_motor.set(-self._speed)
