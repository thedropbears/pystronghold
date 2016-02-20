
from wpilib import CANTalon, PowerDistributionPanel

from components import shooter

import csv, time

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
        self.current_deque = deque([0.0] * 3, 3)  # Used to average currents over last n readings
        self.log_queue = []
        self.intake_time = 0.0

    def toggle(self):
        if self.state != States.no_ball:
            self.state = States.no_ball
            self.log_current()
        else:
            self.state = States.intaking_free

    def fire(self):
        self.state = States.fire

    def log_current(self):
        csv_file = open("/tmp/current_log.csv", "a")
        csv_file.write(str(self.log_queue).strip('[]').replace(' ', '')+"\n")
        csv_file.close()
        self.log_queue = []

    def execute(self):
        # add next reading on right, will automatically pop on left
        maxlen = self.current_deque.maxlen
        prev_current_avg = sum(self.current_deque)/maxlen
        self.current_deque.append(self.intake_motor.getOutputCurrent())
        current_avg = sum(self.current_deque) / maxlen
        current_rate = current_avg - prev_current_avg#self.current_deque[maxlen-1]-self.current_deque[maxlen-2]
        if self.state != States.no_ball and self.state != States.pinned:
            self.log_queue.append(self.current_deque[maxlen-1])

        if self.state == States.no_ball:
            self._speed = 0.0

        if self.state == States.intaking_free:
            self.shooter.change_state(shooter.States.off)
            self.intake_motor.setVoltageRampRate(6.0)  # V/s
            self._speed = 1.0
            if current_avg > 4.0:# and current_rate >= 0.0:  # amps
                self.state = States.intaking_contact
                self.intake_time = time.time()

        if self.state == States.intaking_contact:
            """#self.shooter.change_state(shooter.States.off)
            self._speed = 1.0
            if current_avg < 4.0:# and current_rate <= 0.0:  # amps
                self.state = States.pinning"""
            if time.time() - self.intake_time > 0.5:
                self.state = States.pinning

        if self.state == States.pinning:
            self._speed = -0.3
            self.shooter.change_state(shooter.States.backdriving)
            self.intake_motor.setVoltageRampRate(120.0)  # Max ramp rate
            if current_avg > 4 and current_rate >= 0.0:
                self.state = States.pinned

        if self.state == States.pinned:
            self.shooter.change_state(shooter.States.off)
            self._speed = 0.0
            if self.log_queue:
                self.log_current()

        if self.state == States.fire:
            self.intake_motor.setVoltageRampRate(240.0)  # Max ramp rate
            if abs(self.shooter.shooter_motor.getClosedLoopError())<= 0.02*(self.shooter.max_speed) and self.shooter._speed != 0.0:
                self._speed = 1.0

        self.intake_motor.set(self._speed)
