
from wpilib import CANTalon, PowerDistributionPanel

from components import shooter

import csv, time

import logging
from _collections import deque

class States:
    no_ball = 0
    up_to_speed = 10
    intaking_free = 1
    intaking_contact = 2
    pinning = 4
    pinned = 5
    fire = 6

class Intake:
    intake_motor = CANTalon
    shooter = shooter.Shooter

    max_speed = 9000.0

    def __init__(self):
        self._speed = 0.0
        self.state = States.no_ball
        self.current_deque = deque([0.0] * 3, 3)  # Used to average currents over last n readings
        self.log_queue = []
        self.velocity_queue = []
        self.intake_time = 0.0
        self.previous_velocity = 0.0
        self.shoot_time = None

    def stop(self):
        self.state = States.no_ball

    def toggle(self):
        if self.state != States.no_ball and self.state != States.pinned:
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
        csv_file = open("/tmp/velocity_log.csv", "a")
        csv_file.write(str(self.velocity_queue).strip('[]').replace(' ', '')+"\n")
        csv_file.close()
        self.log_queue = []
        self.velocity_queue = []

    def on_enable(self):
        self.stop()

    def execute(self):
        # add next reading on right, will automatically pop on left
        maxlen = self.current_deque.maxlen
        prev_current_avg = sum(self.current_deque)/maxlen
        self.current_deque.append(self.intake_motor.getOutputCurrent())
        current_avg = sum(self.current_deque) / maxlen
        current_rate = current_avg - prev_current_avg#self.current_deque[maxlen-1]-self.current_deque[maxlen-2]

        velocity = self.intake_motor.get()
        acceleration = velocity - self.previous_velocity

        if self.state != States.no_ball and self.state != States.pinned:
            self.log_queue.append(self.current_deque[maxlen-1])
            self.velocity_queue.append(self.intake_motor.get())

        if self.state == States.no_ball:
            self._speed = 0.0

        if self.state == States.intaking_free:
            self.intake_motor.changeControlMode(CANTalon.ControlMode.Speed)
            self.intake_motor.setPID(0.0, 0.0, 0.0, 1023.0/Intake.max_speed)
            self.shooter.change_state(shooter.States.off)
            if self._speed == 1.0 and self.intake_motor.getClosedLoopError() < Intake.max_speed*0.05:
                self.state = States.up_to_speed
            self._speed = 1.0

        if self.state == States.up_to_speed:
            if self.intake_motor.getClosedLoopError() > Intake.max_speed*0.01 and acceleration < 0.0:
                self.state = States.intaking_contact

        if self.state == States.intaking_contact:
            self.shooter.change_state(shooter.States.backdriving)
            if acceleration > 0.0:#self.intake_motor.getClosedLoopError() < Intake.max_speed*0.1:
                self.state = States.pinning

        if self.state == States.pinning:
            self._speed = -0.3
            self.shooter.change_state(shooter.States.backdriving)
            if velocity < 0.0 and acceleration > 0.0:
                self.state = States.pinned
                self.intake_motor.changeControlMode(CANTalon.ControlMode.Position)
                self.intake_motor.setPID(1.0, 0.0, 0.0)
                self.intake_motor.setPosition(0.0)
                self.intake_motor.set(-1000)

        if self.state == States.pinned:
            self.shooter.change_state(shooter.States.off)
            self._speed = 0.0
            if self.log_queue:
                self.log_current()

        if self.state == States.fire:
            self.intake_motor.changeControlMode(CANTalon.ControlMode.Speed)
            self.intake_motor.setPID(0.0, 0.0, 0.0, 1023.0/Intake.max_speed)
            if abs(self.shooter.shooter_motor.getClosedLoopError())<= 0.02*(self.shooter.max_speed) and self.shooter._speed != 0.0:
                self._speed = 1.0
                if not self.shoot_time:
                    self.shoot_time = time.time()
            if self.shoot_time and time.time() - self.shoot_time > 1.0:
                self.state = States.no_ball
                self.shooter.stop()
                self.shoot_time = None

        if self.state != States.pinned:
            self.intake_motor.set(self._speed*Intake.max_speed)

        self.previous_velocity = velocity
