
from networktables import NetworkTable
from wpilib import CANTalon

from _collections import deque


class Intake:
    intake_motor = CANTalon

    max_speed = 9000.0

    def __init__(self):
        self._speed = 0.0
        self.current_deque = deque([0.0] * 3, 3)  # Used to average currents over last n readings
        self.log_queue = []
        self.velocity_queue = []
        self.previous_velocity = 0.0
        self.shoot_time = None
        self.sd = NetworkTable.getTable('SmartDashboard')
        self.write_log = False

    def intake(self):
        """ Spin the intake at the maximum speed to suck balls in """
        self.speed_mode()
        self.intake_motor.set(0.7*Intake.max_speed)

    def backdrive_slow(self):
        """ Backdrive the intake at 0.5 speed """
        self.speed_mode()
        self.intake_motor.set(-0.5*Intake.max_speed)

    def backdrive_pin(self):
        """ Used when pinning the ball """
        self.speed_mode()
        self.intake_motor.set(-0.3*Intake.max_speed)

    def stop(self):
        """ Stop the intake """
        self.speed_mode()
        self.intake_motor.set(0.0)

    def jam(self):
        """ Jam the ball in the intake """
        self.position_mode()
        self.intake_motor.set(-1000)

    def up_to_speed(self):
        """ Is the intake up to speed yet? """
        return (self.intake_motor.getSetpoint() == 0.7 * Intake.max_speed
                and self.intake_motor.getClosedLoopError() < Intake.max_speed * 0.05)

    def ball_detected(self):
        return (self.intake_motor.getClosedLoopError() > Intake.max_speed * 0.1
                and self.acceleration < 0.0 and self.current_rate > 0.0)

    def slowing(self):
        return self.velocity < 0.0 and self.acceleration > 0.0

    def pinned(self):
        return (self.intake_motor.getClosedLoopError() < 20
                and abs(self.intake_motor.get()) > abs(self.intake_motor.getSetpoint()) * 0.5)

    def speed_mode(self):
        self.intake_motor.changeControlMode(CANTalon.ControlMode.Speed)
        self.intake_motor.setPID(0.0, 0.0, 0.0, 1023.0/Intake.max_speed)

    def position_mode(self):
        self.intake_motor.changeControlMode(CANTalon.ControlMode.Position)
        self.intake_motor.setPID(1.0, 0.0, 0.0)
        self.intake_motor.setPosition(0.0)

    def clear_queues(self):
        self.log_queue = []
        self.velocity_queue = []

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
        self.current_avg = sum(self.current_deque) / maxlen
        self.current_rate = self.current_avg - prev_current_avg
        self.velocity = self.intake_motor.get()
        self.acceleration = self.velocity - self.previous_velocity

        self.sd.putDouble("intake_current_rate", self.current_rate)
        self.sd.putDouble("intake_current_avg", self.current_avg)
        self.sd.putDouble("intake_closed_loop_error",
                          self.intake_motor.getClosedLoopError())
        self.sd.putDouble("intake_acceleration", self.acceleration)
        self.sd.putDouble("intake_velocity", self.velocity)

        self.log_queue.append(self.current_deque[maxlen-1])
        self.velocity_queue.append(self.intake_motor.get())

        if self.write_log:
            self.log_current()
            self.write_log = True

        self.previous_velocity = self.velocity
