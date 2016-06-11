from wpilib import CANTalon

from components.chassis import Chassis


class Shooter:
    # closed-loop controls for shooting mechanism

    shooter_motor = CANTalon
    chassis = Chassis
    shoot_encoder_cpr = 4096.0
    max_speed = 36000.0
    # shoot_percentage = 0.99
    shoot_percentage = 0.67

    def __init__(self):
        self._changed_state = True
        self.initialised = False
        self._speed = 0.0

    def up_to_speed(self):
        return (abs(self.shooter_motor.getClosedLoopError()) <= 0.02 * (self.max_speed)
                and self.shooter_motor.getSetpoint() != 0.0
                and abs(self.shooter_motor.get()) > abs(self.shooter_motor.getSetpoint() * 0.5)
                )

    def shoot(self):
        self.shooter_motor.set(-Shooter.max_speed * self.shoot_percentage)

    def backdrive(self):
        self.shooter_motor.set(Shooter.max_speed * 0.01)

    def backdrive_recovery(self):
        self.shooter_motor.set(Shooter.max_speed * 1.0)

    def stop(self):
        self.shooter_motor.set(0.0)

    def execute(self):
        if not self.initialised:
            self.shooter_motor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
            self.shooter_motor.changeControlMode(CANTalon.ControlMode.Speed)
            self.shooter_motor.reverseSensor(True)
            self.shooter_motor.setPID(0.075, 0.00075, 0,
                                      1023.0 / Shooter.max_speed, izone=3000)
            self.initialised = True
