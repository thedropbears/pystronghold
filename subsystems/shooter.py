from wpilib import CANTalon
from robot_map import RobotMap

class Shooter():
#closed-loop controls for shooting mechanism

    shoot_encoder_cpr = 4096.0
    max_speed = 29861.0


    def __init__ (self, robot):
       self.shoot_motor = CANTalon(5)
       self.shoot_motor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
       self.shoot_motor.changeControlMode(CANTalon.ControlMode.Speed)
       self.shoot_motor.reverseSensor(True)
       self.shoot_motor.setPID(0.1, 0.0001, 0, 1023.0/Shooter.max_speed)

    def shoot(self, throttle):
        self.shoot_motor.set(-throttle*Shooter.max_speed)


