from components.shooter import Shooter
from unittest.mock import MagicMock
from wpilib import CANTalon

def test_control_methods():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter.shooter_motor.set = MagicMock()

    shooter.shoot()
    shooter.shooter_motor.set.assert_called_with(-Shooter.max_speed*Shooter.shoot_percentage)

    shooter.backdrive()
    shooter.shooter_motor.set.assert_called_with(Shooter.max_speed*0.01)

    shooter.backdrive_recovery()
    shooter.shooter_motor.set.assert_called_with(Shooter.max_speed*1.0)

    shooter.stop()
    shooter.shooter_motor.set.assert_called_with(0.0)

def test_up_to_speed():
    shooter = Shooter()
    shooter.shoter_motor = MagicMock()

    # test error condition
    shooter.shooter_motor.get = MagicMock(return_value=(Shooter.max_speed*Shooter.shoot_percentage/2.0)+1.0)
    shooter.shooter_motor.getSetpoint = MagicMock(return_value=Shooter.max_speed*Shooter.shoot_percentage)

    shooter.shooter_motor.getClosedLoopError = MagicMock(return_value=0.03*Shooter.max_speed)
    assert not shooter.up_to_speed()

    shooter.shooter_motor.getClosedLoopError = MagicMock(return_value=0.02*Shooter.max_speed)
    assert shooter.up_to_speed()

    # test that we are at least half of setpoint
    shooter.shooter_motor.getSetpoint = MagicMock(return_value=Shooter.max_speed*Shooter.shoot_percentage)
    shooter.shooter_motor.getClosedLoopError = MagicMock(return_value=0.02*Shooter.max_speed)

    shooter.shooter_motor.get = MagicMock(return_value=(Shooter.max_speed*Shooter.shoot_percentage/2.0))
    assert not shooter.up_to_speed()

    shooter.shooter_motor.get = MagicMock(return_value=(Shooter.max_speed*Shooter.shoot_percentage/2.0)+1.0)
    assert shooter.up_to_speed()
