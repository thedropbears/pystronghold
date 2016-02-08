from components.shooter import Shooter
from unittest.mock import MagicMock
from wpilib import CANTalon

def test_set_speed():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter.set_speed(0.5)
    shooter.execute()
    assert shooter._speed != 0.0
    shooter.shooter_motor.setFeedbackDevice.assert_called_once_with(CANTalon.FeedbackDevice.QuadEncoder)
    assert shooter.shooter_motor.set.called

def test_stop():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter._speed = 12345
    shooter.stop()
    shooter.execute()
    assert shooter._speed == 0.0
    assert shooter.shooter_motor.set.called

def test_changed_speed():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter.set_speed(0.5)
    shooter.execute()
    shooter.execute()
    # Setpoint for Talon SRX should only get called once
    assert shooter.shooter_motor.set.call_count == 1

def test_toggle():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter.toggle(0.5)
    shooter.execute()
    assert shooter._speed != 0.0
    shooter.toggle(0.5)
    shooter.execute()
    assert shooter._speed == 0.0
    assert shooter.shooter_motor.set.call_count == 2

