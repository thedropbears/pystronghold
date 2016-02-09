from components.shooter import Shooter, States
from unittest.mock import MagicMock
from wpilib import CANTalon

def test_set_speed():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter.start_shoot()
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
    shooter.start_shoot()
    shooter.execute()
    shooter.execute()
    # Setpoint for Talon SRX should only get called once
    assert shooter.shooter_motor.set.call_count == 1

def test_toggle():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter.state = States.off
    shooter.changed_state = False
    shooter.toggle()
    shooter.execute()
    shooter.execute()
    assert shooter._speed != 0.0
    shooter.state = States.shooting
    shooter.changed_state = False
    shooter.execute()
    shooter.toggle()
    shooter.execute()
    shooter.execute()
    assert shooter._speed == 0.0
    assert shooter.shooter_motor.set.call_count == 2

def test_backdrive():
    shooter = Shooter()
    shooter.shooter_motor = MagicMock()
    shooter.state = States.backdriving
    shooter.execute()
    assert shooter._speed >= 0.0
