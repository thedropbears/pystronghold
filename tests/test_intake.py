from components.intake import Intake
from unittest.mock import MagicMock

def test_set_speed():
    intake = Intake()
    intake.intake_motor = MagicMock()
    intake.set_speed(0.5)
    intake.execute()
    assert intake._speed != 0.0
    assert intake.intake_motor.set.called

def test_stop():
    shooter = Intake()
    shooter.intake_motor = MagicMock()
    shooter._speed = 12345
    shooter.stop()
    shooter.execute()
    assert shooter._speed == 0.0
    assert shooter.intake_motor.set.called

def test_toggle():
    shooter = Intake()
    shooter.intake_motor = MagicMock()
    shooter.toggle(0.5)
    shooter.execute()
    assert shooter._speed != 0.0
    shooter.toggle(0.5)
    shooter.execute()
    assert shooter._speed == 0.0
    assert shooter.intake_motor.set.call_count == 2
