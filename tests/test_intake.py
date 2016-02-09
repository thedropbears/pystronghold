from components.intake import Intake, States
from unittest.mock import MagicMock

def test_toggle():
    intake = Intake()
    intake.intake_motor = MagicMock()
    intake.shooter = MagicMock()
    intake.changed_state = False
    intake.states = States.no_ball
    intake.toggle()
    intake.execute()
    intake.execute()
    assert intake._speed != 0.0
    intake.states = States.intaking_free
    intake.toggle()
    intake.execute()
    intake.execute()
    assert intake._speed == 0.0
    assert intake.intake_motor.set.call_count == 2
