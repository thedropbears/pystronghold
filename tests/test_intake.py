from components.intake import Intake, States
from unittest.mock import MagicMock



def test_state_transition():
    intake = Intake()
    intake.intake_motor = MagicMock()
    intake.shooter = MagicMock()
    intake.intake_motor.getOutputCurrent = MagicMock(side_effect=
                                                     [0,  # no ball
                                                      2, 2, 2, 2, 2,  # intaking
                                                      3, 4, 5, 4, 3,  # ball in
                                                      2, 2, 2, 2, 2,  # backdriving
                                                      2, 3, 3, 4, 6,  # pinning
                                                      6, 6, 7, 7, 9,  # still pinning!
                                                      4, 4, 4, 4, 4,  # FIRE!
                                                      ])
    intake.execute()
    assert intake.state == States.no_ball

    intake.state = States.intaking_free
    for _ in range(5):
        intake.execute()
    assert intake.shooter.change_state.called
    assert intake.intake_motor.setVoltageRampRate.called
    intake.intake_motor.setVoltageRampRate.called = False
    assert intake.state == States.intaking_free

    for _ in range(5):
        intake.execute()
    assert intake.state == States.intaking_contact

    for _ in range(5):
        intake.execute()
    assert intake.state == States.pinning

    for _ in range(10):
        intake.execute()
    assert intake.state == States.pinned
    assert intake.shooter.change_state.called

    intake.state = States.fire
    intake.execute()
    assert intake._speed != 0.0
    assert intake.intake_motor.setVoltageRampRate.called

def test_toggle():
    intake = Intake()
    intake.toggle()
    assert intake.state == States.intaking_free
    intake.toggle()
    assert intake.state == States.no_ball
