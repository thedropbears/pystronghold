from components.defeater import Defeater
from unittest.mock import MagicMock

def test_control_methods():
    defeater = Defeater()
    defeater.defeater_motor = MagicMock()
    defeater.defeater_motor.set = MagicMock()

    defeater.up()
    defeater.defeater_motor.set.assert_called_with(0.5)

    defeater.down()
    defeater.defeater_motor.set.assert_called_with(-0.5)

def test_execute():
    defeater = Defeater()
    defeater.defeater_motor = MagicMock()
    defeater.defeater_motor.set = MagicMock()

    defeater.defeater_motor.getOutputCurrent = MagicMock(return_value=9.0)
    defeater.execute()
    assert defeater.defeater_motor.set.not_called

    defeater.defeater_motor.getOutputCurrent = MagicMock(return_value=11.0)
    defeater.execute()
    defeater.defeater_motor.set.assert_called_with(0.0)
