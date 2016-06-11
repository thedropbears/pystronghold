
from components.intake import Intake
from unittest.mock import MagicMock
from wpilib import CANTalon

def test_control_methods():
    intake = Intake()
    intake.intake_motor = MagicMock()
    intake.intake_motor.set = MagicMock()
    intake.speed_mode = MagicMock()
    intake.position_mode = MagicMock()

    intake.intake()
    intake.intake_motor.set.assert_called_with(0.7*Intake.max_speed)
    assert intake.speed_mode.called
    intake.backdrive_slow()
    intake.intake_motor.set.assert_called_with(-0.5*Intake.max_speed)
    assert intake.speed_mode.called
    intake.backdrive_pin()
    intake.intake_motor.set.assert_called_with(-0.3*Intake.max_speed)
    assert intake.speed_mode.called
    intake.stop()
    intake.intake_motor.set.assert_called_with(0.0)
    assert intake.speed_mode.called
    intake.jam()
    intake.intake_motor.set.assert_called_with(-1000)
    assert intake.position_mode.called

def test_up_to_speed():
    intake = Intake()
    intake.intake_motor = MagicMock()

    #test setpoint condition
    intake.intake_motor.getClosedLoopError = MagicMock(return_value=0.0)

    intake.intake_motor.getSetpoint = MagicMock(0.0)
    assert not intake.up_to_speed()

    intake.intake_motor.getSetpoint = MagicMock(return_value=0.7*Intake.max_speed)
    assert intake.up_to_speed()

    #test closed loop error condition
    intake.intake_motor.getSetpoint = MagicMock(return_value=0.7*Intake.max_speed)

    intake.intake_motor.getClosedLoopError = MagicMock(return_value=0.05*Intake.max_speed)
    assert not intake.up_to_speed()

    intake.intake_motor.getClosedLoopError = MagicMock(return_value=0.0)
    assert intake.up_to_speed()

def test_ball_detected():
    intake = Intake()
    intake.intake_motor = MagicMock()

    #test error condition
    intake.acceleration = -1.0
    intake.current_rate = 1.0

    intake.intake_motor.getClosedLoopError = MagicMock(return_value=0.0)
    assert not intake.ball_detected()

    intake.intake_motor.getClosedLoopError = MagicMock(return_value=0.2*Intake.max_speed)
    assert intake.ball_detected()

    #test acceleration condition
    intake.current_rate = 1.0
    intake.intake_motor.getClosedLoopError = MagicMock(return_value=0.2*Intake.max_speed)

    intake.acceleration = 0.0
    assert not intake.ball_detected()

    intake.acceleration = -1.0
    assert intake.ball_detected()

    #test current rate condition
    intake.acceleration = -1.0
    intake.intake_motor.getClosedLoopError = MagicMock(return_value=0.2*Intake.max_speed)

    intake.current_rate = 0.0
    assert not intake.ball_detected()

    intake.current_rate = 1.0
    assert intake.ball_detected()

def test_slowing():
    intake = Intake()

    #test velocity condition
    intake.acceleration = 1.0

    intake.velocity = 0.0
    assert not intake.slowing()

    intake.velocity = -1.0
    assert intake.slowing()

    #test acceleration condition
    intake.velocity = -1.0

    intake.acceleration = 0.0
    assert not intake.slowing()

    intake.acceleration = 1.0
    assert intake.slowing()

def test_pinned():
    intake = Intake()
    intake.intake_motor = MagicMock()

    # test error condition
    intake.intake_motor.get = MagicMock(return_value=-501)
    intake.intake_motor.getSetpoint = MagicMock(return_value=-1000)

    intake.intake_motor.getClosedLoopError = MagicMock(return_value=20)
    assert not intake.pinned()

    intake.intake_motor.getClosedLoopError = MagicMock(return_value=19)
    assert intake.pinned()

    # test only return after half of setpoint reached
    intake.intake_motor.getClosedLoopError = MagicMock(return_value=19)
    intake.intake_motor.getSetpoint = MagicMock(return_value=-1000)

    intake.intake_motor.get = MagicMock(return_value=-500)
    assert not intake.pinned()

    intake.intake_motor.get = MagicMock(return_value=-501)
    assert intake.pinned()

def test_modes():
    intake = Intake()
    intake.intake_motor=MagicMock()
    intake.intake_motor.changeControlMode = MagicMock()
    intake.intake_motor.setPID = MagicMock()
    intake.intake_motor.setPosition = MagicMock()

    #speed mode
    intake.speed_mode()
    intake.intake_motor.changeControlMode.assert_called_with(CANTalon.ControlMode.Speed)
    assert intake.intake_motor.setPID.called

    #position mode
    intake.position_mode()
    intake.intake_motor.changeControlMode.assert_called_with(CANTalon.ControlMode.Position)
    assert intake.intake_motor.setPID.called
    intake.intake_motor.setPosition.assert_called_with(0.0)
