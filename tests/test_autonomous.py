from magicbot.magic_tunable import setup_tunables
from unittest.mock import MagicMock
from autonomous.autonomous import ObstacleHighGoal
import math

class StepController(object):
    '''
        Robot test controller
    '''

    def __init__(self, control, on_step):
        '''constructor'''
        self.control = control
        self.step = 0
        self._on_step = on_step

    def __call__(self, tm):
        '''Called when a new robot DS packet is received'''
        self.step += 1
        return self._on_step(tm, self.step)

def test_autonomous_state_machine(control):
    dx = 1
    dy = -1
    heading = math.pi/3
    portcullis=True
    class TestAuto(ObstacleHighGoal):
        MODE_NAME = "Test Auto"
        DEFAULT = True

        def __init__(self):
            super().__init__(dx, dy, heading, True)
    #arbitary values, just so we have something to test against
    a = TestAuto()
    a.chassis = MagicMock()
    a.shooter = MagicMock
    a.shooter.shoot = MagicMock()
    a.intake = MagicMock()
    a.defeater = MagicMock()
    a.defeater_motor = MagicMock()
    a.defeater_motor.set = MagicMock()
    a.bno055 = MagicMock()
    a.boulder_automation = MagicMock()
    a.boulder_automation.shoot_boulder = MagicMock()
    a.chassis.distance_pid.onTarget = MagicMock(return_value=False)
    a.chassis.heading_hold_pid.onTarget = MagicMock(return_value=False)
    a.chassis.on_range_target = MagicMock(return_value=False)
    a.chassis.on_vision_target = MagicMock(return_value=False)
    setup_tunables(a, "autonomous")
    def _on_step(tm, step):
        if step == 1:
            a.engage()
            assert a.current_state == "deploy_defeater"
        elif step == 2:
            assert a.defeater_motor.set.called
            assert a.chassis.distance_pid.setOutputRange.called
            a.chassis.field_displace.assert_called_with(a.straight, 0.0)
            assert a.current_state == "breach_defence"
        elif step == 3:
            assert a.current_state == "breach_defence"
            a.chassis.distance_pid.onTarget = MagicMock(return_value=True)
        elif step == 4:
            assert a.chassis.heading_hold_pid.setSetpoint.called
            assert a.defeater_motor.set.callled
            assert a.current_state == "spinning"
        elif step == 5:
            assert a.current_state == "spinning"
            a.chassis.heading_hold_pid.onTarget = MagicMock(return_value=True)
        elif step == 6:
            a.chassis.field_displace.assert_called_with(dx, dy)
            assert a.current_state == "strafing"
            a.chassis.distance_pid.onTarget = MagicMock(return_value=False)
        elif step == 7:
            assert a.current_state == "strafing"
            a.chassis.distance_pid.onTarget = MagicMock(return_value=True)
        elif step == 8:
            assert a.current_state == "range_finding"
            assert a.chassis.distance_pid.setOutputRange.called
            assert a.chassis.distance_pid.reset.called
            assert a.chassis.zero_encoders.called
            assert a.chassis.range_setpoint == a.chassis.correct_range
        elif step == 9:
            assert a.current_state == "range_finding"
            a.chassis.on_range_target = MagicMock(return_value=True)
        elif step == 10:
            a.chassis.on_range_target = MagicMock(return_value=False)
            assert a.current_state == "visual_tracking"
            assert a.chassis.track_vision
            assert a.chassis.distance_pid.reset.called
            a.chassis.distance_pid.setSetpoint.assert_called_with(0.0)
            assert a.chassis.zero_encoders.called
            assert a.chassis.distance_pid.enable.called
            assert a.shooter.shoot.called
        elif step == 11:
            assert a.current_state == "visual_tracking"
            a.chassis.on_range_target = MagicMock(return_value=True)
            a.chassis.on_vision_target = MagicMock(return_value=True)
        elif step == 12:
            assert a.boulder_automation.shoot_boulder.called
            assert not a.current_state
        else:
            return False
        a.execute()  # Magicbot normally does this for us
        return True
    c = StepController(control, _on_step)
    control.run_test(c)
    assert c.step == 13
