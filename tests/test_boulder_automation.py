from components.boulder_automation import BoulderAutomation
from magicbot.magic_tunable import setup_tunables
from unittest.mock import MagicMock

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

def test_intake_boulder(control):
    ba = BoulderAutomation()
    ba.intake = MagicMock()
    ba.shooter = MagicMock()
    ba.intake.up_to_speed = MagicMock(return_value=True)
    ba.intake.ball_detected = MagicMock(return_value=True)
    ba.intake.slowing = MagicMock(return_value=True)
    ba.intake.pinned = MagicMock(return_value=True)
    setup_tunables(ba, "boulder_automation")
    def _on_step(tm, step):
        if step == 1:
            ba.intake_boulder()
            assert ba.current_state == "pre_intake"
        elif step == 2:
            assert ba.intake.intake.called
            assert ba.current_state == "intaking"
        elif step == 3:
            pass
        elif step <= 19:
            assert ba.shooter.backdrive.called
            assert ba.current_state == "intaking_contact"
        elif step == 20:
            assert ba.current_state == "pinning"
        elif step == 21:
            assert ba.shooter.backdrive.called
            assert ba.intake.backdrive_pin.called
            assert ba.current_state == "pinned"
        elif step == 22:
            assert not ba.current_state
        else:
            return False
        ba.execute()  # Magicbot normally does this for us
        return True
    c = StepController(control, _on_step)
    control.run_test(c)
    assert c.step == 23

def test_shoot_boulder(control):
    ba = BoulderAutomation()
    ba.intake = MagicMock()
    ba.shooter = MagicMock()
    ba.shooter.up_to_speed = MagicMock(return_value=True)
    setup_tunables(ba, "boulder_automation")
    
    def _on_step(tm, step):
        if step == 1:
            ba.shoot_boulder()
            assert ba.current_state == "pre_fire"
        elif step == 2:
            # Requires a step for the test for up_to_speed to return true
            assert ba.shooter.shoot.called
        elif step <= 20:
            assert ba.current_state == "firing"
            assert ba.is_executing
            assert ba.intake.intake.called
        if step > 30:  # Should inject ball for around 25 steps - 0.5s
            assert not ba.is_executing
            assert ba.intake.stop.called
            assert ba.shooter.stop.called
        if step == 40:
            return False
        ba.execute()  # Magicbot normally does this for us
        return True
    
    c = StepController(control, _on_step)
    control.run_test(c)
    assert c.step == 40

def test_backdrive_manual():
    ba = BoulderAutomation()
    ba.intake = MagicMock()
    ba.shooter = MagicMock()
    setup_tunables(ba, "boulder_automation")

    ba.engage("backdrive_manual")
    ba.execute()
    assert ba.intake.backdrive_slow.called
    assert ba.shooter.backdrive_recovery.called
    ba.done()

def test_toggle_intake_boulder():
    ba = BoulderAutomation()
    ba.intake = MagicMock()
    ba.shooter = MagicMock()
    setup_tunables(ba, "boulder_automation")

    assert not ba.current_state
    ba.toggle_intake_boulder()
    ba.execute()
    assert ba.current_state
    ba.toggle_intake_boulder()
    ba.execute()
    assert not ba.current_state

def test_toggle_shoot_boulder():
    ba = BoulderAutomation()
    ba.intake = MagicMock()
    ba.shooter = MagicMock()
    setup_tunables(ba, "boulder_automation")

    assert not ba.current_state
    ba.toggle_shoot_boulder()
    ba.execute()
    assert ba.current_state
    ba.toggle_shoot_boulder()
    ba.execute()
    assert not ba.current_state
