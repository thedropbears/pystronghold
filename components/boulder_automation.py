from magicbot.state_machine import StateMachine, state
from components.shooter import Shooter
from components.intake import Intake


class BoulderAutomation(StateMachine):
    intake = Intake
    shooter = Shooter

    def __init__(self):
        super().__init__()

    def toggle_intake_boulder(self):
        if not self.is_executing:
            self.engage("pre_intake")
        else:
            self.done()

    def intake_boulder(self):
        self.engage("pre_intake")

    def toggle_shoot_boulder(self):
        if not self.is_executing:
            self.engage("pre_fire")
        else:
            self.done()

    def shoot_boulder(self):
        self.engage("pre_fire")

    def done(self, stop_intake=True):
        super().done()
        if stop_intake:
            self.intake.stop()
        self.shooter.stop()

    @state(first=True)
    def pre_intake(self):
        self.intake.intake()
        self.intake.clear_queues()
        if self.intake.up_to_speed():
            self.next_state("intaking")

    @state(must_finish=True)
    def intaking(self):
        if self.intake.ball_detected():
            self.next_state("intaking_contact")

    @state(must_finish=True)
    def intaking_contact(self, state_tm):
        self.shooter.backdrive()
        if state_tm > 0.5:
            self.next_state("pinning")

    @state(must_finish=True)
    def pinning(self):
        self.shooter.backdrive()
        self.intake.backdrive_pin()
        if self.intake.slowing():
            self.intake.jam()
            self.next_state("pinned")

    @state(must_finish=True)
    def pinned(self):
        self.shooter.stop()
        if self.intake.pinned():
            self.done(stop_intake=False)
            self.intake.write_log = True

    @state(must_finish=True)
    def pre_fire(self):
        self.shooter.shoot()
        if self.shooter.up_to_speed():
            self.next_state("firing")

    @state(must_finish=True)
    def firing(self, state_tm):
        self.intake.intake()
        if state_tm > 0.5:
            self.done()

    @state(must_finish=False)
    def backdrive_manual(self):
        self.intake.backdrive_slow()
        self.shooter.backdrive_recovery()
