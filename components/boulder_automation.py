from magicbot import StateMachine, state, timed_state
from components.shooter import Shooter
from components.intake import Intake

class BoulderAutomation(StateMachine):
    intake = Intake
    shooter = Shooter

    def __init__(self):
        super().__init__()

    def toggle_intake_boulder(self):
        if not self.is_executing():
            self.engage("pre_intake")
        else:
            self.done()

    def intake_boulder(self):
        self.engage("pre_intake")

    def toggle_shoot_boulder(self):
        if not self.is_executing():
            self.engage("pre_fire")
        else:
            self.done()

    def shoot_boulder(self):
        self.engage("pre_fire")

    def done(self):
        super().done()
        self.intake.stop()
        self.shooter.stop()

    @state(first=True)
    def pre_intake(self):
        self.intake.intake()
        self.intake.clear_queues()
        if self.intake.up_to_speed():
            self.next_state("intaking")

    @state()
    def intaking(self):
        if self.intake.ball_detected():
            self.next_state("intaking_contact")

    @timed_state(duration=0.3, next_state="pinning")
    def intaking_contact(self):
        self.shooter.backdrive()

    @state()
    def pinning(self):
        self.shooter.backdrive()
        self.intake.backdrive_pin()
        if self.intake.slowing():
            self.intake.position_mode()
            self.intake_motor.set(-1000)
            self.next_state("pinned")

    @state()
    def pinned(self):
        self.shooter.off()
        if self.intake.pinned():
            self.done()
            self.intake.write_log = True

    @state()
    def pre_fire(self):
        self.shooter.shoot()
        if self.shooter.up_to_speed():
            self.next_state("firing")

    @timed_state(duration=0.3)
    def firing(self):
        self.intake.intake()
