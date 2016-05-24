import magicbot.state_machine
from components import shooter

class IntakeAutomation(magicbot.state_machine.StateMachine):
    intake = Intake
    shooter = Shooter

    def done(self):
        super().done()
        self.intake.stop()

    @state(first=True)
    def intaking_free(self):
        self.intake.intake()
        if intake.up_to_speed:
            self.next_state("up_to_speed")

    @state()
    def up_to_speed(self):
        if ball_detected:
            self.contact_time = time.time()
            self.state = States.intaking_contact
            self.next_state("intaking_contact")

    @timed_state(duration=0.3, next_state="pinning")
    def intaking_contact(self):
        self.shooter.change_state(shooter.States.backdriving)

    @state()
    def pinning(self):
        pass
