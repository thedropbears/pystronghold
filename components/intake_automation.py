import magicbot.state_machine

class IntakeAutomation(magicbot.state_machine.StateMachine):
    intake = Intake
    shooter = Shooter

    def done(self):
        super().done()
        self.intake.stop()

    @state(first=True)
    def intaking_free(self):
        self.intake.intake()
        if intake._speed == 0.7 and intake.intake_motor.getClosedLoopError() < Intake.max_speed*0.05:
            self.next_state("up_to_speed")
