import magicbot.state_machine
from components import shooter

class BoulderAutomation(magicbot.state_machine.StateMachine):
    intake = Intake
    shooter = Shooter

    def intake_boulder(self):
        engage("intaking_free")

    def shoot_boulder(self):
        engage("spin_up_shooter")

    def done(self):
        super().done()
        self.intake.stop()

    @state()
    def intaking_free(self):
        self.intake.intake()
        self.intake.clear_queues()
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
        self.shooter.change_state(shooter.States.backdriving)
        self.intake.backdrive_pin()
        if intake.slowing():
            self.intake_motor.changeControlMode(CANTalon.ControlMode.Position)
            self.intake_motor.setPID(1.0, 0.0, 0.0)
            self.intake_motor.setPosition(0.0)
            self.intake_motor.set(-1000)
            self.next_state("pinned")

    @state()
    def pinned(self):
        self.shooter.change_state(shooter.States.off)
        self._speed = 0.0
        if intake.pinned():
            self.done()
            self.intake.write_log = True

    @state()
    def spin_up_shooter(self):
        pass
