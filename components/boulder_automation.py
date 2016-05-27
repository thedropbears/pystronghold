import magicbot.state_machine
from magicbot.state_machine import state, timed_state
from components.shooter import Shooter
from components.intake import Intake

class BoulderAutomation(magicbot.state_machine.StateMachine):
    intake = Intake
    shooter = Shooter

    def __init__(self):
        super().__init__()
        self.armed = True # warning: if false intake will not feed shooter boulder

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False

    def toggle_intake_boulder(self):
        if not self.is_executing():
            engage("intaking_free")
        else:
            self.done()

    def intake_boulder(self):
        engage("intaking_free")

    def toggle_shoot_boulder(self):
        if not self.is_executing():
            engage("spin_up_shooter")
        else:
            self.done()

    def shoot_boulder(self):
        engage("spin_up_shooter")

    def done(self):
        super().done()
        self.intake.stop()
        self.shooter.stop()
        self.armed = True

    @state(first=True)
    def intaking_free(self):
        self.intake.intake()
        self.intake.clear_queues()
        if intake.up_to_speed:
            self.next_state("up_to_speed")

    @state()
    def up_to_speed(self):
        if ball_detected:
            self.contact_time = time.time()
            self.next_state("intaking_contact")

    @timed_state(duration=0.3, next_state="pinning")
    def intaking_contact(self):
        self.shooter.backdrive()

    @state()
    def pinning(self):
        self.shooter.backdrive()
        self.intake.backdrive_pin()
        if intake.slowing():
            self.intake_motor.changeControlMode(CANTalon.ControlMode.Position)
            self.intake_motor.setPID(1.0, 0.0, 0.0)
            self.intake_motor.setPosition(0.0)
            self.intake_motor.set(-1000)
            self.next_state("pinned")

    @state()
    def pinned(self):
        self.shooter.off()
        self._speed = 0.0
        if intake.pinned():
            self.done()
            self.intake.write_log = True

    @state()
    def spin_up_shooter(self):
        self.shooter.start()
        if self.shooter.up_to_speed() and self.armed:
            self.next_state("feed_boulder")

    @timed_state(duration=0.3)
    def feed_boulder(self):
        self.intake.intake()
