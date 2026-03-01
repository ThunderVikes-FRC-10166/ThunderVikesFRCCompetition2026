from magicbot import will_reset_to
from intake import Intake
from hopper import Hopper
from shooter import Shooter

class ThunderVikesSuperScorer:

    intake: Intake
    hopper: Hopper
    shooter: Shooter

    IDLE = "idle"
    INTAKING = "intaking"
    LOADED = "loaded"
    SHOOTING = "shooting"

    _want_intake = will_reset_to(False)
    _want_hopper = will_reset_to(False)
    _want_stop = will_reset_to(False)

    def setup(self) -> None:
        self.state = self.IDLE

    def intake_ball(self) -> None:
        self._want_intake = True

    def shoot_ball(self) -> None:
        self._want_shoot = True

    def stop_all(self) -> None:
        self._want_stop = True

    def get_state(self) -> str:
        return self.state

    def execute(self) -> None:

        if self._want_stop:
            self.state = self.IDLE
            self.intake.stop()
            self.hopper.stop()
            self.shooter.stop()
            return
        if self.state == self.IDLE:
            if self._want_intake:
                self.state = self.INTAKING
            elif self._want_shoot:
                self.state = self.SHOOTING

        if self.state == self.INTAKING:
            self.intake.open_arm()
            self.intake.run_roller()
            self.hopper.feed_from_intake()

            if not self._want_intake:
                self.state = self.LOADED

        elif self.state == self.LOADED:
            if not self.intake.is_closed():
                self.intake.close_arm()

            if self._want_shoot:
                self.state = self.SHOOTING
            elif  self._want_intake:
                self.state = self.INTAKING

        elif self.state == self.SHOOTING:
            self.shooter.spin_up()
            self.hopper.feed_to_shooter()

            if self.shooter.is_at_speed():
                self.shooter.feed()

            if not self._want_shoot:
                self.state = self.IDLE


