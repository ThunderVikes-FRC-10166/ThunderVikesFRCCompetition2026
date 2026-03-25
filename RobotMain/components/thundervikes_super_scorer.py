from magicbot import will_reset_to
from .intake import Intake
from .hopper import Hopper
from .shooter import Shooter

class ThunderVikesSuperScorer:

    intake: Intake
    hopper: Hopper
    shooter: Shooter

    IDLE = "idle"
    INTAKING = "intaking"
    LOADED = "loaded"
    SHOOTING = "shooting"
    REVERSING = "reversing"

    _want_intake = will_reset_to(False)
    _want_hopper = will_reset_to(False)
    _want_stop = will_reset_to(False)
    _want_reverse_shooter = will_reset_to(False)
    _want_shoot = will_reset_to(False)

    def setup(self) -> None:
        self.state = self.IDLE

    def intake_ball(self) -> None:
        self._want_intake = True

    def shoot_ball(self) -> None:
        self._want_shoot = True

    def stop_all(self) -> None:
        self._want_stop = True

    def reverse_shooter_feeder(self) -> None:
        self._want_reverse_shooter = True

    def get_state(self) -> str:
        return self.state

    def execute(self):

        # ============================================================
        # GLOBAL OVERRIDE: STOP EVERYTHING
        # ============================================================
        if self._want_stop:
            self.state = self.IDLE
            self.intake.stop()
            self.hopper.stop()
            self.shooter.stop()
            return

        # ============================================================
        # STATE TRANSITIONS
        # ============================================================
        if self.state == self.IDLE:
            if self._want_shoot:
                self.state = self.SHOOTING
            elif self._want_reverse_shooter:
                self.state = self.REVERSING

        elif self.state == self.SHOOTING:
            if not self._want_shoot:
                self.state = self.IDLE

        elif self.state == self.REVERSING:
            if not self._want_reverse_shooter:
                self.state = self.IDLE

        # ============================================================
        # STATE ACTIONS
        # ============================================================

        # -----------------------------
        # SHOOTING MODE
        # -----------------------------
        if self.state == self.SHOOTING:

            # Optional: allow intake to run during shooting
            if self._want_intake:
                self.intake.run_roller()
            else:
                self.intake.stop()

            # Spin up shooter
            self.shooter.do_spin_up()

            # Hopper always feeds upward during shooting
            self.hopper.feed_to_shooter()

            # Only feed when shooter is ready
            if self.shooter.is_at_speed():
                self.shooter.feed()

        # -----------------------------
        # REVERSE SHOOTER MODE
        # -----------------------------
        elif self.state == self.REVERSING:
            self.intake.stop()
            self.hopper.stop()
            self.shooter.reverse_feeder()

        # -----------------------------
        # IDLE MODE (normal intake allowed)
        # -----------------------------
        elif self.state == self.IDLE:

            # Allow intake to run normally when not shooting
            if self._want_intake:
                self.intake.open_arm()
                self.intake.run_roller()
            else:
                self.intake.stop()

            # Hopper + shooter idle
            self.hopper.stop()
            self.shooter.stop()

        # ============================================================
        # EXECUTE SUBSYSTEMS
        # ============================================================
        self.intake.execute()
        self.hopper.execute()
        self.shooter.execute()

