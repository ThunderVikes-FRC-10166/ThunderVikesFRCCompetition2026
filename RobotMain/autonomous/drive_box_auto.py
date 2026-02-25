import wpilib

class DriveBoxAuto():
    MODE_NAME = "Drive Box (SIM)"

    DEFAULT = False

    def __init__(self):
        # This will be filled in by robot.py  (we do that in robotInit)
        self.robot = None

        self.timer = wpilib.Timer()
        self.state = "FORWARD_1"

    def on_enable(self):
        self.timer.restart()
        self.state = "FORWARD_1"

    def on_iteration(self, time_elapsed: float):
        # This will use our own timer because it's easier for beginners to rest per stop
        t = self.timer.get()


        # Safety: if robot isn't attached yet, do nothing
        if self.robot is None:
            return

        # A simple "state machine" auto:
        # foward -left - foward- stop
        if self.state == "FORWARD_1":
            self.robot.swerve.drive(1.0, 0.0, 0.0)
            if t > 1.5:
                self.state = "LEFT"
                self.timer.restart()

        elif self.state == "LEFT":
            self.robot.swerve.drive(0.0, 1.0, 0.0)
            if t > 1.0:
                self.state = "FORWARD_2"
                self.timer.restart()

        else:
            # DONE state stop
            self.robot.swerve.drive(0.0, 0.0, 0.0)

    def on_disable(self):
        if self.robot is not None:
            self.robot.swerve.drive(0.0, 0.0, 0.0)