import wpilib

class DriveBoxAuto():
    MODE_NAME = "Drive Box (SIM)"

    DEFAULT = True

    def __init__(self):
        # This will be filled in by robot.py  (we do that in robotInit)
        self.robot = None

        self.timer = wpilib.Timer()
        self.state = "FORWARD_1"

    def on_enable(self):
        self.timer.restart()
        self.state = "FORWARD_1"

    def on_iteratiom(self, time_elapsed: float):
        # This will use our own timer because it's easier for beginners to rest per stop
        t = self.timer.get()
        