import math

import wpilib
import sys
# sys.path.append("..")
from components.swerve_drive import SwerveDrive
from components.thundervikes_super_scorer import ThunderVikesSuperScorer
import constants as constants

class DefaultStrategy():
    MODE_NAME = 'default_strategy'
    DEFAULT = True

    swerve_drive: SwerveDrive
    # super_scorer: ThunderVikesSuperScorer

    def on_enable(self):
        self.timer = wpilib.Timer()
        self.timer.start()
        self.state = 'STEP 1'

    def on_disable(self):
        self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, True, False)

    def on_iteration(self, time_elapsed):
        if self.state == 'STEP 1':

            if self.timer.get() < 2.0:
                self.swerve_drive.set_drive_command(
                    2.5/ constants.kMaxSpeed, 0.0, 0.0, True, False
                )
            else:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, 0.0, True, False
                )
                self.state = 'STEP 2'
                self.timer.reset()
        elif self.state == 'STEP 2':
            self.swerve_drive.set_drive_command(
                0.0, 0.0, -1*math.pi /2.0, True, False
            )
            self.state = 'STEP 3'
            self.timer.reset()
        elif self.state == 'STEP 3':
            if self.timer.get() < 0.16:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, -1*math.pi / 2.0, True, False
                )
            else:
                self.state = 'DONE'
        else:
            self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, True, False)