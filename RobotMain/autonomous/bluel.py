import math

import wpilib

from components.swerve_drive import SwerveDrive
from components.thundervikes_super_scorer import ThunderVikesSuperScorer
import constants as constants


class BlueL():
    MODE_NAME = 'BLUE LEFT SIDE'
    DEFAULT = False

    swerve_drive: SwerveDrive
    #super_scorer: ThunderVikesSuperScorer

    def on_enable(self):
        self.timer = wpilib.Timer()
        self.timer.start()
        self.state = 'STEP 1'

    def on_disable(self):
        self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, True, False)
        #self.super_scorer.stop_all()

    def on_iteration(self, time_elapsed):
        if self.state == 'STEP 1':

            if self.timer.get() < 2.0:
                self.swerve_drive.set_drive_command(
                    2.5 / constants.kMaxSpeed, 0.0, 0.0, True, False
                )
            else:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, 0.0, True, False
                )
                self.state = 'STEP 2'
                self.timer.reset()
        elif self.state == 'STEP 2':
            self.swerve_drive.set_drive_command(
                0.0, 0.0, 1*math.pi /2.0 , True, False
            )
            self.state = 'STEP 3'
            self.timer.reset()
        elif self.state == 'STEP 3':
            if self.timer.get() > 0.18:

                if self.timer.get() < 0.82:
                    self.swerve_drive.set_drive_command(
                        0.0, -2.5/constants.kMaxSpeed, 0.0, True, False

                    )
                else:
                    self.swerve_drive.set_drive_command(
                        0.0, 0.0, 0.0, True, False
                    )
                    self.state = 'STEP 4'
                    self.timer.reset()
            else:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, 1*math.pi / 2.0, True, False
                )
        elif self.state == 'STEP 4':
            if self.timer.get() < 1.0:
                pass
                #self.super_scorer.intake_ball()
            elif self.timer.get() > 1.0 + 1.0/8.0:
                #self.super_scorer.intake_ball()
                self.swerve_drive.set_drive_command(
                    0.0, -2.0/constants.kMaxSpeed, 0.0, True, False
                )
            else:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, 0.0, True, False
                )
                self.state = 'STEP 5'
                self.timer.reset()

        elif self.state == 'STEP 5':
            if self.timer.get() < 0.8:
                self.swerve_drive.set_drive_command(
                    0.0, 2.0/constants.kMaxSpeed, 0.0, True, False
                )
            elif self.timer.get() < 0.8 + 1.0/1.95:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, -1*math.pi,  True, False
                )
            else:
                self.state = 'STEP 6'
                self.timer.reset()
        elif self.state == 'STEP 6':
            if self.timer.get() < 3.0:
                self.swerve_drive.set_drive_command(
                    -2.0/constants.kMaxSpeed, 0.0, 0.0, True, False
                )
            else:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, 0.0, True, False
                )
                #self.super_scorer.shoot_ball()
                self.state = 'STEP 7'
                self.timer.reset()
        elif self.state == 'STEP 7':
            if self.timer.get() < 5.0:
                pass
                #self.super_scorer.shoot_ball()
            else:
                self.state = 'STEP 8'
                self.timer.reset()
        elif self.state == 'STEP 8':
            if self.timer.get() < 3.0:
                self.swerve_drive.set_drive_command(
                    2.0/constants.kMaxSpeed, 0.0, 0.0, True, False
                )
            else:
                self.swerve_drive.set_drive_command(
                    0.0, 0.0, 0.0, True, False
                )
                self.state = 'DONE'
        else:
            self.swerve_drive.set_drive_command(
                0.0, 0.0, 0.0, True, False
            )
            #self.super_scorer.stop_all()
