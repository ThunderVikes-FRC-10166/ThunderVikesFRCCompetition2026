import wpilib
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants

class Intake:

    _arm_opening = will_reset_to(False)
    _arm_closing = will_reset_to(False)
    _roller_running = will_reset_to(False)

    def setup(self) -> None:
        self.arm_spark = SparkMax(constants.kIntakeArmCanId, SparkMax.MotorType.kBrushless)
