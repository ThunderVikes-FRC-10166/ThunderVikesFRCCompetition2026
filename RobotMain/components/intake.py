import wpilib
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import components.constants as constants

class Intake:

    _arm_opening = will_reset_to(False)
    _arm_closing = will_reset_to(False)
    _roller_running = will_reset_to(False)

    def setup(self) -> None:
        self.arm_spark = SparkMax(constants.kIntakeArmCanId, SparkMax.MotorType.kBrushless)
        self.roller_spark = SparkMax(constants.kIntakeRollerCanId, SparkMax.MotorType.kBrushless)

        arm_config = SparkMaxConfig()
        arm_config.setIdleMode(constants.kIntakeArmIdleMode)
        arm_config.smartCurrentLimit(constants.kIntakeArmCurrentLimit)

        roller_config = SparkMaxConfig()
        roller_config.setIdleMode(constants.kIntakeArmIdleMode)
        roller_config.smartCurrentLimit(constants.kIntakeRollerCurrentLimit)

        self.arm_spark.configure(
            arm_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.roller_spark.configure(
            roller_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.arm_forward_limit = wpilib.DigitalInput(constants.kIntakeArmForwardLimitDIO)
        self.arm_reverse_limit = wpilib.DigitalInput(constants.kIntakeArmReverseLimitDIO)

    def open_arm(self) -> None:
        self._arm_opening = True
        self._arm_closing = False

    def close_arm(self) -> None:
        self._arm_opening = False
        self._arm_closing = True

    def run_roller(self) -> None:
        self._roller_running = True

    def stop(self) -> None:
        self._arm_opening = False
        self._arm_closing = False
        self._roller_running = False

    def is_open(self) -> bool:
        return not self.arm_forward_limit.get()

    def is_closed(self) -> bool:
        return not self.arm_reverse_limit.get()

    def execute(self) -> None:
        print("IS OPEN ", self.is_open())
        print("IS CLOSED ", self.is_closed())
        if self._arm_opening and not self.is_open():
            print("its moving")
            self.arm_spark.set(constants.kIntakeArmSpeed)
        elif self._arm_closing and not self.is_closed():
            print("It is closing")
            self.arm_spark.set(-2.0*constants.kIntakeArmSpeed)
        elif self.is_open():
            self.arm_spark.set(0.0)
        elif self.is_closed():
            self.arm_spark.set(0.0)
        else:
            print("stopped moving")
            self.arm_spark.set(0.0)

        if self._roller_running:
            self.roller_spark.set(constants.kIntakeRollerSpeed)
        else:
            self.roller_spark.set(0.0)