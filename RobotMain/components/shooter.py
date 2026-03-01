import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants


class Shooter:
    _spin_up = will_reset_to(False)
    _feed = will_reset_to(False)


    def setup(self) -> None:
        self.feeder_spark = SparkMax(
            constants.kShooterFeederCanId, SparkMax.MotorType.kBrushless
        )
        self.flywheel_top_spark = SparkMax(
            constants.kShooterFlywheelTopCanId, SparkMax.MotorType.kBrushless
        )
        self.flywheel_bottom_spark = SparkMax(
            constants.kShooterFlywheelBottomCanId, SparkMax.MotorType.kBrushless
        )

        feeder_config = SparkMaxConfig()
        feeder_config.setIdleMode(constants.kShooterFeederIdleMode)
        feeder_config.smartCurrentLimit(constants.kShooterFeederCurrentLimit)

        flywheel_top_config = SparkMaxConfig()
        flywheel_top_config.setIdleMode(constants.kShooterFlywheelIdleMode)
        flywheel_top_config.smartCurrentLimit(constants.kShooterFlywheelCurrentLimit)

        flywheel_bottom_config = SparkMaxConfig()
        flywheel_bottom_config.setIdleMode(constants.kShooterFlywheelIdleMode)
        flywheel_bottom_config.smartCurrentLimit(constants.kShooterFlywheelCurrentLimit)
        flywheel_bottom_config.follow(
             constants.kShooterFlywheelTopCanId, invert= True
        )

        self.feeder_spark.configure(
            feeder_config,
            rev.RestMode.kRestSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.flywheel_top_configure(
            flywheel_top_config,
            rev.RestMode.kRestSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.flywheel_bottom_spark.configure(
            flywheel_bottom_config,
            rev.RestMode.kBRestSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.flywheel_encoder = self.flywheel_top_spark.getEncoder()

    def spin_up(self) -> None:
        self._spin_up = True

    def feed(self) -> None:
        self._feed = True

    def is_at_speed(self) -> bool:
        current_speed = abs(self.flywheel_encoder.getVelocity())
        target_rpm = abs(constants.kShooterFlywheelSpeed) * 5676.0 / 60.0
        if target_rpm ==0:
            return False
        return current_speed >= (target_rpm * constants.kShooterSpinUpThreshold)

    def execute(self) -> None:
        if self._spin_up:
            self.flywheel_top_spark.set(constants.kShooterFlyWheelSpeed)
        else:
            self.flywheel_top_spark.set(0.0)

        if self._feed:
            self.feeder_spark.set(constants.kShooterFeederSpeed)
        else:
            self.feeder_spark.set(0.0)