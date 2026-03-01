"""
hopper.py - Hopper/Conveyor System (MagicBot Component)
========================================================

WHAT IS THE HOPPER?
The hopper is a conveyor belt system that moves balls from the intake to the
shooter. It uses 3 motors spinning 3 rollers to push balls along.

The hopper sits on a slight incline - balls naturally tend to roll toward the
shooter side due to gravity. This means:
- When pulling balls FROM the intake (against the incline), we need MORE power
- When feeding balls TO the shooter (gravity helps), we need LESS power

MAGIC BOT COMPONENT:
- setup() is called once when the robot starts
- execute() is called every 20ms (50 times per second)
- will_reset_to variable automatically reset each cycle (safety feature)
"""

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants

class Hopper:
    """
    MagicBot component that controls the hopper conveyor system.

    the hopper moves balls between the intake and the shooter using
    3 roller motors arrange in a conveyor belt configuration.

    HOW MAGICBOT COMPONENTS WORK:
    - variables set with will_reset_to() automatically reset each cycle
    - This means if nothing calls feed_from_intake() or feed_to_shooter(),
    the hopper stops automatically
    - This is a safety feature! if the code crashes, the hopper stops.
    """

    _motor_speed = will_reset_to(0.0)

    def setup(self) -> None:
        """
        Called once when the robot starts up.

        Creates all 3 hopper motor controllers and configures them
        with the correct settings (current limits, idle mode, etc.).
        """

        self.motor1 = SparkMax(constants.kHopperMotor1CanId, SparkMax.MotorType.kBrushless)
        self.motor2 = SparkMax(constants.kHopperMotor2CanId, SparkMax.MotorType.kBrushless)
        self.motor3 = SparkMax(constants.kHopperMotor3CanId, SparkMax.MotorType.kBrushless)

        config = SparkMaxConfig()
        config.setIdleMode(constants.kHopperMotorIdleMode)
        config.smartCurrentLimit(constants.kHopperMotorCurrentLimit)

        self.motor1.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters.
        )
        self.motor2.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.motor3.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        def feed_from_intake(self) -> None:
            """
            Run the hopper to pull balls from the intake toward the shooter.

            This runs against the incline so it uses the higher intake speed.
            """
            self._motor_speed = constants.kHopperIntakeSpeed

        def feed_to_shooter(self) -> None:
            """
            Run the hopper to push balls toward the shooter.

            Gravity assists in this direction so it uses the lower shooter speed.
            """
            self._motor_speed = constants.kHopperShooterSpeed

        def stop(self) -> None:
            """stop all hopper motors immediately."""
            self._motor_speed = 0.0

        def execute(self) -> None:
            """
            Called every 20ms by MagicBot.

            Sets all 3 motors to the requested speed. If no method was called
            this cycle, _motor_speed resets to 0.0 and the hopper stops.
            """
            self.motor1.set(self._motor_speed)
            self.motor2.set(self._motor_speed)
            self.motor3.set(self._motor_speed)
