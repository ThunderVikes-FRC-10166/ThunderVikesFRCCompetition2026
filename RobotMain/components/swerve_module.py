"""
swerve_module.py - Individual Swerve Module Controller
========================================================

WHAT IS A SWERVE MODULE?
A swerve module is one wheel unit on the robot. Each module has TWO motors:
1. DRIVE motor (SparkFlex) - Spins the wheel to move the robot forward/backward
2. TURN motor (SparkMax) - Rotates the wheel left/right to change direction

The magic of swerve drive is that each wheel can point in ANY direction
independently. This lets the robot move in any direction without turning
the whole robot body.

PICTURE OF ONE MODULE:
    ┌─────────┐
    │  TURN   │  ← SparkMax motor rotates the whole module
    │  MOTOR  │
    │    │    │
    │    ▼    │
    │ ┌─────┐ │
    │ │WHEEL│ │  ← 3-inch wheel
    │ │ ◄── │ │  ← SparkFlex motor spins the wheel
    │ └─────┘ │
    └─────────┘

HOW THE ENCODERS WORK:
- The DRIVE encoder is a relative encoder (counts rotations from startup)
  It tells us how far and how fast the wheel has traveled.
- The TURN encoder is an absolute encoder (always knows its exact angle)
  It tells us which direction the wheel is pointing, even after a reboot.

PID CONTROLLERS:
Both motors use PID controllers that run ON the SparkMax/SparkFlex hardware.
This means the motor controller itself does the math to reach the target
speed/position, which is faster and more reliable than doing it on the RoboRIO.
"""

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import rev
from rev import SparkMax, SparkMaxConfig, SparkFlex, SparkFlexConfig, SparkBase
import constants


class SwerveModule:
    """
    Controls a single swerve module (one corner of the robot).

    Each module can independently:
    - Drive at any speed (controlled by the SparkFlex drive motor)
    - Point in any direction (controlled by the SparkMax turn motor)

    This class handles all the low-level motor control, encoder reading,
    and PID setup for one module.
    """

    def __init__(
        self,
        drive_can_id: int,
        turn_can_id: int,
        chassis_angular_offset: float,
        absolute_encoder_offset: float,
    ) -> None:
        """
        Set up one swerve module.

        Parameters:
        -----------
        drive_can_id : int
            The CAN bus ID number for the SparkFlex drive motor controller.
            This is set using the REV Hardware Client.

        turn_can_id : int
            The CAN bus ID number for the SparkMax turning motor controller.

        chassis_angular_offset : float
            How much this module's "forward" direction is rotated compared to
            the robot's forward direction (in radians). This is a fixed property
            of the MAXSwerve module design.

        absolute_encoder_offset : float
            A calibration value for the absolute encoder. Since each module's
            encoder zero point is slightly different, this offset corrects it
            so that "0" means the wheel is pointing forward.
        """

        # =====================================================================
        # STEP 1: Create the motor controller objects
        # =====================================================================
        # SparkFlex for driving (more powerful motor for wheel spinning)
        # SparkMax for turning (smaller motor for steering the module)
        # Both use brushless motors (NEO/NEO Vortex) - more efficient than brushed
        self.driving_spark = SparkFlex(drive_can_id, SparkFlex.MotorType.kBrushless)
        self.turning_spark = SparkMax(turn_can_id, SparkMax.MotorType.kBrushless)

        # =====================================================================
        # STEP 2: Create configuration objects
        # =====================================================================
        # Instead of setting parameters one at a time, we build up a complete
        # configuration and then apply it all at once. This is the newer
        # REV API pattern (2025+).
        self.driving_config = SparkFlexConfig()
        self.turning_config = SparkMaxConfig()

        # =====================================================================
        # STEP 3: Get encoder objects
        # =====================================================================
        # Drive motor uses a RELATIVE encoder (built into the motor)
        # - Counts rotations from when the robot starts up
        # - Needs conversion factors to give us meters instead of rotations
        self.driving_encoder = self.driving_spark.getEncoder()

        # Turn motor uses an ABSOLUTE encoder (knows its position always)
        # - Tells us the exact angle of the module at all times
        # - This is critical because we need to know which way the wheel
        #   is pointing even right after the robot turns on
        self.turning_encoder = self.turning_spark.getAbsoluteEncoder()

        # =====================================================================
        # STEP 4: Configure the DRIVE motor encoder
        # =====================================================================
        # Raw encoder values are in "rotations" and "RPM".
        # We want "meters" and "meters per second" so WPILib can use them.
        #
        # Position factor: Each motor rotation moves the wheel by
        #   (wheel circumference) / (gear ratio) meters
        # Velocity factor: Same thing but per second instead of per minute
        self.driving_config.encoder.positionConversionFactor(
            constants.kDrivingEncoderPositionFactor
        )
        self.driving_config.encoder.velocityConversionFactor(
            constants.kDrivingEncoderVelocityFactor
        )

        # =====================================================================
        # STEP 5: Configure the TURN motor encoder
        # =====================================================================
        # The absolute encoder gives us the angle in rotations (0 to 1).
        # We convert to radians (0 to 2π) for WPILib compatibility.
        self.turning_config.absoluteEncoder.positionConversionFactor(
            constants.kTurningEncoderPositionFactor
        )
        self.turning_config.absoluteEncoder.velocityConversionFactor(
            constants.kTurningEncoderVelocityFactor
        )

        # Set the zero offset for this specific module
        # This corrects for the physical mounting position of the encoder
        self.turning_config.absoluteEncoder.zeroOffset(absolute_encoder_offset)

        # The turning encoder reads backwards because of the gearing,
        # so we invert it to match the actual module direction
        self.turning_config.absoluteEncoder.inverted(
            constants.kTurningEncoderInverted
        )

        # =====================================================================
        # STEP 6: Set up PID controllers
        # =====================================================================
        # PID controllers live ON the motor controller hardware.
        # They continuously adjust motor power to reach the target.
        #
        # "Closed loop" means the controller measures its output (via encoder)
        # and adjusts itself - like a thermostat that checks the temperature
        # and turns the heater on/off to reach the set point.
        self.driving_pid = self.driving_spark.getClosedLoopController()
        self.turning_pid = self.turning_spark.getClosedLoopController()

        # Tell each PID controller which encoder to use for feedback
        # Drive motor uses the built-in encoder (primary encoder)
        self.driving_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        # Turn motor uses the absolute encoder
        self.turning_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kAbsoluteEncoder
        )

        # =====================================================================
        # STEP 7: Enable position wrapping for turning
        # =====================================================================
        # Without wrapping: Going from 350° to 10° would go 340° the wrong way!
        # With wrapping: It goes 20° through 360°/0° - the short way around.
        # This is essential for smooth module rotation.
        self.turning_config.closedLoop.positionWrappingEnabled(True)
        self.turning_config.closedLoop.positionWrappingMinInput(
            constants.kTurningEncoderPositionPIDMinInput
        )
        self.turning_config.closedLoop.positionWrappingMaxInput(
            constants.kTurningEncoderPositionPIDMaxInput
        )

        # =====================================================================
        # STEP 8: Set PID gains for DRIVE motor
        # =====================================================================
        # These numbers control how the motor reaches the desired speed.
        # P=0.04 is gentle - the motor gradually ramps to the target speed.
        # FF (Feed Forward) predicts the needed output based on desired speed.
        self.driving_config.closedLoop.P(constants.kDrivingP)
        self.driving_config.closedLoop.I(constants.kDrivingI)
        self.driving_config.closedLoop.D(constants.kDrivingD)
        self.driving_config.closedLoop.velocityFF(constants.kDrivingFF)
        self.driving_config.closedLoop.outputRange(
            constants.kDrivingMinOutput, constants.kDrivingMaxOutput
        )

        # =====================================================================
        # STEP 9: Set PID gains for TURN motor
        # =====================================================================
        # P=2 is aggressive - the module snaps quickly to the target angle.
        # This is important for responsive driving.
        self.turning_config.closedLoop.P(constants.kTurningP)
        self.turning_config.closedLoop.I(constants.kTurningI)
        self.turning_config.closedLoop.D(constants.kTurningD)
        self.turning_config.closedLoop.velocityFF(constants.kTurningFF)
        self.turning_config.closedLoop.outputRange(
            constants.kTurningMinOutput, constants.kTurningMaxOutput
        )

        # =====================================================================
        # STEP 10: Set motor idle modes and current limits
        # =====================================================================
        # Brake mode: When no power is applied, the motor actively resists movement.
        # This is important so the robot doesn't slide around when you let go of
        # the joystick.
        self.driving_config.setIdleMode(constants.kDrivingMotorIdleMode)
        self.turning_config.setIdleMode(constants.kTurningMotorIdleMode)

        # Current limits protect the motors and prevent tripping circuit breakers.
        # If a motor tries to draw more current than this, it gets throttled.
        self.driving_config.smartCurrentLimit(constants.kDrivingMotorCurrentLimit)
        self.turning_config.smartCurrentLimit(constants.kTurningMotorCurrentLimit)

        # =====================================================================
        # STEP 11: Apply the configurations to the motor controllers
        # =====================================================================
        # kResetSafeParameters: Start from factory defaults before applying
        # kPersistParameters: Save to flash memory so settings survive power loss
        self.driving_spark.configure(
            self.driving_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.turning_spark.configure(
            self.turning_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # =====================================================================
        # STEP 12: Initialize module state
        # =====================================================================
        # Remember the chassis angular offset for later calculations
        self.chassis_angular_offset = chassis_angular_offset

        # Set the initial desired state to "stopped, pointing current direction"
        self.desired_state = wpimath.kinematics.SwerveModuleState(
            0.0,
            wpimath.geometry.Rotation2d(self.turning_encoder.getPosition()),
        )

        # Reset the drive encoder to zero - we start counting distance from now
        self.driving_encoder.setPosition(0)

    def get_state(self) -> wpimath.kinematics.SwerveModuleState:
        """
        Get the current state of this module.

        Returns a SwerveModuleState containing:
        - speed: How fast the wheel is spinning (meters/second)
        - angle: Which direction the wheel is pointing (relative to robot front)

        The chassis angular offset is subtracted so the angle is relative to
        the ROBOT's forward direction, not the module's internal reference.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driving_encoder.getVelocity(),
            wpimath.geometry.Rotation2d(
                self.turning_encoder.getPosition() - self.chassis_angular_offset
            ),
        )

    def get_position(self) -> wpimath.kinematics.SwerveModulePosition:
        """
        Get the current position of this module.

        Returns a SwerveModulePosition containing:
        - distance: Total distance the wheel has traveled (meters)
        - angle: Which direction the wheel is pointing (relative to robot front)

        This is used by odometry to track where the robot is on the field.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driving_encoder.getPosition(),
            wpimath.geometry.Rotation2d(
                self.turning_encoder.getPosition() - self.chassis_angular_offset
            ),
        )

    def set_desired_state(
        self, desired_state: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """
        Tell this module to go to a specific speed and angle.

        Parameters:
        -----------
        desired_state : SwerveModuleState
            Contains the target speed (m/s) and angle (Rotation2d).

        HOW IT WORKS:
        1. Apply the chassis angular offset (convert from robot-relative
           to module-relative coordinates)
        2. Optimize the angle - if turning more than 90°, it's faster to
           reverse the drive direction and turn less
        3. Send the commands to the motor controllers' PID loops
        """
        # Add the chassis offset to convert from robot coordinates to module coordinates
        corrected_state = wpimath.kinematics.SwerveModuleState(
            desired_state.speed,
            desired_state.angle
            + wpimath.geometry.Rotation2d(self.chassis_angular_offset),
        )

        # Get where the module is currently pointing
        current_angle = wpimath.geometry.Rotation2d(
            self.turning_encoder.getPosition()
        )

        # OPTIMIZE: If we need to turn more than 90°, it's faster to turn less
        # and reverse the drive direction.
        # Example: Instead of turning 150° and driving forward, turn 30° and
        # drive backward. Same result, less movement!
        corrected_state.optimize(current_angle)

        # Send the speed command to the drive motor PID (velocity control)
        self.driving_pid.setReference(
            corrected_state.speed, SparkFlex.ControlType.kVelocity
        )

        # Send the angle command to the turn motor PID (position control)
        self.turning_pid.setReference(
            corrected_state.angle.radians(), SparkMax.ControlType.kPosition
        )

        # Remember what we asked for (useful for debugging/logging)
        self.desired_state = desired_state

    def reset_encoders(self) -> None:
        """
        Reset the drive encoder to zero.

        This is called when we want to start counting distance from scratch.
        The turn encoder is ABSOLUTE so it doesn't need resetting.
        """
        self.driving_encoder.setPosition(0)
