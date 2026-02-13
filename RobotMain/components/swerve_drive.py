"""
swerve_drive.py - Swerve Drive System (MagicBot Component)
============================================================

WHAT IS SWERVE DRIVE?
Swerve drive is a type of robot drivetrain where each wheel can independently
rotate and drive. This gives the robot the ability to move in ANY direction
while also rotating - all at the same time!

Imagine you're on a shopping cart where each wheel can spin to point any
direction. You could push the cart sideways, diagonally, or spin it in
circles - that's swerve drive!

HOW IT ALL FITS TOGETHER:
1. The DRIVER pushes the joystick (e.g., forward)
2. robot.py reads the joystick and calls set_drive_command()
3. This component calculates what each of the 4 modules needs to do
4. It sends commands to each SwerveModule
5. Each module's motors make the wheels spin and turn
6. The robot moves!

FIELD-ORIENTED DRIVE:
When field_relative=True, pushing the joystick "up" ALWAYS moves the robot
toward the far end of the field, regardless of which way the robot is facing.
This is done using the gyroscope to know which way the robot is pointing.

Without field-oriented: Push up → robot drives wherever it's facing (confusing!)
With field-oriented: Push up → robot always drives "north" on the field (intuitive!)

MAGICBOT COMPONENT:
This file uses the MagicBot component pattern:
- setup() is called once when the robot starts
- execute() is called every 20ms (50 times per second) during operation
- The robot.py file sets what we want to do, execute() makes it happen

ODOMETRY:
The robot tracks its position on the field using "odometry" - combining
wheel encoder data with gyroscope data to calculate where the robot is.
Think of it like your car's odometer, but in 2D (tracking X, Y, and angle).
"""

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import wpimath.filter
import wpimath.units
import navx
import ntcore

from magicbot import will_reset_to
from components.swerve_module import SwerveModule
import constants


class SwerveDrive:
    """
    MagicBot component that controls the entire swerve drive system.

    This is the "brain" of the drivetrain. It takes high-level commands
    like "drive forward at 2 m/s" and figures out what each of the 4
    swerve modules needs to do.

    HOW MAGICBOT COMPONENTS WORK:
    - Variables set with will_reset_to() automatically reset each cycle
    - This means if nothing calls set_drive_command(), the robot stops
    - This is a safety feature! If the code crashes, the robot stops.
    """

    # =========================================================================
    # MAGICBOT STATE VARIABLES
    # =========================================================================
    # will_reset_to() means these values reset to the given default EVERY cycle
    # (every 20ms). If nobody sets new values, the robot stays stopped.
    # This is a SAFETY FEATURE - if something goes wrong, the robot stops!

    x_speed = will_reset_to(0.0)      # Forward/backward speed (-1 to 1)
    y_speed = will_reset_to(0.0)      # Left/right speed (-1 to 1)
    rot_speed = will_reset_to(0.0)    # Rotation speed (-1 to 1)
    field_relative = will_reset_to(True)  # Use field-oriented drive by default
    rate_limit = will_reset_to(True)      # Enable acceleration smoothing by default

    def setup(self) -> None:
        """
        Called once when the robot starts up.

        This creates all the hardware objects:
        - 4 swerve modules (one for each corner)
        - Gyroscope (for knowing which way the robot faces)
        - Kinematics (math for converting robot motion to wheel motion)
        - Odometry (tracking robot position on the field)
        """

        # =====================================================================
        # KINEMATICS SETUP
        # =====================================================================
        # SwerveDrive4Kinematics knows the math to convert between:
        #   "robot wants to go forward at 2 m/s"
        #   → "front-left wheel: speed=X, angle=Y" (for each module)
        #
        # We tell it where each module is relative to the robot center.
        # Translation2d(x, y) where:
        #   x = forward distance from center (positive = front)
        #   y = left distance from center (positive = left)
        #
        # Our robot is a 60cm × 60cm square, so each module is 30cm from center.
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            # Front Left: 30cm forward, 30cm left
            wpimath.geometry.Translation2d(
                constants.kWheelBase / 2, constants.kTrackWidth / 2
            ),
            # Front Right: 30cm forward, 30cm right (negative Y)
            wpimath.geometry.Translation2d(
                constants.kWheelBase / 2, -constants.kTrackWidth / 2
            ),
            # Rear Left: 30cm backward (negative X), 30cm left
            wpimath.geometry.Translation2d(
                -constants.kWheelBase / 2, constants.kTrackWidth / 2
            ),
            # Rear Right: 30cm backward, 30cm right
            wpimath.geometry.Translation2d(
                -constants.kWheelBase / 2, -constants.kTrackWidth / 2
            ),
        )

        # =====================================================================
        # CREATE THE 4 SWERVE MODULES
        # =====================================================================
        # Each module gets its CAN IDs, angular offset, and encoder offset.
        # See constants.py for what these numbers mean.

        self.front_left = SwerveModule(
            constants.kFrontLeftDrivingCanId,
            constants.kFrontLeftTurningCanId,
            constants.kFrontLeftChassisAngularOffset,
            constants.kFrontLeftAbsoluteEncoderOffset,
        )

        self.front_right = SwerveModule(
            constants.kFrontRightDrivingCanId,
            constants.kFrontRightTurningCanId,
            constants.kFrontRightChassisAngularOffset,
            constants.kFrontRightAbsoluteEncoderOffset,
        )

        self.rear_left = SwerveModule(
            constants.kRearLeftDrivingCanId,
            constants.kRearLeftTurningCanId,
            constants.kRearLeftChassisAngularOffset,
            constants.kRearLeftAbsoluteEncoderOffset,
        )

        self.rear_right = SwerveModule(
            constants.kRearRightDrivingCanId,
            constants.kRearRightTurningCanId,
            constants.kRearRightChassisAngularOffset,
            constants.kRearRightAbsoluteEncoderOffset,
        )

        # =====================================================================
        # GYROSCOPE SETUP
        # =====================================================================
        # The NavX gyroscope tells us which direction the robot is facing.
        # This is ESSENTIAL for field-oriented drive.
        # It connects via the MXP port on the RoboRIO using SPI.
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        # Zero the gyro on startup so the robot's current forward direction
        # is treated as "0 degrees". This way field-oriented drive works
        # correctly from the moment the robot turns on - pushing the joystick
        # forward drives the robot in whichever direction it's facing at boot.
        # The driver can still re-zero it later with the Start button.
        self.gyro.reset()

        # =====================================================================
        # SLEW RATE LIMITERS (Acceleration Smoothing)
        # =====================================================================
        # These prevent sudden changes in speed/direction.
        # Like a car's gas pedal - you can't go from 0 to 60 instantly.
        self.current_rotation = 0.0
        self.current_translation_dir = 0.0
        self.current_translation_mag = 0.0

        self.mag_limiter = wpimath.filter.SlewRateLimiter(
            constants.kMagnitudeSlewRate
        )
        self.rot_limiter = wpimath.filter.SlewRateLimiter(
            constants.kRotationalSlewRate
        )
        self.prev_time = wpilib.Timer.getFPGATimestamp()

        # =====================================================================
        # ODOMETRY SETUP
        # =====================================================================
        # Odometry tracks where the robot is on the field.
        # It combines gyroscope heading + wheel encoder distances to
        # continuously update the robot's estimated position.
        #
        # This starts at position (0, 0) facing "forward".
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.rear_left.get_position(),
                self.rear_right.get_position(),
            ),
            wpimath.geometry.Pose2d(),
        )

        # =====================================================================
        # LIMELIGHT CAMERA (STUB)
        # =====================================================================
        # The Limelight camera will be used later for vision-based positioning.
        # For now, we just have placeholder code.
        # When connected, it will help correct our odometry position using
        # AprilTag targets on the field.
        self.limelight_connected = False
        self.limelight_pose = None  # Will be a Pose2d when available

        # Reset drive encoders to zero
        self.reset_encoders()

    def set_drive_command(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
        rate_limit: bool = True,
    ) -> None:
        """
        Tell the swerve drive what to do this cycle.

        This is the main method that robot.py calls to control the drive.

        Parameters:
        -----------
        x_speed : float
            Forward/backward speed. Positive = forward.
            Range: -1.0 to 1.0 (percentage of max speed)

        y_speed : float
            Left/right (strafe) speed. Positive = left.
            Range: -1.0 to 1.0 (percentage of max speed)

        rot : float
            Rotation speed. Positive = counter-clockwise.
            Range: -1.0 to 1.0 (percentage of max angular speed)

        field_relative : bool
            True = Field-oriented drive (recommended!)
            False = Robot-oriented drive

        rate_limit : bool
            True = Smooth acceleration (default)
            False = Instant response (can be jerky)
        """
        self.x_speed = x_speed
        self.y_speed = y_speed
        self.rot_speed = rot
        self.field_relative = field_relative
        self.rate_limit = rate_limit

    def execute(self) -> None:
        """
        Called every 20ms by MagicBot.

        This is where the actual driving happens. It:
        1. Updates odometry (where is the robot?)
        2. Applies rate limiting (smooth acceleration)
        3. Converts joystick commands to wheel commands
        4. Sends commands to each swerve module

        IMPORTANT: This runs 50 times per second! Keep it fast!
        """

        # =====================================================================
        # STEP 1: Update Odometry
        # =====================================================================
        # Every cycle, we update our estimate of where the robot is.
        # This combines:
        # - Gyroscope: Which way are we facing?
        # - Wheel encoders: How far has each wheel moved since last cycle?
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.rear_left.get_position(),
                self.rear_right.get_position(),
            ),
        )

        # =====================================================================
        # STEP 2: Limelight Vision Update (STUB)
        # =====================================================================
        # When the Limelight is connected, we'll use it to correct our
        # odometry position. AprilTags on the field have known positions,
        # so if we can see one, we know exactly where we are.
        self._update_limelight()

        # =====================================================================
        # STEP 3: Apply Rate Limiting
        # =====================================================================
        # Rate limiting smooths out sudden joystick movements.
        # Without it, slamming the joystick would jolt the robot.
        x_speed_commanded = self.x_speed
        y_speed_commanded = self.y_speed
        current_rotation = self.rot_speed

        if self.rate_limit:
            # Convert X/Y speeds to polar coordinates (direction + magnitude)
            # This lets us limit the TOTAL speed and direction change independently
            input_translation_dir = math.atan2(self.y_speed, self.x_speed)
            input_translation_mag = math.sqrt(
                self.x_speed ** 2 + self.y_speed ** 2
            )

            # Calculate how fast we're allowed to change direction
            # When moving fast, direction changes more slowly (safer!)
            if self.current_translation_mag != 0.0:
                direction_slew_rate = abs(
                    constants.kDirectionSlewRate / self.current_translation_mag
                )
            else:
                # When stopped, we can change direction instantly
                direction_slew_rate = 500.0

            current_time = wpilib.Timer.getFPGATimestamp()
            elapsed_time = current_time - self.prev_time
            angle_diff = _angle_difference(
                input_translation_dir, self.current_translation_dir
            )

            # Apply the appropriate rate limiting based on how much we're
            # changing direction:
            if angle_diff < 0.45 * math.pi:
                # Small direction change: smoothly follow the input
                self.current_translation_dir = _step_towards_circular(
                    self.current_translation_dir,
                    input_translation_dir,
                    direction_slew_rate * elapsed_time,
                )
                self.current_translation_mag = self.mag_limiter.calculate(
                    input_translation_mag
                )
            elif angle_diff > 0.85 * math.pi:
                # Near-reversal: slow down first, then change direction
                if self.current_translation_mag > 1e-4:
                    self.current_translation_mag = self.mag_limiter.calculate(0.0)
                else:
                    self.current_translation_dir = _wrap_angle(
                        self.current_translation_dir + math.pi
                    )
                    self.current_translation_mag = self.mag_limiter.calculate(
                        input_translation_mag
                    )
            else:
                # Medium direction change: change direction but slow down
                self.current_translation_dir = _step_towards_circular(
                    self.current_translation_dir,
                    input_translation_dir,
                    direction_slew_rate * elapsed_time,
                )
                self.current_translation_mag = self.mag_limiter.calculate(0.0)

            self.prev_time = current_time

            # Convert back from polar to X/Y
            x_speed_commanded = self.current_translation_mag * math.cos(
                self.current_translation_dir
            )
            y_speed_commanded = self.current_translation_mag * math.sin(
                self.current_translation_dir
            )
            current_rotation = self.rot_limiter.calculate(self.rot_speed)

        # =====================================================================
        # STEP 4: Convert to actual speeds
        # =====================================================================
        # The joystick gives us -1 to 1 values.
        # We multiply by max speed to get actual m/s and rad/s values.
        x_speed_delivered = x_speed_commanded * constants.kMaxSpeed
        y_speed_delivered = y_speed_commanded * constants.kMaxSpeed
        rot_delivered = current_rotation * constants.kMaxAngularSpeed

        # =====================================================================
        # STEP 5: Calculate what each module needs to do
        # =====================================================================
        # The kinematics object does the heavy math:
        # "The robot wants to go X m/s forward, Y m/s sideways, and rotate R rad/s"
        # → "Front-left module: drive at A m/s and point at B degrees"
        # (repeated for all 4 modules)

        if self.field_relative:
            # FIELD-ORIENTED: Convert from field coordinates to robot coordinates
            # using the gyroscope heading. This way, "forward" on the joystick
            # always means "toward the far end of the field".
            chassis_speeds = (
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    x_speed_delivered,
                    y_speed_delivered,
                    rot_delivered,
                    self.gyro.getRotation2d(),
                )
            )
        else:
            # ROBOT-ORIENTED: "Forward" means "wherever the robot is facing"
            chassis_speeds = wpimath.kinematics.ChassisSpeeds(
                x_speed_delivered, y_speed_delivered, rot_delivered
            )

        # Convert chassis speeds to individual module states
        fl, fr, rl, rr = self.kinematics.toSwerveModuleStates(chassis_speeds)

        # =====================================================================
        # STEP 6: Desaturate wheel speeds
        # =====================================================================
        # When translating AND rotating at the same time, individual modules
        # can be asked to go faster than physically possible. Desaturation
        # scales ALL module speeds down proportionally so no module exceeds
        # the max speed, while keeping the correct speed RATIOS between modules.
        # Without this, the robot would drive crooked during combined moves!
        fl, fr, rl, rr = (
            wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
                (fl, fr, rl, rr), constants.kMaxSpeed
            )
        )

        # =====================================================================
        # STEP 7: Send commands to modules
        # =====================================================================
        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.rear_left.set_desired_state(rl)
        self.rear_right.set_desired_state(rr)

    def set_x_formation(self) -> None:
        """
        Set all modules to point inward in an X pattern.

        This makes the robot nearly impossible to push because all wheels
        are pointing toward the center. Use this when you need to hold
        position (like when parked on a charging station).

             ╲    ╱
              ╲  ╱
               ╳
              ╱  ╲
             ╱    ╲
        """
        self.front_left.set_desired_state(
            wpimath.kinematics.SwerveModuleState(
                0, wpimath.geometry.Rotation2d.fromDegrees(45)
            )
        )
        self.front_right.set_desired_state(
            wpimath.kinematics.SwerveModuleState(
                0, wpimath.geometry.Rotation2d.fromDegrees(-45)
            )
        )
        self.rear_left.set_desired_state(
            wpimath.kinematics.SwerveModuleState(
                0, wpimath.geometry.Rotation2d.fromDegrees(-45)
            )
        )
        self.rear_right.set_desired_state(
            wpimath.kinematics.SwerveModuleState(
                0, wpimath.geometry.Rotation2d.fromDegrees(45)
            )
        )

    def get_heading(self) -> float:
        """Get the robot's heading in degrees (-180 to 180)."""
        return self.gyro.getRotation2d().degrees()

    def zero_heading(self) -> None:
        """
        Reset the gyroscope heading to zero.

        Call this when the robot is facing the direction you want to be
        "forward" for field-oriented drive. Usually done at the start
        of a match.
        """
        self.gyro.reset()

    def get_pose(self) -> wpimath.geometry.Pose2d:
        """
        Get the robot's estimated position on the field.

        Returns a Pose2d with x, y coordinates (in meters) and rotation angle.
        """
        return self.odometry.getPose()

    def reset_odometry(self, pose: wpimath.geometry.Pose2d) -> None:
        """
        Reset the odometry to a specific position.

        Use this at the start of autonomous when you know exactly where
        the robot is on the field.
        """
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.front_left.get_position(),
                self.front_right.get_position(),
                self.rear_left.get_position(),
                self.rear_right.get_position(),
            ),
            pose,
        )

    def reset_encoders(self) -> None:
        """Reset all drive encoders to zero."""
        self.front_left.reset_encoders()
        self.front_right.reset_encoders()
        self.rear_left.reset_encoders()
        self.rear_right.reset_encoders()

    def get_turn_rate(self) -> float:
        """Get the robot's turn rate in degrees per second."""
        return -self.gyro.getRate()

    def _update_limelight(self) -> None:
        """
        Update robot position using Limelight camera data.

        STUB: This will be implemented when the Limelight is connected.

        When working, this will:
        1. Read pose data from the Limelight NetworkTables
        2. Check if the data is trustworthy (is it seeing AprilTags?)
        3. If good, blend it with our odometry for a more accurate position
        """
        # TODO: Implement Limelight vision processing
        # Example of what this will look like:
        #
        # table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        # tv = table.getNumber("tv", 0)  # 1 if target visible
        # if tv == 1:
        #     bot_pose = table.getNumberArray("botpose", [0]*6)
        #     x, y, z, roll, pitch, yaw = bot_pose
        #     vision_pose = wpimath.geometry.Pose2d(x, y, wpimath.geometry.Rotation2d.fromDegrees(yaw))
        #     # Use this to correct odometry
        #     self.limelight_pose = vision_pose
        #     self.limelight_connected = True
        pass


# =============================================================================
# HELPER FUNCTIONS (Swerve Utils)
# =============================================================================
# These functions help with angle math. Angles are tricky because they
# "wrap around" - 360° is the same as 0°, and -10° is the same as 350°.


def _step_towards(current: float, target: float, step_size: float) -> float:
    """
    Move a value toward a target by at most step_size.

    Like a volume knob that can only turn so fast - if you want volume 10
    but you're at 3, with step_size 2, you'd go to 5 this cycle.
    """
    if abs(current - target) <= step_size:
        return target
    elif target < current:
        return current - step_size
    else:
        return current + step_size


def _step_towards_circular(
    current: float, target: float, step_size: float
) -> float:
    """
    Same as step_towards but for angles (handles wrapping around 360°/0°).

    Takes the SHORTEST path around the circle to reach the target.
    """
    current = _wrap_angle(current)
    target = _wrap_angle(target)

    difference = abs(current - target)

    if difference <= step_size:
        return target

    if target > current:
        step_direction = 1
    elif target < current:
        step_direction = -1
    else:
        return target

    if difference > math.pi:
        # It's shorter to go the other way around
        if (
            current + 2 * math.pi - target < step_size
            or target + 2 * math.pi - current < step_size
        ):
            return target
        else:
            return _wrap_angle(current - step_direction * step_size)
    else:
        return current + step_direction * step_size


def _angle_difference(angle_a: float, angle_b: float) -> float:
    """
    Find the smallest angle between two angles.

    For example: The difference between 10° and 350° is 20° (not 340°).
    """
    difference = abs(angle_a - angle_b)
    if difference > math.pi:
        return (2 * math.pi) - difference
    return difference


def _wrap_angle(angle: float) -> float:
    """
    Wrap an angle to be between 0 and 2π (0° and 360°).

    Examples: -10° → 350°, 370° → 10°
    """
    two_pi = 2 * math.pi

    if angle == two_pi:
        return 0.0
    elif angle > two_pi:
        return angle - two_pi * math.floor(angle / two_pi)
    elif angle < 0.0:
        return angle + two_pi * (math.floor((-angle) / two_pi) + 1)
    else:
        return angle
