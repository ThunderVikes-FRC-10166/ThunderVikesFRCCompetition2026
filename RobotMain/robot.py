"""
robot.py - Main Robot Program
================================

This is the ENTRY POINT for the robot code - the first file that runs.
Think of it like the "main menu" of a video game.

WE USE MAGICBOT FRAMEWORK:
MagicBot is a framework built on top of WPILib that makes robot code easier
to organize. Here's what it does for us:

1. AUTOMATIC COMPONENT MANAGEMENT:
   - We create "components" (like SwerveDrive) as class variables
   - MagicBot automatically calls their setup() and execute() methods
   - Components reset to safe values every cycle (safety feature!)

2. AUTOMATIC AUTONOMOUS SUPPORT:
   - Put autonomous mode files in the 'autonomous/' folder
   - MagicBot finds them and lets you pick from the Driver Station

3. CLEAR STRUCTURE:
   - createObjects(): Set up hardware and create components
   - teleopPeriodic(): Read driver inputs and tell components what to do
   - Components do the actual work in their execute() methods

CONTROL SCHEME:
==============
LEFT JOYSTICK:
  - Up/Down → Drive forward/backward (field-relative)
  - Left/Right → Strafe left/right (field-relative)

RIGHT JOYSTICK:
  - Left/Right → Rotate the robot

D-PAD (Direction Pad):
  - Up → Drive forward at 1 m/s
  - Down → Drive backward at 1 m/s
  - Left → Strafe left at 1 m/s
  - Right → Strafe right at 1 m/s
  (These are FIELD-RELATIVE so "up" always goes toward the far end)

BUTTONS:
  - Start button → Reset the gyroscope heading (re-calibrate "forward")
  - X button → Lock wheels in X formation (resist being pushed)
"""

import wpilib
import wpimath
import wpimath.filter
import magicbot

from components.swerve_drive import SwerveDrive
import constants


class SwerveRobot(magicbot.MagicRobot):
    """
    Our FRC robot using swerve drive with the MagicBot framework.

    HOW MAGICBOT WORKS:
    1. We declare components as class-level annotations (type hints)
    2. MagicBot automatically creates them and calls their setup()
    3. During operation, MagicBot calls execute() on each component every 20ms
    4. Our job is to read inputs in teleopPeriodic() and set commands on components
    """

    # =========================================================================
    # COMPONENT DECLARATIONS
    # =========================================================================
    # By declaring these here, MagicBot will:
    # 1. Create an instance of each class
    # 2. Call their setup() method
    # 3. Automatically call their execute() method every 20ms
    #
    # The variable names MATTER - they're used for injection into autonomous modes.
    # If an autonomous mode has a variable called 'swerve_drive', MagicBot will
    # automatically give it our SwerveDrive instance.

    swerve_drive: SwerveDrive

    def createObjects(self) -> None:
        """
        Called when the robot first starts up.

        This is where we create all our input devices (joysticks, controllers)
        and any other hardware that isn't part of a component.

        IMPORTANT: Don't create component objects here! MagicBot handles that.
        Just create input devices and other non-component objects.
        """

        # =====================================================================
        # DRIVER CONTROLLER
        # =====================================================================
        # Xbox controller connected to the Driver Station on port 0.
        # The driver uses this to control the robot's movement.
        self.driver_controller = wpilib.XboxController(
            constants.kDriverControllerPort
        )

        # =====================================================================
        # SLEW RATE LIMITERS
        # =====================================================================
        # These smooth out the joystick inputs so the robot doesn't jerk around.
        # The number (3) means the value can change by 3 units per second max.
        # So going from 0 to 1 takes about 1/3 of a second.
        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)

    def teleopInit(self) -> None:
        """
        Called once when teleop mode starts.

        Teleop is the period of the match when the driver controls the robot
        (after autonomous ends). This is a good place to reset things.
        """
        pass

    def teleopPeriodic(self) -> None:
        """
        Called every 20ms during teleop mode.

        This is the main driver control loop. We:
        1. Read the joystick/d-pad inputs
        2. Apply deadbands (ignore tiny accidental movements)
        3. Tell the swerve drive what to do

        The swerve drive component's execute() will then make it happen.
        """

        # =====================================================================
        # CHECK FOR SPECIAL BUTTONS
        # =====================================================================

        # START button: Reset the gyroscope heading
        # Press this when the robot is facing the direction you want to be "forward"
        if self.driver_controller.getStartButtonPressed():
            self.swerve_drive.zero_heading()

        # X button (held): Lock wheels in X formation
        # This makes the robot really hard to push around
        if self.driver_controller.getXButton():
            self.swerve_drive.set_x_formation()
            return  # Skip normal driving when X is held

        # =====================================================================
        # D-PAD CONTROL
        # =====================================================================
        # The D-pad (POV hat) moves the robot at exactly 1 m/s in the
        # pressed direction. This is great for precise positioning!
        #
        # getPOV() returns the angle in degrees:
        #   0 = up, 90 = right, 180 = down, 270 = left, -1 = not pressed
        pov = self.driver_controller.getPOV()

        if pov != -1:
            # D-pad is pressed! Calculate the X and Y speeds.
            # We use the D-pad speed from constants (1 m/s).
            # Since this is field-relative, "up" on the D-pad always moves
            # the robot toward the far end of the field.

            # Convert POV angle to X/Y components
            # Note: POV 0 = forward (positive X), 90 = right (negative Y in WPILib)
            import math
            pov_rad = math.radians(pov)

            # Forward/backward component (positive = forward)
            # cos(0°) = 1 (forward), cos(180°) = -1 (backward)
            dpad_x = math.cos(pov_rad) * (constants.kDpadSpeed / constants.kMaxSpeed)

            # Left/right component (positive = left in WPILib convention)
            # We negate sin because POV 90° = right, but WPILib positive Y = left
            dpad_y = -math.sin(pov_rad) * (constants.kDpadSpeed / constants.kMaxSpeed)

            self.swerve_drive.set_drive_command(
                dpad_x,
                dpad_y,
                0.0,  # No rotation from D-pad
                True,  # Always field-relative for D-pad
                False,  # No rate limiting for D-pad (immediate response)
            )
            return  # Skip joystick input when D-pad is active

        # =====================================================================
        # JOYSTICK CONTROL
        # =====================================================================
        # Read the joystick axes and convert them to drive commands.
        #
        # Xbox controller axes:
        # Left stick Y (getLeftY): Forward/backward, inverted (push up = negative)
        # Left stick X (getLeftX): Left/right, inverted (push left = negative)
        # Right stick X (getRightX): Rotation, inverted (push left = negative)
        #
        # We INVERT the values because:
        # - Xbox: pushing stick forward gives NEGATIVE values
        # - WPILib: forward is POSITIVE
        # So we negate to match WPILib conventions.

        # Apply deadband first, then slew rate limiting
        # Deadband: Ignore joystick values below 0.08 (prevents drift from
        # a joystick that doesn't perfectly center at 0)

        # Forward/backward speed
        x_speed = -self.x_speed_limiter.calculate(
            wpimath.applyDeadband(
                self.driver_controller.getLeftY(), constants.kDriveDeadband
            )
        )

        # Left/right (strafe) speed
        y_speed = -self.y_speed_limiter.calculate(
            wpimath.applyDeadband(
                self.driver_controller.getLeftX(), constants.kDriveDeadband
            )
        )

        # Rotation speed
        rot = -self.rot_limiter.calculate(
            wpimath.applyDeadband(
                self.driver_controller.getRightX(), constants.kDriveDeadband
            )
        )

        # Send the drive command to the swerve drive component
        # field_relative=True means "up on joystick = toward far end of field"
        self.swerve_drive.set_drive_command(
            x_speed, y_speed, rot, True, True
        )
        # wpilib.SmartDashboard.putNumber("bob", x_speed)

    def autonomousInit(self) -> None:
        """Called once when autonomous mode starts."""
        # MagicBot handles autonomous mode selection automatically!
        # Just put autonomous mode files in the 'autonomous/' folder.
        pass

    def testInit(self) -> None:
        """Called once when test mode starts."""
        pass

    def testPeriodic(self) -> None:
        """Called every 20ms during test mode."""
        pass


# =============================================================================
# PROGRAM ENTRY POINT
# =============================================================================
# This is what actually starts the robot code.
# When you deploy to the RoboRIO or run in simulation, this line kicks
# everything off.

if __name__ == "__main__":
    SwerveRobot.main()
