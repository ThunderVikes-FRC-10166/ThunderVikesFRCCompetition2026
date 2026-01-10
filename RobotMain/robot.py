import magicbot
import wpilib

# ------------------------------------------------------------------------------
#  MagicBot Overview
# ------------------------------------------------------------------------------
# MagicBot is a framework built on top of WPILib that helps FRC teams organize
# their robot code in a clean, modular way. Instead of putting all logic inside
# one giant file, MagicBot encourages you to break your robot into "components"
# (like DriveTrain, Shooter, Intake, etc.) and "controllers" (logic that decides
# what those components should do).
#
# This file defines the *main robot program* that the roboRIO runs. MagicBot
# calls specific methods at specific times during the match:
#
#   - createObjects():   Runs once when the robot starts up
#   - teleopInit():      Runs once when Teleop mode begins
#   - teleopPeriodic():  Runs repeatedly (50 times per second) during Teleop
#
# Students will fill these methods with real robot code as the robot develops.
# ------------------------------------------------------------------------------


class MyRobot(magicbot.MagicRobot):
    """
    This class represents your entire robot program.

    MagicBot automatically handles:
      - Running your code at the correct times
      - Managing components (subsystems)
      - Ensuring periodic functions run at 50Hz
      - Organizing your robot logic cleanly

    You will expand this class as your robot gains motors, sensors, and features.
    """

    # --------------------------------------------------------------------------
    # createObjects()
    # --------------------------------------------------------------------------
    def createObjects(self):
        """
        This method is called ONCE when the robot boots.

        Use this method to:
          - Create motor controller objects (Talon, Spark, Victor, etc.)
          - Create sensors (Encoders, Gyros, Limit Switches)
          - Create joysticks or Xbox controllers
          - Create pneumatics objects (Solenoids, Compressors)
          - Instantiate MagicBot components (DriveTrain, Shooter, etc.)

        IMPORTANT:
        - Do NOT put robot logic here.
        - Only create hardware objects and assign them to variables.
        - MagicBot will automatically inject these into your components.

        Example of what this might look like later:

            self.left_motor = wpilib.Talon(1)
            self.right_motor = wpilib.Talon(2)
            self.driver_controller = wpilib.XboxController(0)

        For now, it's empty because we haven't defined hardware yet.
        """
        pass


    # --------------------------------------------------------------------------
    # teleopInit()
    # --------------------------------------------------------------------------
    def teleopInit(self):
        """
        This method runs ONCE when Teleop mode begins.

        Use this for:
          - Resetting sensors (encoders, gyros)
          - Zeroing state variables
          - Preparing subsystems for driver control
          - Printing debug info to the console

        Example of what this might look like later:

            self.drive.resetEncoders()
            print("Teleop has started!")

        This method is optional — if you don't need it, you can leave it empty.
        """
        pass


    # --------------------------------------------------------------------------
    # teleopPeriodic()
    # --------------------------------------------------------------------------
    def teleopPeriodic(self):
        """
        This method runs REPEATEDLY during Teleop (about 50 times per second).

        This is where you:
          - Read driver inputs (joysticks, Xbox controller)
          - Tell your components what to do
          - Implement driver controls (driving, shooting, intaking, etc.)
          - Run logic that needs to update continuously

        IMPORTANT:
        - Do NOT directly control motors here if you're using MagicBot components.
          Instead, call methods on your components, and let MagicBot handle the
          actual motor outputs.

        Example of what this might look like later:

            forward = -self.driver_controller.getLeftY()
            turn = self.driver_controller.getRightX()
            self.drive.arcade_drive(forward, turn)

        For now, this is empty until hardware and components are added.
        """
        pass
