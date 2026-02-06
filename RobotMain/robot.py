import magicbot
import wpilib
import math
from ntcore import NetworkTableInstance
import rev
# from RobotMain.samples.samplecomponentusage import MyRobot
from components.swerve_drive_real import SwerveDrive
from components.swerve_drive_module import SwerveModule
# ----------------------
# --------------------------------------------------------
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
def deadband(x: float, db: float = 0.08) -> float:
    return 0.0 if abs(x) < db else x




class MyRobot(magicbot.MagicRobot):
    swerve: SwerveDrive
    """
    This class represents your entire robot program.

    MagicBot automatically handles:
      - Running your code at the correct times
      - Managing components (subsystems)
      - Ensuring periodic functions run at 50Hz
      - Organizing your robot logic cleanly

    You will expand this class as your robot gains motors, sensors, and features.
    """
    def robotInit(self):
        super().robotInit()
        self._attach_robot_to_auto_modes()

    def _attach_robot_to_auto_modes(self):
        """
        Give each autonomous mode a reference to this robot instance.
        This allows autonomous code to call:
            self.robot.swerve.drive(...)
        """
        selector = getattr(self, "_automodes", None)
        if selector is None:
            selector = getattr(self, "autonomous", None)

        if selector is None:
            wpilib.reportWarning("No autonomous selector found; cannot attach robot to auto modes")
            return

        modes = getattr(selector, "modes", None)
        if not isinstance(modes, dict) or not modes:
            wpilib.reportWarning("Autonomous selector has no modes (did you create autonomous/__init__.py?")
            return

        for mode in modes.values():
            mode.robot = self

        wpilib.reportWarning(f"Attached robot reference to {len(modes)} autonomous mode(s)")
    # --------------------------------------------------------------------------
    # createObjects()
    # --------------------------------------------------------------------------
    def createObjects(self):
        self.driver = wpilib.XboxController(0)
        self.ll = NetworkTableInstance.getDefault().getTable("limelight")

        #----------------------------------
        #Driver Scaling constants
        #----------------------------------
        #These are "how fast should the robot be allowed to go"
        # Students can tune these later
        self.max_speed_mps = 4.5 # typical swerve top speed
        self.max_omega_radps = 2.5 * math.pi  # about 1.25 rotations/sec

        #----------------------------
        # CREATE HARDWARE for ALL 4 modules
        #----------------------------
        # Replace CAN IDs and DIO channels with YOUR real
        #
        # CAN IDs (example):
        # drive motors: 1,2,3,4
        # turn motors: 5,6,7,8
        #
        # DIO ports (example):
        #  abs encoders: 0,1,2,3
        #
        # IMPORTANT: Spark MAX motor type for NEO/NEO550 is Brushless.
        motor_type = rev.SparkLowLevel.MotorType.KBrushless

        # ----- Front left (fl) ---
        self.fl_drive_motor = rev.SparkMax(20, motor_type)
        self.fl_turn_motor  = rev.SparkMax(21, motor_type)
        self.fl_abs_encoder = wpilib.DutyCycleEncoder(0)

        # ----- Front Right (Fr) ------
        self.fr_drive_motor = rev.SparkMax(17, motor_type)
        self.fr_turn_motor  = rev.SparkMax(23, motor_type)
        self.fr_abs_encoder = wpilib.DutyCycleEncoder(1)

        # ------ Back Left (bl) ---
        self.bl_drive_motor = rev.SparkMax(18, motor_type)
        self.bl_turn_motor  = rev.SparkMax(22, motor_type)
        self.bl_abs_encoder = wpilib.DutyCycleEncoder(2)

        # ---- Back Right (br) ---
        self.br_drive_motor = rev.SparkMax(19, motor_type)
        self.br_turn_motor = rev.SparkMax(24, motor_type)
        self.br_abs_encoder = wpilib.DutyCycleEncoder(3)




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

        x = -self.driver.getLeftY() #Forward/back
        y = -self.driver.getLeftX() # strafe
        rot = -self.driver.getRightX() #rotate
        x = deadband(x)
        y = deadband(y)
        rot = deadband(rot)

        x = math.copysign(x * x, x)
        y = math.copysign(y * y, y)
        rot = math.copysign(rot * rot, rot)
        vx = x * self.swerve.max_speed_mps
        vy = y * self.swerve.max_speed_mps
        omega = rot * self.swerve.max_omega_radps

        self.swerve.drive(vx, vy, omega)

        tv = self.ll.getNumber("tv", 0)
        if tv >= 1:
            tid = int(self.ll.getNumber("tid", -1))
            tx = self.ll.getNumber("tx", 0.0)
            botpose = self.ll.getNumberArray("botpose_wpiblue", [])
            print(f"[robot] sees tag tid={tid} tx={tx:.1f} botpose={botpose}")

if __name__ == "__main__":
    wpilib.run(MyRobot)

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
