import magicbot
import wpilib
import math
from ntcore import NetworkTableInstance
import rev

from components.swerve_drive_module import SwerveModule
from components.swerve_drive_real import SwerveDrive


def deadband(x: float, db: float = 0.08) -> float:
    return 0.0 if abs(x) < db else x


class MyRobot(magicbot.MagicRobot):
    # Module components
    fl: SwerveModule
    fr: SwerveModule
    bl: SwerveModule
    br: SwerveModule

    # Drive component
    swerve: SwerveDrive

    def createObjects(self):
        self.driver = wpilib.XboxController(0)
        self.ll = NetworkTableInstance.getDefault().getTable("limelight")

        # Driver scaling constants (swerve also has defaults, but keeping your vibe here)
        self.max_speed_mps = 4.5
        self.max_omega_radps = 2.5 * math.pi

        # ----------------------------------
        # CREATE HARDWARE for ALL 4 modules
        # ----------------------------------
        motor_type = rev.SparkLowLevel.MotorType.kBrushless

        # ----- Front left (fl) ---
        self.fl_drive_motor = rev.SparkFlex(20, motor_type)
        self.fl_turn_motor  = rev.SparkMax(21, motor_type)

        # ----- Front Right (fr) ---
        self.fr_drive_motor = rev.SparkFlex(17, motor_type)
        self.fr_turn_motor  = rev.SparkMax(23, motor_type)

        # ------ Back Left (bl) ---
        self.bl_drive_motor = rev.SparkFlex(18, motor_type)
        self.bl_turn_motor  = rev.SparkMax(22, motor_type)

        # ---- Back Right (br) ---
        self.br_drive_motor = rev.SparkFlex(19, motor_type)
        self.br_turn_motor  = rev.SparkMax(24, motor_type)

        # Offsets in radians (measure in REV Hardware Client with wheels facing forward)
        self.fl_abs_offset_rad = 0.0
        self.fr_abs_offset_rad = 0.0
        self.bl_abs_offset_rad = 0.0
        self.br_abs_offset_rad = 0.0

        # If any module drives/turns backwards, flip these:
        # self.fl_drive_inverted = True/False, etc.
        # Then, in each module's setup, you can read those values if you want.
        #
        # Keeping it simple: set these directly on the module objects in teleopInit
        # after MagicBot has created components.

    def teleopInit(self):
        # Keep the swerve component's limits synced with your driver scaling
        self.swerve.max_speed_mps = self.max_speed_mps
        self.swerve.max_omega_radps = self.max_omega_radps

    def teleopPeriodic(self):
        x = -self.driver.getLeftY()   # forward/back
        y = -self.driver.getLeftX()   # strafe
        rot = -self.driver.getRightX()  # rotate

        x = deadband(x)
        y = deadband(y)
        rot = deadband(rot)

        # squared inputs for finer control near center
        x = math.copysign(x * x, x)
        y = math.copysign(y * y, y)
        rot = math.copysign(rot * rot, rot)

        vx = x * self.max_speed_mps
        vy = y * self.max_speed_mps
        omega = rot * self.max_omega_radps

        wpilib.SmartDashboard.putNumber("cmd/vx_mps", vx)
        wpilib.SmartDashboard.putNumber("cmd/vy_mps", vy)
        wpilib.SmartDashboard.putNumber("cmd/omega_radps", omega)

        # Hold BACK to disable field-oriented (handy for debugging)
        field_oriented = not self.driver.getBackButton()
        self.swerve.drive(vx, vy, omega, field_oriented=field_oriented)

        # Limelight debug (unchanged vibe)
        tv = self.ll.getNumber("tv", 0)
        if tv >= 1:
            tid = int(self.ll.getNumber("tid", -1))
            tx = self.ll.getNumber("tx", 0.0)
            botpose = self.ll.getNumberArray("botpose_wpiblue", [])
            print(f"[robot] sees tag tid={tid} tx={tx:.1f} botpose={botpose}")


if __name__ == "__main__":
    wpilib.run(MyRobot)
