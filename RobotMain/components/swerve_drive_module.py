import math
import wpilib
import rev


def wrap_to_pi_(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad

class SwerveModuleRev:
    # magicBot will inject these from robot.py creatObjects()
    drive_motor = None
    turn_motor = None
    abs_encoder = None
    def setup(self):
        # --- Constants (edit for your robot) ---
        self.wheel_diameter_m = 0.0762 # 3 inches in meterss
        self,drive_gear_ratio = 5.08 #motor
        self.turn_gear_ratio = 5.08 # these aremt verify change theem to the real one

        self.abs_offset_rad = 0.0
        #---Rev sensprs/controllers ---
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder =  self.turn_motor.getEncoder()

        self.drive_ctrl = self.drive_motor.getClosedLoopController()
        self.turn_ctrl = self.turn_motor.getClosedLoopController() 