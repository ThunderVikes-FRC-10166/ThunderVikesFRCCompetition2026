import math
import wpilib

def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad

class SwerveModuleSim:

    """
    A simulation-only swerve module.
    Stores the commanded speed/angle and publishes them to SmartDashboard.
    """

    def __init__(self, name: str):
        self.name = name
        self.speed_mps = 0.0
        self.angle_rad = 0.0

    def set(self, speed_mps: float, angle_rad: float):
        self.speed_mps = speed_mps
        self.angle_rad = wrap_to_pi(angle_rad)

    def execute(self):
        wpilib.SmartDashboard.putNumber(f"{self.name}/speed_mps", self.speed_mps)
        wpilib.SmartDashboard.putNumber(f"{self.name}/angle_deg", math.degrees(self.angle_rad))

    