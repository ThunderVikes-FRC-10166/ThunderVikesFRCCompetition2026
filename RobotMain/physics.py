# physics.py
import math
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from wpilib import Field2d, SmartDashboard


def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class PhysicsEngine:
    """
    pyfrc will create this class automatically when running `robot.py sim`.
    It calls update_sim(now, tm_diff) repeatedly.
    """

    def __init__(self, physics_controller, robot):
        self.robot = robot

        self.pose = Pose2d(1.0, 1.0, Rotation2d(0.0))  # start somewhere not at (0,0)
        self.yaw_rad = 0.0

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def update_sim(self, now: float, tm_diff: float):
        # Read commanded chassis speeds (robot-relative)
        swerve = getattr(self.robot, "swerve", None)
        cmd = getattr(swerve, "last_cmd", ChassisSpeeds(0.0, 0.0, 0.0))

        # Integrate yaw
        self.yaw_rad = wrap_to_pi(self.yaw_rad + cmd.omega * tm_diff)
        heading = Rotation2d(self.yaw_rad)

        # Convert robot-relative (vx, vy) to field-relative
        cosA = heading.cos()
        sinA = heading.sin()
        field_vx = cmd.vx * cosA - cmd.vy * sinA
        field_vy = cmd.vx * sinA + cmd.vy * cosA

        # Integrate position
        new_x = self.pose.X() + field_vx * tm_diff
        new_y = self.pose.Y() + field_vy * tm_diff
        self.pose = Pose2d(new_x, new_y, heading)

        # Publish robot pose to dashboard field widget
        self.field.setRobotPose(self.pose)

        # Feed the sim gyro (so field-oriented stays correct)
        if swerve is not None and hasattr(swerve, "gyro"):
            swerve.gyro.set_sim_yaw_rad(self.yaw_rad)