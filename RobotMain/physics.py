#TODO: Add code here for simulating
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
    def _init_(self, physics_controller, robot) -> None:
        self.robot = robot
        self.pose = Pose2d(1.0, 1.0, Rotation2d(0.0)) #
        self.yaw_rad =  0.0
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def update_sim(self, now: float, tm_diff: float):
        swerve = getattr(self.robot, "swerve", None)
        cmd = getattr(swerve, "last_cmd", ChassisSpeeds(0.0, 0.0, 0.0))
        self.yaw_rad = wrap_to_pi(self.yaw_rad + cmd.omega * tm_diff)
        heading = Rotation2d(self.yaw_rad)
        cosA = heading.cos()
        sinA = heading.sin()
        field_vx = cmd.vx * cosA - cmd.vy * sinA
        field_vy = cmd.vx * sinA + cmd.vy * cosA
        new_x = self.pose.x() + field_vx * tm_diff
        new_y = self.pose.y() + field_vy * tm_diff
        self.pose = Pose2d(new_x, new_y, heading)
        self.field.setRobotPose(self.pose)
        if swerve is not None and hasattr(swerve, "gyro"):
            swerve.gyro.set_sim_yaw_rad(self.yaw_rad)
            
