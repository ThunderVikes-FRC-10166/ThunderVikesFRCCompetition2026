# physics.py

from ntcore import NetworkTableInstance
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d

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
        #Fake Limelight NetworkTables table
        self.ll_table = NetworkTableInstance.getDefault().getTable("limelight")

        #Load official AprilTag layout
        self.tag_layout = AprilTagFieldLayout("resources/2026-rebuilt-welded.json")

        #Camera position (front center of robot)
        self.cam_forward_m = 0.30
        self.cam_left_m = 0.00

        #Visibility settings
        self.max_tag_distance_m = 5.0
        self.fov_half_angle_deg = 30.0

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

        seen = self._pick_visible_tag(self.pose)

        if seen is None:
            self.ll_table.putNumber("tv", 0)
            self.ll_table.putNumber("tid", -1)
            self.ll_table.putNumber("tx", 0.0)
            self.ll_table.putNumber("ty", 0.0)
        else:
            tag_id, tx_deg, dist_m = seen
            self.ll_table.putNumber("tv", 1)
            self.ll_table.putNumber("tid", tag_id)
            self.ll_table.putNumber("tx", tx_deg)
            self.ll_table.putNumber("ty", 0.0)

            yaw_deg = math.degrees(self.yaw_rad)
            botpose = [self.pose.X(), self.pose.Y(), 0.0, 0.0, 0.0, yaw_deg, 11.0]
            self.ll_table.putNumberArray("botpose_wpiblue", botpose)

            if int(now * 5) != int((now - tm_diff) * 5):
                print(f"[FakeLimelight] tv=1 tid={tag_id} tx={tx_deg:.1f} dist={dist_m:.2f}m")


    def _pick_visible_tag(self, robot_pose_2d):
        heading = robot_pose_2d.rotation()
        cosA = heading.cos()
        sinA = heading.sin()

        cam_dx = self.cam_forward_m * cosA - self.cam_left_m *sinA
        cam_dy = self.cam_forward_m * sinA + self.cam_left_m *cosA

        cam_x = robot_pose_2d.X() + cam_dx
        cam_y = robot_pose_2d.Y() + cam_dy

        best = None

        for tag in self.tag_layout.getTags():
            tag_id = tag.ID
            tag_pose: Pose3d = tag.pose

            dx = tag_pose.X() - cam_x
            dy = tag_pose.Y() - cam_y
            dist = math.hypot(dx, dy)

            if dist > self.max_tag_distance_m:
                continue

            rel_x = dx * cosA + dy * sinA
            rel_y = -dx * sinA + dy *cosA

            if rel_x <= 0.01:
                continue

            tx_deg = math.degrees(math.atan2(rel_y, rel_x))

            if abs(tx_deg) > self.fov_half_angle_deg:
                continue

            if best is None or dist < best[0]:
                best = (dist, tag_id, tx_deg)

        if best is None:
            return None

        dist, tag_id, tx_deg = best
        return (tag_id, tx_deg, dist)
