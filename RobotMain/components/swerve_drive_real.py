# components/swerve_drive_real.py
import math
import wpilib
from ntcore import NetworkTableInstance

import navx

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator

from .swerve_drive_module import SwerveModule


class SwerveDrive:
    # MagicBot injects these from robot.py createObjects()
    fl: SwerveModule
    fr: SwerveModule
    bl: SwerveModule
    br: SwerveModule

    def setup(self) -> None:
        # ------------------------
        # Robot geometry (meters)
        # ------------------------
        self.fl_location = Translation2d(+0.30, +0.30)
        self.fr_location = Translation2d(+0.30, -0.30)
        self.bl_location = Translation2d(-0.30, +0.30)
        self.br_location = Translation2d(-0.30, -0.30)

        self.kinematics = SwerveDrive4Kinematics(
            self.fl_location,
            self.fr_location,
            self.bl_location,
            self.br_location,
        )

        # -----------------------
        # navX2 gyro (robot yaw)
        # -----------------------
        self.ahrs = navx.AHRS.create_spi()
        self.ahrs.reset()

        # ----------------------
        # Stored drive command
        # ----------------------
        self.cmd_vx = 0.0    # m/s field-forward
        self.cmd_vy = 0.0    # m/s field-left
        self.cmd_omega = 0.0 # rad/s CCW

        self.field_oriented = True

        # Driving limits (used for scaling + desaturation)
        self.max_speed_mps = 4.5
        self.max_omega_radps = 2.5 * math.pi

        # ---------------------
        # Pose estimator (odometry + vision)
        # ---------------------
        initial_pose = Pose2d(0.0, 0.0, Rotation2d())
        zero_position_modules = (
            SwerveModulePosition(), SwerveModulePosition(),
            SwerveModulePosition(), SwerveModulePosition()
        )
        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_yaw(),
            zero_position_modules,
            initial_pose,
        )

        # ---------------------
        # Limelight NetworkTables
        # ---------------------
        nt = NetworkTableInstance.getDefault()
        self.ll_table = nt.getTable("limelight")

    def get_yaw(self) -> Rotation2d:
        # NavX yaw is clockwise-positive; WPILib expects CCW-positive, so negate.
        return Rotation2d.fromDegrees(-float(self.ahrs.getYaw()))

    def get_module_positions(self):
        return (
            self.fl.get_position(),
            self.fr.get_position(),
            self.bl.get_position(),
            self.br.get_position(),
        )

    def drive(self, vx: float, vy: float, omega: float, field_oriented: bool = True) -> None:
        self.cmd_vx = float(vx)
        self.cmd_vy = float(vy)
        self.cmd_omega = float(omega)
        self.field_oriented = bool(field_oriented)

    def _get_robot_relative_speeds(self) -> ChassisSpeeds:
        wpilib.SmartDashboard.putNumber("navx/yaw_deg", self.get_yaw().degrees())

        if self.field_oriented:
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                self.cmd_vx,
                self.cmd_vy,
                self.cmd_omega,
                self.get_yaw(),
            )

        return ChassisSpeeds(self.cmd_vx, self.cmd_vy, self.cmd_omega)

    def _compute_module_states(self):
        robot_speeds = self._get_robot_relative_speeds()
        states = self.kinematics.toSwerveModuleStates(robot_speeds)
        # desaturate using your configured max speed
        self.kinematics.desaturateWheelSpeeds(states, self.max_speed_mps)
        return states

    def _apply_states(self, states) -> None:
        self.fl.set(states[0].speed, states[0].angle.radians())
        self.fr.set(states[1].speed, states[1].angle.radians())
        self.bl.set(states[2].speed, states[2].angle.radians())
        self.br.set(states[3].speed, states[3].angle.radians())

    def update_odometry(self) -> None:
        self.pose_estimator.update(
            self.get_yaw(),
            self.get_module_positions(),
        )

    def try_add_vision_measurement(self) -> None:
        tv = self.ll_table.getNumber("tv", 0)
        if tv < 1:
            return

        botpose = self.ll_table.getNumberArray("botpose_wpiblue", [])
        if len(botpose) < 6:
            return

        x = float(botpose[0])
        y = float(botpose[1])
        yaw_deg = float(botpose[5])
        vision_pose = Pose2d(x, y, Rotation2d.fromDegrees(yaw_deg))

        latency_ms = float(self.ll_table.getNumber("tl", 0.0))
        timestamp = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)

        self.pose_estimator.addVisionMeasurement(vision_pose, timestamp)

    def get_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: Pose2d) -> None:
        self.fl.reset_drive_distance()
        self.fr.reset_drive_distance()
        self.bl.reset_drive_distance()
        self.br.reset_drive_distance()

        self.pose_estimator.resetPosition(
            self.get_yaw(),
            self.get_module_positions(),
            pose,
        )

    def execute(self) -> None:
        states = self._compute_module_states()
        self._apply_states(states)

        self.update_odometry()
        self.try_add_vision_measurement()

        pose = self.get_pose()
        wpilib.SmartDashboard.putNumber("pose/x_m", pose.X())
        wpilib.SmartDashboard.putNumber("pose/y_m", pose.Y())
        wpilib.SmartDashboard.putNumber("pose/deg", pose.rotation().degrees())
