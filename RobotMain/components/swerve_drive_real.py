#components/swerve_drive_real.py
import wpilib
from ntcore import NetworkTableInstance

import navx

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
from wpimath.estimator import SwerveDrive4PoseEstimator

from RobotMain.components.swervemodule import SwerveModule

class SwerveDrive:
    #MagicBot injects these from robot.py createObjects()
    fl: SwerveModule
    fr: SwerveModule
    bl: SwerveModule
    br: SwerveModule

    def setup(self) -> None:
        #------------------------
        #Robot geometry (meters)
        #------------------------
        #Measure from robot center to module locations.
        # +X forward, +Y left.
        #Example numbers - replace with your robot measurements.
        self.fl_location = Translation2d(+0.30, +0.30)
        self.fr_location = Translation2d(+0.30, -0.30)
        self.bl_location = Translation2d(-0.30, +0.30)
        self.br_location = Translation2d(-0.30, -0.30)

        #------------------------
        #Kinematics (swerve math)
        #------------------------
        self.kinematics = SwerveDrive4Kinematics(
            self.fl_location,
            self.fr_location,
            self.bl_location,
            self.br_location,
        )

        #-----------------------
        # navX2 gyro (robot yaw)
        #-----------------------
        # Gyro tells us robot heading so we can do field-oriented drive.
        self.ahrs = navx.AHRS.create_spi()
        self.ahrs.reset()

        #----------------------
        # Stored drive command
        #----------------------
        # We store what the driver wants.
        self.cmd_vx = 0.0    # m/s field-forward
        self.cmd_vy = 0.0    # m/s field-left
        self.cmd_omega = 0.0 # rad/s CCW

        #---------------------
        # Pose estimator (odometry + vision)
        #---------------------
        initial_pose = Pose2d(0.0, 0.0, Rotation2d())
        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_yaw(),
            self.get_module_positions(),
            initial_pose,
        )

        #---------------------
        # Limelight NetworkTables
        #---------------------
        self.ll_name = "limelight"
        nt = NetworkTableInstance.getDefault()
        self.ll_table = nt.getTable(self.ll_name)

    def get_yaw(self) -> Rotation2d:
        yaw_deg = self.ahrs.getYaw()
        return Rotation2d.fromDegrees(yaw_deg)

    def get_module_positions(self):
        return (
            self.fl.get_position(),
            self.fr.get_position(),
            self.bl.get_position(),
            self.br.get_position()
        )

    def drive(self, vx: float, vy: float, omega: float) -> None:
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_omega = omega

    def _get_robot_relative_speeds(self) -> ChassisSpeeds:
        # Convert FIELD commands to ROBOT commands using gyro yaw.
        # this makes joystick forward always mean field forward.
        return ChassisSpeeds.fromFIeldRelativeSpeeds(
            self.cmd_vx,
            self.cmd_vy,
            self.cmd_omega,
            self.get_yaw(),
        )

    def _compute_module_states(self):
        robot_speeds = self._get_robot_relative_speeds()
        states = self.kinematics.toSwerveModuleStates(robot_speeds)
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
        # tv = 1 means Limelight currently has a valid target
        tv = self.ll_table.getNumber("tv", 0)
        if tv < 1:
            return

        # botpose_wpiblue is an array. We use x, y, yaw.
        botpose = self.ll_table.getNumberArray("botpose_wpiblue", [])
        if len(botpose) < 6:
            return

        x = botpose[0]
        y = botpose[1]
        yaw_deg = botpose[5]

        vision_pose = Pose2d(x, y, Rotation2d.fromDegrees(yaw_deg))

        # Simple timestamp correction using Limelight Pipeline latency (ms)
        latency_ms = self.ll_table.getNumber("tl", 0.0)
        timestamp_ms = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)