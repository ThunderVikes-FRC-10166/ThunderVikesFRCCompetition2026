import math
from gettext import translation
from idlelib.tree import wheel_event

# swerve_drive_sim.py
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds,SwerveDrive4Kinematics, SwerveModuleState

from RobotMain.components.swerve_module_sim import SwerveModuleSim


class SimpleGyro:
    def __init__(self):
        self._yaw_rad = 0.0

    def get_yaw(self):
        return Rotation2d(self._yaw_rad)

    def set_sim_yaw_rad(self, yaw_rad: float):
        self._yaw_rad = yaw_rad

class SwerveDriveSim:
    """
    Minimal swerve drive component for simulation:
    - drive(vx, vy, omega) stores desired chassis speeds
    - execute() compute 4 module targets (speed+angle) and updates the modules
    - last_cmd is used by physics.py to move the robot pose on the field
    """

    def __init__(self):
        # Top speeds (tune later)
        self.max_speed_mps = 4.0
        self.max_omega_radps = 2.0 * math.pi


        # Robot geometry (meters)
        wheelbase_m = 0.60
        trackwidth_m = 0.60
        hw = wheelbase_m / 2.0
        ht = trackwidth_m / 2.0

        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(+hw, +ht), # FL
            Translation2d(+hw, -ht), # FR
            Translation2d(-hw, +ht), # BL
            Translation2d(-hw, -ht), # BR
        )

        self.gyro = SimpleGyro()
        self.field_oriented = True

        # Sim modules
        self.fl = SwerveModuleSim("FL")
        self.fr = SwerveModuleSim("FR")
        self.bl = SwerveModuleSim("BL")
        self.br = SwerveModuleSim("BR")

        self._desired_cmd = ChassisSpeeds(0.0, 0.0, 0.0)

        # physics.pi reads this to move the robot
        self.last_cmd = ChassisSpeeds(0.0, 0.0, 0.0)

    def drive(self, vx_mps: float,vy_mps: float, omega_radps: float):
         vx_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vx_mps))
         vy_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vy_mps))
         omega_radps = max(-self.max_omega_radps, min(self.max_omega_radps, omega_radps))

         if self.field_oriented:
             yaw = self.gyro.get_yaw()
             self._desired_cmd = ChassisSpeeds.fromFieldRelativeSpeeds(
                 vx_mps, vy_mps, omega_radps, yaw
             )
         else:
            self._desired_cmd = ChassisSpeeds(vx_mps, vy_mps, omega_radps)

         self.last_cmd = self._desired_cmd

    def execute(self):
        states = self.kinematics.toSwerveModuleStates(self._desired_cmd)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, self.max_speed_mps)

        self.fl.set(states[0].speed, states[0].angle.radians())
        self.fr.set(states[1].speed, states[1].angle.radians())
        self.bl.set(states[2].speed, states[2].angle.radians())
        self.br.set(states[3].speed, states[3].angle.radians())

        self.fl.execute()
        self.fr.execute()
        self.bl.execute()
        self.br.execute()






