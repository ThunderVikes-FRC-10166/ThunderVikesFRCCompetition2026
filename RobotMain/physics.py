"""
physics.py - Robot Simulation Physics
========================================

This file is ONLY used during simulation (when you run 'python -m robotpy sim').
It does NOT run on the real robot.

WHAT DOES IT DO?
This simulates the swerve drive so the robot moves in the simulator.
It reads the desired state that the robot code sends to each module,
then simulates smooth movement toward that target. The encoder values
are updated via SparkSim.iterate() so the robot code sees realistic
feedback.

The simulation loop:
  Robot code sets desired state on each module (speed + angle)
  → Physics reads those targets
  → Smoothly moves simulated velocity/angle toward the targets
  → Feeds results back to SparkSim so encoders update
  → Updates gyro heading based on overall robot motion
  → Moves robot on the simulation field
"""

import math
import wpilib
import wpilib.simulation
import wpimath.geometry
import wpimath.kinematics
from wpimath.system.plant import DCMotor
import rev

import constants


class SimulatedModule:
    """
    Simulates one swerve module by tracking toward desired states.

    Instead of relying on the SparkSim PID (which has simulation-specific
    unit conversion behavior), we read the target speed and angle from the
    SwerveModule's desired_state and smoothly move toward it.

    IMPORTANT COORDINATE FRAMES:
    - "encoder space": the raw angle the encoder reads, includes chassis offset
    - "robot space": the direction the wheel points relative to the robot
    - The chassis angular offset converts between them:
        encoder_angle = robot_angle + chassis_offset
        robot_angle   = encoder_angle - chassis_offset
    - The kinematics (toChassisSpeeds) expects ROBOT-SPACE angles
    - The encoder and PID work in ENCODER-SPACE
    """

    def __init__(self, swerve_module, drive_spark, turn_spark):
        self.module = swerve_module
        self.drive_spark_sim = rev.SparkFlexSim(drive_spark, DCMotor.NEO(1))
        self.turn_spark_sim = rev.SparkMaxSim(turn_spark, DCMotor.NEO550(1))

        self.drive_spark_sim.enable()
        self.turn_spark_sim.enable()

        self.current_speed_mps = 0.0
        self.current_angle_rad = 0.0

    def update(self, dt):
        """
        Move toward the desired state and update encoder feedback.

        Reads the target from the module's desired_state (set by robot code),
        applies a first-order lag for realistic motor response, then feeds
        the result to SparkSim.iterate() to update the encoders.

        Steps:
        1. Read target speed and angle from desired_state
        2. Convert target angle to encoder space (add chassis offset)
        3. Optimize: if turn > 90deg, reverse drive instead (like real module)
        4. Smoothly move speed and angle toward targets
        5. Feed matching velocity to iterate() so encoders stay in sync
        """
        vbus = 12.0

        target_speed = self.module.desired_state.speed
        target_angle = self.module.desired_state.angle.radians()
        target_angle += self.module.chassis_angular_offset

        corrected = wpimath.kinematics.SwerveModuleState(
            target_speed,
            wpimath.geometry.Rotation2d(target_angle),
        )
        corrected.optimize(wpimath.geometry.Rotation2d(self.current_angle_rad))
        target_speed = corrected.speed
        target_angle = corrected.angle.radians()

        drive_alpha = 1.0 - math.exp(-dt / 0.1)
        self.current_speed_mps += drive_alpha * (
            target_speed - self.current_speed_mps
        )

        angle_error = self._wrap_angle_error(
            target_angle - self.current_angle_rad
        )
        turn_alpha = 1.0 - math.exp(-dt / 0.03)
        angle_change = turn_alpha * angle_error
        self.current_angle_rad += angle_change
        self.current_angle_rad = self.current_angle_rad % (2.0 * math.pi)

        self.drive_spark_sim.iterate(self.current_speed_mps, vbus, dt)

        turn_velocity = angle_change / max(dt, 0.001)
        self.turn_spark_sim.iterate(turn_velocity, vbus, dt)

    def get_state(self):
        """
        Get current simulated state for kinematics calculations.

        Returns the state in ROBOT-RELATIVE coordinates (subtracts the
        chassis angular offset) so toChassisSpeeds() works correctly.
        This matches what swerve_module.get_state() returns on a real robot.
        """
        robot_angle = self.current_angle_rad - self.module.chassis_angular_offset
        return wpimath.kinematics.SwerveModuleState(
            self.current_speed_mps,
            wpimath.geometry.Rotation2d(robot_angle),
        )

    @staticmethod
    def _wrap_angle_error(error):
        """Wrap an angle error to [-pi, pi] for shortest-path turning."""
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi
        return error


class PhysicsEngine:
    """
    Simulates the physics of our swerve drive robot.

    Automatically found and used by robotpy's simulation engine.
    Must be called 'PhysicsEngine' and must be in 'physics.py'.
    """

    def __init__(self, physics_controller, robot):
        self.physics_controller = physics_controller
        self.robot = robot

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            wpimath.geometry.Translation2d(
                constants.kWheelBase / 2, constants.kTrackWidth / 2
            ),
            wpimath.geometry.Translation2d(
                constants.kWheelBase / 2, -constants.kTrackWidth / 2
            ),
            wpimath.geometry.Translation2d(
                -constants.kWheelBase / 2, constants.kTrackWidth / 2
            ),
            wpimath.geometry.Translation2d(
                -constants.kWheelBase / 2, -constants.kTrackWidth / 2
            ),
        )

        self.sim_heading = 0.0
        self.modules_initialized = False
        self.sim_modules = []

        self.navx_yaw = None
        try:
            navx_sim = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
            self.navx_yaw = navx_sim.getDouble("Yaw")
        except Exception:
            pass

    def _init_modules(self):
        """Create SimulatedModule objects after MagicBot sets up components."""
        try:
            swerve = self.robot.swerve_drive
            if not hasattr(swerve, 'front_left'):
                return False

            self.sim_modules = [
                SimulatedModule(
                    swerve.front_left,
                    swerve.front_left.driving_spark,
                    swerve.front_left.turning_spark,
                ),
                SimulatedModule(
                    swerve.front_right,
                    swerve.front_right.driving_spark,
                    swerve.front_right.turning_spark,
                ),
                SimulatedModule(
                    swerve.rear_left,
                    swerve.rear_left.driving_spark,
                    swerve.rear_left.turning_spark,
                ),
                SimulatedModule(
                    swerve.rear_right,
                    swerve.rear_right.driving_spark,
                    swerve.rear_right.turning_spark,
                ),
            ]

            self.modules_initialized = True
            return True

        except (AttributeError, Exception) as e:
            print(f"Physics init error: {e}")
            return False

    def update_sim(self, now, tm_diff):
        """
        Called every simulation time step (~20ms).

        Updates motor simulation, calculates robot motion, and updates gyro.
        """
        if not self.modules_initialized:
            if not self._init_modules():
                return

        for sim_mod in self.sim_modules:
            sim_mod.update(tm_diff)

        module_states = tuple(
            sim_mod.get_state() for sim_mod in self.sim_modules
        )

        chassis_speeds = self.kinematics.toChassisSpeeds(module_states)

        self.sim_heading += chassis_speeds.omega * tm_diff

        if self.navx_yaw is not None:
            try:
                self.navx_yaw.set(-math.degrees(self.sim_heading))
            except Exception:
                pass

        self.physics_controller.drive(chassis_speeds, tm_diff)
