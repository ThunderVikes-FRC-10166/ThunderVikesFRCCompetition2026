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

It also simulates a Limelight camera that can "see" AprilTags on the
2026 field. The simulated camera publishes data to NetworkTables using
the same keys a real Limelight uses (tv, tid, tx, ty, botpose_wpiblue).
This lets you develop vision-based autonomous code in the simulator and
have it work on the real robot with zero changes.

The simulation loop:
  Robot code sets desired state on each module (speed + angle)
  → Physics reads those targets
  → Smoothly moves simulated velocity/angle toward the targets
  → Feeds results back to SparkSim so encoders update
  → Updates gyro heading based on overall robot motion
  → Moves robot on the simulation field
  → Checks which AprilTags the camera can see
  → Publishes fake Limelight data to NetworkTables
"""

import math
import os
import wpilib
import wpilib.simulation
import wpimath.geometry
import wpimath.kinematics
from wpimath.system.plant import DCMotor
from ntcore import NetworkTableInstance
import rev

try:
    from robotpy_apriltag import AprilTagFieldLayout
    from wpimath.geometry import Pose3d
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False

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
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.modules_initialized = False
        self.sim_modules = []

        self.navx_yaw = None
        try:
            navx_sim = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
            self.navx_yaw = navx_sim.getDouble("Yaw")
        except Exception:
            pass

        # =====================================================================
        # FAKE LIMELIGHT CAMERA SIMULATION
        # =====================================================================
        # This creates a "fake" Limelight camera that publishes the same
        # NetworkTables data a real Limelight would. That way your robot
        # code can read vision data in simulation AND on the real robot
        # without changing anything.
        #
        # HOW IT WORKS:
        # 1. We load the official 2026 AprilTag field layout (tag positions)
        # 2. Every sim cycle, we check if any tags are in front of the camera
        # 3. If yes, we publish the tag info to NetworkTables
        # 4. Your robot code reads it the same way it would read a real Limelight

        self.ll_table = NetworkTableInstance.getDefault().getTable("limelight")

        self.vision_enabled = False
        if APRILTAG_AVAILABLE:
            layout_path = os.path.join("resources", "2026-rebuilt-welded.json")
            if not os.path.exists(layout_path):
                layout_path = os.path.join(
                    os.path.dirname(os.path.abspath("physics.py")),
                    "resources", "2026-rebuilt-welded.json",
                )
            if os.path.exists(layout_path):
                try:
                    self.tag_layout = AprilTagFieldLayout(layout_path)
                    self.vision_enabled = True
                    print("[FakeLimelight] Loaded 2026 AprilTag field layout")
                except Exception as e:
                    print(f"[FakeLimelight] Could not load tag layout: {e}")

        if not self.vision_enabled:
            print("[FakeLimelight] Vision sim disabled (no apriltag library or layout file)")

        self.cam_forward_m = 0.30
        self.cam_left_m = 0.00

        self.max_tag_distance_m = 5.0
        self.fov_half_angle_deg = 30.0

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

    def _pick_visible_tag(self, robot_x, robot_y, heading_rad):
        """
        Find the closest AprilTag that our fake camera can see.

        HOW THIS WORKS:
        1. Calculate where the camera is (offset from robot center)
        2. For each tag on the field, check:
           - Is it close enough? (within max_tag_distance_m)
           - Is it in front of the camera? (not behind us)
           - Is it within our field of view? (within fov_half_angle_deg)
        3. Return the closest visible tag, or None if nothing is visible

        Returns: (tag_id, tx_degrees, distance_meters) or None
        """
        cosA = math.cos(heading_rad)
        sinA = math.sin(heading_rad)

        cam_dx = self.cam_forward_m * cosA - self.cam_left_m * sinA
        cam_dy = self.cam_forward_m * sinA + self.cam_left_m * cosA

        cam_x = robot_x + cam_dx
        cam_y = robot_y + cam_dy

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
            rel_y = -dx * sinA + dy * cosA

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

    def update_sim(self, now, tm_diff):
        """
        Called every simulation time step (~20ms).

        Updates motor simulation, calculates robot motion, updates gyro,
        and simulates the Limelight camera seeing AprilTags.
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

        vx_field = (
            chassis_speeds.vx * math.cos(self.sim_heading)
            - chassis_speeds.vy * math.sin(self.sim_heading)
        )
        vy_field = (
            chassis_speeds.vx * math.sin(self.sim_heading)
            + chassis_speeds.vy * math.cos(self.sim_heading)
        )
        self.sim_x += vx_field * tm_diff
        self.sim_y += vy_field * tm_diff

        if self.navx_yaw is not None:
            try:
                self.navx_yaw.set(-math.degrees(self.sim_heading))
            except Exception:
                pass

        self.physics_controller.drive(chassis_speeds, tm_diff)

        # =================================================================
        # FAKE LIMELIGHT - publish simulated vision data to NetworkTables
        # =================================================================
        if self.vision_enabled:
            seen = self._pick_visible_tag(
                self.sim_x, self.sim_y, self.sim_heading
            )

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

                yaw_deg = math.degrees(self.sim_heading)
                botpose = [
                    self.sim_x, self.sim_y, 0.0,
                    0.0, 0.0, yaw_deg, 11.0,
                ]
                self.ll_table.putNumberArray("botpose_wpiblue", botpose)

                if int(now * 5) != int((now - tm_diff) * 5):
                    print(
                        f"[FakeLimelight] tv=1 tid={tag_id}"
                        f" tx={tx_deg:.1f} dist={dist_m:.2f}m"
                    )
