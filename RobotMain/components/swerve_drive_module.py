import math
import wpilib
import rev

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition


def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class SwerveModule:
    """
    REV MAXSwerve-style module (3" wheels):
      - Drive motor: SPARK Flex (NEO / NEO Vortex) -> velocity closed-loop
      - Turn motor:  SPARK MAX  (NEO 550)          -> position closed-loop using SPARK absolute encoder

    MagicBot will inject these from robot.py createObjects():
      - <name>_drive_motor -> drive_motor
      - <name>_turn_motor  -> turn_motor
      - <name>_abs_offset_rad -> abs_offset_rad
    """
    drive_motor: rev.SparkFlex
    turn_motor: rev.SparkMax
    abs_offset_rad: float

    # Optional per-module inversion flags (set in robot.py if needed)
    drive_inverted: bool = False
    turn_inverted: bool = False

    def setup(self) -> None:
        # ----------------------------
        # Physical constants (edit if needed)
        # ----------------------------
        self.wheel_diameter_m = 0.0762  # 3 inches
        # Medium MAXSwerve is commonly ~5.08:1 (motor rotations per 1 wheel rotation)
        # VERIFY this matches your installed pinion/spur combo
        self.drive_gear_ratio = 5.08

        # ----------------------------
        # Conversion factors
        # ----------------------------
        wheel_circumference_m = math.pi * self.wheel_diameter_m
        meters_per_motor_rotation = wheel_circumference_m / self.drive_gear_ratio
        meters_per_second_per_rpm = meters_per_motor_rotation / 60.0

        # ----------------------------
        # DRIVE: SparkFlex velocity closed-loop in m/s
        # ----------------------------
        drive_cfg = rev.SparkFlexConfig()
        drive_cfg.inverted(self.drive_inverted)
        drive_cfg.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        drive_cfg.smartCurrentLimit(60)

        drive_cfg.encoder.positionConversionFactor(meters_per_motor_rotation)
        drive_cfg.encoder.velocityConversionFactor(meters_per_second_per_rpm)

        # STARTING GAINS (you WILL tune these)
        drive_cfg.closedLoop.pid(0.15, 0.0, 0.0)
        drive_cfg.closedLoop.velocityFF(0.0)
        drive_cfg.closedLoop.outputRange(-1.0, 1.0)

        self.drive_motor.configure(
            drive_cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_ctrl = self.drive_motor.getClosedLoopController()

        # ----------------------------
        # TURN: SparkMax position closed-loop in radians, using absolute encoder
        # ----------------------------
        turn_cfg = rev.SparkMaxConfig()
        turn_cfg.inverted(self.turn_inverted)
        turn_cfg.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        turn_cfg.smartCurrentLimit(30)

        # Absolute encoder native units are rotations. Convert to radians.
        turn_cfg.absoluteEncoder.positionConversionFactor(2.0 * math.pi)

        # Make the turn PID use the absolute encoder
        turn_cfg.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)


        # Choose shortest path around the circle
        turn_cfg.closedLoop.positionWrappingEnabled(True)
        turn_cfg.closedLoop.positionWrappingInputRange(-math.pi, math.pi)

        # STARTING GAINS (oscillation? lower P first)
        turn_cfg.closedLoop.pid(1.2, 0.0, 0.10)
        # test with 0.35 if it works, can update to 1.0
        turn_cfg.closedLoop.outputRange(-0.35, 0.35)

        self.turn_motor.configure(
            turn_cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.turn_ctrl = self.turn_motor.getClosedLoopController()
        self.abs_encoder = self.turn_motor.getAbsoluteEncoder()

        # ----------------------------
        # Targets (set by SwerveDrive)
        # ----------------------------
        self.target_speed_mps = 0.0
        self.target_angle_rad = 0.0

        # Anti-jitter: when you're basically stopped, hold last angle
        self.hold_angle_speed_deadband_mps = 0.05
        self._last_good_angle_rad = 0.0

        # Start by holding current angle so it doesn't jump on enable
        self._last_good_angle_rad = self.get_abs_angle_rad()
        self.target_angle_rad = self._last_good_angle_rad

    def get_abs_angle_rad(self) -> float:
        """
        Absolute turning angle in radians, wrapped to [-pi, pi], with offset applied.

        NOTE: because we set positionConversionFactor(2*pi),
              abs_encoder.getPosition() returns radians already.
        """
        raw_rad = float(self.abs_encoder.getPosition())
        adjusted = raw_rad - float(self.abs_offset_rad)
        return wrap_to_pi(adjusted)

    def set(self, speed_mps: float, angle_rad: float) -> None:
        # Create desired state and optimize to avoid spinning 180° unnecessarily
        desired = SwerveModuleState(float(speed_mps), Rotation2d(float(angle_rad)))
        current_angle = Rotation2d(self.get_abs_angle_rad())
        desired.optimize(current_angle)

        self.target_speed_mps = float(desired.speed)
        self.target_angle_rad = wrap_to_pi(float(desired.angle.radians()))

        # Anti-jitter: don't chase tiny angle changes if we aren't driving
        if abs(self.target_speed_mps) < self.hold_angle_speed_deadband_mps:
            self.target_speed_mps = 0.0
            self.target_angle_rad = self._last_good_angle_rad
        else:
            self._last_good_angle_rad = self.target_angle_rad

    def stop(self) -> None:
        self.target_speed_mps = 0.0
        self.target_angle_rad = self._last_good_angle_rad

    def execute(self) -> None:
        # Drive velocity setpoint is m/s (because we configured conversion factors)
        self.drive_ctrl.setSetpoint(
            self.target_speed_mps,
            rev.SparkLowLevel.ControlType.kVelocity,
        )

        # Turn position setpoint is radians (because we configured conversion factors)
        self.turn_ctrl.setSetpoint(
            self.target_angle_rad,
            rev.SparkLowLevel.ControlType.kPosition,
        )

        # Debug for students
        wpilib.SmartDashboard.putNumber("swerve/abs_angle_deg", math.degrees(self.get_abs_angle_rad()))
        wpilib.SmartDashboard.putNumber("swerve/target_angle_deg", math.degrees(self.target_angle_rad))
        wpilib.SmartDashboard.putNumber("swerve/target_speed_mps", self.target_speed_mps)

    def get_state(self) -> SwerveModuleState:
        speed_mps = float(self.drive_encoder.getVelocity())
        angle = Rotation2d(self.get_abs_angle_rad())
        return SwerveModuleState(speed_mps, angle)

    def get_position(self) -> SwerveModulePosition:
        distance_m = float(self.drive_encoder.getPosition())
        angle = Rotation2d(self.get_abs_angle_rad())
        return SwerveModulePosition(distance_m, angle)

    def reset_drive_distance(self) -> None:
        self.drive_encoder.setPosition(0.0)
        self._last_good_angle_rad = self.get_abs_angle_rad()
        self.target_angle_rad = self._last_good_angle_rad
