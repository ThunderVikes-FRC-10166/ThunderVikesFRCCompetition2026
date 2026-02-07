import math
import wpilib
import rev


from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad

class SwerveModule:
    # magicBot will inject these from robot.py createObjects()
    drive_motor: rev.SparkFlex
    turn_motor: rev.SparkMax
    abs_encoder: wpilib.DutyCycleEncoder
    def setup(self) -> None:

        # --- Physical Constants (edit for your robot) ---
        self.wheel_diameter_m = 0.0762 # 3 inches in meters
        self.drive_gear_ratio = 5.08 #Medium MAXSwerve (motor rotation per wheel rotation)

        # Absolute encoder offset (radians)
        # Meaning: when wheel is pointing FORWARD, we want angle = 0 rad.


        self.abs_offset_rad = 0.0 # TODO: measure per module and set robot in constants
        #---Rev sensors/controllers ---
        self.drive_encoder = self.drive_motor.getEncoder()

        wheel_circumference_m = math.pi * self.wheel_diameter_m
         # 1 motor rotation -> (1/gear_ratio) wheel rotations -> meters traveled
        self.meter_per_motor_rotation = wheel_circumference_m / self.drive_gear_ratio

        # Spark encoder velocity is RPM, so convert RPM -> m/s (divide by 60)
        self.meter_per_second_per_rpm = self.meter_per_motor_rotation / 60.0

        # Tell spark MAX encoder to report meter and m/s directly
        cfg = rev.SparkFlexConfig()
        cfg.encoder.positionConversionFactor(self.meter_per_motor_rotation)
        cfg.encoder.velocityConversionFactor(self.meter_per_second_per_rpm)

        self.drive_motor.configure(
            cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        # Closed-loop controller for drive velocity
        self.drive_ctrl = self.drive_motor.getClosedLoopController()

        #--------------------
        # Turning PID (we control turning with voltage)
        #--------------------
        self.turn_pid = PIDController(4.0, 0.0, 0.2) #TODO: Tune
        self.turn_pid.enableContinuousInput(-math.pi, math.pi)

        self.max_turn_volts = 6.0  # safety limit so it doesn't slam

        #--------------
        # target (what we WANT the module to do)
        #--------------
        self.target_speed_mps = 0.0
        self.target_angle_rad = 0.0

    def get_abs_angle_rad(self) -> float:
        # DutyCycleEncoder gives 0.0..1.0 for one full rotation
        turns_0_to_1 = self.abs_encoder.getAbsolutePosition()
        raw_rad = turns_0_to_1 * 2.0 * math.pi

        # Apply offset so 'forward' becomes 0 rad
        adjusted = raw_rad - self.abs_offset_rad
        return wrap_to_pi(adjusted)

    def set(self, speed_mps: float, angle_rad: float) -> None:
        # Create a desired state using WPILib types
        desired = SwerveModuleState(speed_mps, Rotation2d(angle_rad))

        # Current module angle from the absolute encoder
        current_angle = Rotation2d(self.get_abs_angle_rad())

        # Optimize: may flip wheel 180° and reverse speed to avoid long turns
        desired.optimize(current_angle)

        # Store targets for execute() to use
        self.target_speed_mps = desired.speed
        self.target_angle_rad = desired.angle.radians()

    def _apply_drive(self) -> None:
        # Because we configured conversion factors,
        # The velocity setpoint is in meter/sec (m/s).
        self.drive_ctrl.setSetpoint(
            self.target_speed_mps,
            rev.SparkLowLevel.ControlType.kVelocity
        )
    def _apply_turn(self) -> None:
        current = self.get_abs_angle_rad()

        #PID output is a "turn effort" to match the target angle
        output = self.turn_pid.calculate(current, self.target_angle_rad)

        # Clamp voltage so we don't slam the turn motor
        if output > self.max_turn_volts:
            output = self.max_turn_volts
        elif output < -self.max_turn_volts:
            output = -self.max_turn_volts

        self.turn_motor.setVoltage(output)
    def execute(self) -> None:
        # Apply drive and turn outputs using the targets from set()
        self._apply_drive()
        self._apply_turn()

        # Debug info so students can see what's happening
        wpilib.SmartDashboard.putNumber(
           "swerve/abs_angle_deg",
           math.degrees(self.get_abs_angle_rad())
       )
        wpilib.SmartDashboard.putNumber(
            "swerve/target_angle_deg",
            math.degrees(self.target_angle_rad)
        )
        wpilib.SmartDashboard.putNumber(
            "swerve/target_speed_mps",
            self.target_speed_mps
        )
    def get_state(self) -> SwerveModuleState:
        # Current wheel speed from drive encoder (m/s)
        speed_mps = self.drive_encoder.getVelocity()

        # Current wheel angle from absolute encoder
        angle = Rotation2d(self.get_abs_angle_rad())
        return SwerveModuleState(speed_mps, angle)
    def get_position (self) -> SwerveModulePosition:
        #Total distances traveled by the wheels (meters)
        distance_m = self.drive_encoder.getPosition()

        # Current wheel angle
        angle = Rotation2d(self.get_abs_angle_rad())

        return SwerveModulePosition(distance_m, angle)
    def reset_drive_distance(self) -> None:
        # Reset the drive encoder to 0 meters
        self.drive_encoder.setPosition(0.0)

        # Start by holding the current angle so it doesn't jump on enable
        self.target_angle_rad = self.get_abs_angle_rad()