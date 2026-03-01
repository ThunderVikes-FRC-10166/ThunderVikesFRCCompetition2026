"""
constants.py - Robot Configuration Constants
=============================================

This file holds ALL the important numbers that define how our robot works.
Think of it like the robot's "settings" file. If we need to change something
about how the robot drives, turns, or behaves, we change it here.

WHY A SEPARATE FILE?
- Keeps all our "magic numbers" in one place
- Easy to find and change values without digging through code
- Students can quickly adjust the robot without understanding all the code

UNITS:
- Distances are in METERS (1 meter ≈ 3.28 feet)
- Angles are in RADIANS (π radians = 180 degrees)
- Speeds are in meters per second (m/s)
- CAN IDs are just numbers assigned to each motor controller on the CAN bus
"""

import math
import rev
from rev import SparkMaxConfig, SparkFlexConfig

# =============================================================================
# DRIVE SPEED LIMITS
# =============================================================================
# These are NOT the maximum speeds the robot CAN go, but the maximum speeds
# we ALLOW the robot to go. Think of it like setting a speed limit.

# Maximum forward/sideways speed in meters per second
# 4.0 m/s is about 8.9 mph - a solid speed for competition driving!
kMaxSpeed = 4.0  # meters per second

# Maximum spinning speed - how fast the robot can rotate in place
# 2π radians per second = one full rotation per second
kMaxAngularSpeed = 2 * math.pi  # radians per second

# =============================================================================
# SLEW RATE LIMITS (Acceleration Smoothing)
# =============================================================================
# These control how quickly the robot speeds up and slows down.
# Without these, the robot would jerk around when you move the joystick.
# Think of it like cruise control that smoothly gets you up to speed.

kDirectionSlewRate = 1.2   # How fast the drive direction can change (radians/sec)
kMagnitudeSlewRate = 1.8   # How fast the drive speed can change (percent/sec)
kRotationalSlewRate = 2.0  # How fast the rotation speed can change (percent/sec)

# =============================================================================
# CHASSIS DIMENSIONS
# =============================================================================
# These define the physical size of our robot's drive base.
# We measure from the CENTER of the robot to each wheel module.
#
# Our robot is a PERFECT SQUARE with modules 30cm (0.30m) from center.
# That means the total distance between left and right wheels is 0.60m,
# and the total distance between front and back wheels is also 0.60m.
#
#   Front
#   FL ---- FR      ← kTrackWidth = 0.60m (total left-to-right)
#   |        |
#   |   [C]  |      ← [C] = Center of robot
#   |        |
#   RL ---- RR      ← kWheelBase = 0.60m (total front-to-back)

kTrackWidth = 0.60  # Distance between left and right wheels (meters)
kWheelBase = 0.60   # Distance between front and back wheels (meters)

# =============================================================================
# ANGULAR OFFSETS
# =============================================================================
# Each swerve module's turning encoder has a "home" position.
# These offsets tell the code where each module's "forward" direction is
# relative to the chassis. This is a property of the MAXSwerve module design.
#
# Think of it like calibrating a compass - we need to know which way each
# module thinks is "north" compared to the robot's actual "north" (forward).

kFrontLeftChassisAngularOffset = math.pi   # -90 degrees
kFrontRightChassisAngularOffset = 0              # 0 degrees
kRearLeftChassisAngularOffset = math.pi          # 180 degrees
kRearRightChassisAngularOffset = 0    # 90 degrees

# =============================================================================
# CAN BUS IDS
# =============================================================================
# Every motor controller on the robot has a unique ID number on the CAN bus.
# The CAN bus is like a network cable that connects all the motor controllers.
# Each motor needs its own "address" so the robot brain knows who to talk to.
#
# DRIVE motors use SparkFlex controllers (more powerful, for driving)
# TURN motors use SparkMax controllers (for steering the modules)
#
# You set these IDs using the REV Hardware Client software on a Windows PC.

# Front Left Module
kFrontLeftDrivingCanId = 20
kFrontLeftTurningCanId = 21

# Front Right Module
kFrontRightDrivingCanId = 17
kFrontRightTurningCanId = 23

# Rear Left Module
kRearLeftDrivingCanId = 18
kRearLeftTurningCanId = 22

# Rear Right Module
kRearRightDrivingCanId = 19
kRearRightTurningCanId = 24

# =============================================================================
# SWERVE MODULE MECHANICAL CONSTANTS
# =============================================================================
# These numbers come from the physical hardware - the gears, wheels, and motors.

# Should the turning encoder read backwards? Yes, because the MAXSwerve module
# has a gear that reverses the direction between the motor and the output.
kTurningEncoderInverted = True

# GEAR RATIO for driving
# The "gear ratio" is how many times the motor spins for each wheel rotation.
# A higher number means more torque (pushing power) but less speed.
# REV MAXSwerve comes in 3 options:
#   - 12T pinion = 5.50 ratio (HIGH torque, LOW speed)
#   - 13T pinion = 5.08 ratio (MEDIUM - what we're using!)
#   - 14T pinion = 4.71 ratio (LOW torque, HIGH speed)
kDrivingMotorReduction = 5.08  # Medium gear ratio

# WHEEL SIZE
# Our wheels are 3 inches in diameter.
# We convert to meters because WPILib works in metric units.
# 3 inches = 0.0762 meters
kWheelDiameter = 0.0762  # meters (3 inches)
kWheelCircumference = kWheelDiameter * math.pi  # How far the wheel travels in one rotation

# MOTOR SPECS
# The NEO motor (used in SparkFlex for driving) spins at 5676 RPM with no load.
# We convert to rotations per second (RPS) by dividing by 60.
kDrivingMotorFreeSpeedRps = 5676.0 / 60  # ~94.6 rotations per second

# CONVERSION FACTORS
# These convert the raw encoder readings into useful units.
#
# Position: We want METERS (how far the wheel has traveled)
#   wheel circumference / gear ratio = meters per motor rotation
kDrivingEncoderPositionFactor = (kWheelDiameter * math.pi) / kDrivingMotorReduction

# Velocity: We want METERS PER SECOND (how fast the wheel is moving)
#   Same as position factor, but divided by 60 to convert from per-minute to per-second
kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0

# Free speed of the WHEEL (not the motor)
# Motor free speed × wheel circumference / gear ratio
kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDrivingMotorReduction

# TURNING ENCODER
# The turning encoder measures the angle of the module in radians.
# One full rotation = 2π radians = 360 degrees
kTurningEncoderPositionFactor = 2 * math.pi  # radians per rotation
kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0  # radians per second

# PID wrapping bounds for the turning motor
# This tells the PID controller that 0 and 2π are the same position,
# so it can take the shortest path (like going from 350° to 10° through 0°
# instead of going the long way around).
kTurningEncoderPositionPIDMinInput = 0
kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # 2π

# =============================================================================
# PID CONTROLLER GAINS
# =============================================================================
# PID stands for Proportional-Integral-Derivative.
# It's a control algorithm that tries to reach a target value smoothly.
#
# P (Proportional): How aggressively to correct errors. Higher = faster but can overshoot.
# I (Integral): Corrects small persistent errors over time. Usually 0 for driving.
# D (Derivative): Slows down as we approach the target. Prevents overshooting.
# FF (Feed Forward): A prediction of what output is needed. Helps the P term.

# DRIVING motor PID (controls wheel speed)
kDrivingP = 0.04
kDrivingI = 0
kDrivingD = 0
kDrivingFF = 1 / kDriveWheelFreeSpeedRps  # Predict output based on desired speed
kDrivingMinOutput = -1  # Full reverse
kDrivingMaxOutput = 1   # Full forward

# TURNING motor PID (controls module direction)
kTurningP = 2      # Aggressive P gain for snappy turning response
kTurningI = 0
kTurningD = 0
kTurningFF = 0     # No feed forward needed for position control
kTurningMinOutput = -1
kTurningMaxOutput = 1

# =============================================================================
# MOTOR SETTINGS
# =============================================================================

# Idle mode: What happens when no power is being sent to the motor?
# kBrake = Motor resists movement (like putting a car in park)
# kCoast = Motor spins freely (like putting a car in neutral)
kDrivingMotorIdleMode = SparkMaxConfig.IdleMode.kBrake
kTurningMotorIdleMode = SparkMaxConfig.IdleMode.kBrake

# Current limits prevent the motors from drawing too much power
# and potentially burning out or tripping breakers.
kDrivingMotorCurrentLimit = 50  # Amps - drive motors work harder
kTurningMotorCurrentLimit = 20  # Amps - turning motors need less power

# =============================================================================
# ABSOLUTE ENCODER OFFSETS
# =============================================================================
# Each module's absolute encoder has a slightly different "zero" position
# due to how it was assembled. These offsets correct for that.
# You find these by pointing all modules straight forward and reading
# the raw encoder values.
#
# SET THESE TO 0 INITIALLY, then calibrate on the real robot!
kFrontLeftAbsoluteEncoderOffset = 0.446
kFrontRightAbsoluteEncoderOffset = 0.492
kRearLeftAbsoluteEncoderOffset = 0.640
kRearRightAbsoluteEncoderOffset = 0.781

# =============================================================================
# AUTONOMOUS CONSTANTS
# =============================================================================
# These control how the robot moves during the autonomous period
# (first 15 seconds of the match when the robot drives itself).

kAutoMaxSpeed = 3.0          # meters per second (slower than teleop for safety)
kAutoMaxAcceleration = 2.0   # meters per second squared
kAutoMaxAngularSpeed = math.pi       # radians per second
kAutoMaxAngularAcceleration = math.pi  # radians per second squared

# Auto PID controllers for path following
kPXController = 0.5
kPYController = 0.5
kPThetaController = 0.5

# =============================================================================
# OPERATOR INTERFACE (CONTROLS)
# =============================================================================
# Port numbers for the joysticks/controllers plugged into the Driver Station.

kDriverControllerPort = 0    # Main driver's controller
kOperatorControllerPort = 1  # Operator's controller (for other mechanisms)
kDriveDeadband = 0.08        # Ignore tiny joystick movements (prevents drift)

# =============================================================================
# D-PAD SPEED
# =============================================================================
# When using the D-pad (direction buttons), the robot moves at exactly
# 1 meter per second in the pressed direction. This is useful for
# precise positioning on the field.
kDpadSpeed = 1.0  # meters per second

# =====================================
# Intake constants
# =====================================
# The intake has two motors:
# 1. ARM motor - moves the intake arm up/down using the limit switches
# 2. ROLLER motor - spins to sweep balls into the robot

# CAN IDs for intake motors (CHANGE THESE to match your robot's actual wiring!)
kIntakeArmCanId = 30        # SparkMax controlling the arm pivot (default :30)
kIntakeRollerCanId = 31     # SparkMax controlling the roller (default: 31)

# motor speeds (percentage: -1.0 to 1.0)
kIntakeArmSpeed = 0.5       # how fast the arm opens/closes (50% power)
kIntakeRollerSpeed = 0.7    # how fast the roller spins to grab balls (70% power)

# Current limits (amps) - protects motors from burning out
kIntakeArmCurrentLimit = 20
kIntakeRollerCurrentLimit = 30

# Idle modes
kIntakeArmIdleMode = SparkMaxConfig.IdleMode.kBrake   # Hold position when stopped
kIntakeRollerIdleMode = SparkMaxConfig.IdleMode.kCoast   # let roller spin freely when stopped

# limit switch DIO channels (robotRIO Digital Input/Output ports)
# CHNAGE THESE to match which DIO parts you wired your limit switches to!
kIntakeArmForwardLimitDIO = 0    # DIO port 0 - forward limit switch (arm fully open)
kIntakeArmReverseLimitDIO = 1   # DIO port 1 - reverse limit switch (arm fully closed)

# ====================================
# hopper constants
# ====================================
# The hopper is a conveyor belt system with 3 motors spinning 3 rollers.
# It sits on a slight incline so balls naturally roll toward the shooter side.
# this means we need MORE power pulling from intake (uphill) and LESS power
# pushing toward the shooter (gravity helps).

# CAN IDs for hopper motors (CHANGE THESE to match your robot's actual wiring)
kHopperMotor1CanId = 32 # Bottom roller, closest to intake (default: 32)
kHopperMotor2CanId = 33 # Middle roller (default: 33)
kHopperMotor3CanId = 34 # Top roller, closest to shooter (default: 34)

# Motor speeds for each direction
kHopperIntakeSpeed = 0.6   # Speed when pulling balls FROM intake (against incline, needs more power)
kHopperShooterSpeed = 0.4  # Speed when pushing balls TO shooter (gravity helps, less power needed)

# Current limits (amps)
kHopperMotorCurrentLimit = 25

# Idle Mode
kHopperMotorIdleMode = SparkMaxConfig.IdleMode.kBrake # Hold balls in place when stopped

# =====================================
# Shooter Constants
# =====================================
# The shooter has 3 motors:
# 1. FEEDER motor - pushes balls into the flywheels
# 2. FLYWHEEL TOP motor - spins one direction (the LEADER)
# 3. FLYWHEEL BOTTOM motor - spins the OPPOSITE direction (follows the leader)
# the two flywheels spin against each other to launch the ball out.

# CAN IDs for the shooter motors (CHANGE THESE to match your robot's actual wiring)
kShooterFeederCanId = 35         # Feeder motor (default: 35)
kShooterFlywheelTopCanId = 36    # Top flywheel, leader (default: 36)
kShooterFlywheelBottomCanId = 37 # Bottom flywheel, follower inverted (default: 37)

# Motor speeds
kShooterFeederSpeed = 0.5        # Feeder speed (50% power)
kShooterFlywheelSpeed = 1.0      # Flywheel speed (100% power max launch distance)

# How close the flywheel speed must be to the target before we feed the ball
# This is a percentage (0.85 = 85% of target speed)
kShooterSpinUpThreshold = 0.85

# Current limits (amps)
kShooterFeederCurrentLimit = 25
kShooterFlywheelCurrentLimit = 40 # Flywheels need more current for high-speed spinning

# idle modes
kShooterFeederIdleMode = SparkMaxConfig.IdleMode.kBrake    # hold ball position
kShooterFlywheelIdleMode = SparkMaxConfig.IdleMode.kCoast  # Let flywheels spin down naturally