# ThunderVikes FRC Robot Code

## Project Overview
FRC (FIRST Robotics Competition) robot code for the ThunderVikes team, built with RobotPy and the MagicBot framework. This is a swerve drive robot with a scoring system (intake, hopper, shooter) and AprilTag vision for the 2026 season.

## Architecture
- **Framework**: RobotPy (Python FRC library) + MagicBot
- **Language**: Python 3.12
- **Entry point**: `RobotMain/robot.py` (main robot) / `RobotMain/robot_SIM.py` (alt sim)
- **Config**: `RobotMain/pyproject.toml`

## Project Structure
```
RobotMain/
  robot.py              # Main robot program (MagicBot entry point)
  robot_SIM.py          # Alternate simulation robot entry point
  physics.py            # Physics simulation + fake Limelight AprilTag sim
  constants.py          # All robot constants (CAN IDs, PID gains, dimensions, speeds)
  pyproject.toml        # RobotPy configuration for roboRIO deployment
  autonomous/           # Autonomous mode programs
    apriltag_auto.py    # AprilTag demo auto (drives, scans, aligns to tag, scorer stubs)
    drive_box_auto.py   # Old simple drive-in-a-box auto (uses outdated swerve API)
  components/           # Robot subsystem components
    swerve_drive.py     # Swerve drive controller (4-module coordination)
    swerve_module.py    # Individual swerve module (SparkMax + SparkFlex)
    swerve_drive_sim.py # Sim-only swerve drive
    swerve_module_sim.py # Sim-only swerve module
    intake.py           # Intake arm + rollers with limit switches
    hopper.py           # Hopper conveyor (intake → shooter)
    shooter.py          # Dual-motor shooter (leader/follower, optional feeder)
    thunder_viking_super_scorer.py  # Super component coordinating intake/hopper/shooter
    constants.py        # Synced copy of ../constants.py (keep in sync!)
    OUTDATED_swerve_drive_module.py # Old code (not used)
    OUTDATED_swerve_drive_real.py   # Old code (not used)
  resources/            # Field JSON for simulation
    2026-rebuilt-welded.json  # Official 2026 AprilTag field layout
  vision sim guide/     # Documentation for AprilTag simulation setup
```

## Key Components

### Drive System
- **SwerveDrive** (`swerve_drive.py`): Main swerve drive controller
- **SwerveModule** (`swerve_module.py`): Individual wheel module (SparkMax + SparkFlex)

### Scoring System
- **ThunderVikingSuperScorer** (`thunder_viking_super_scorer.py`): Coordinates all scoring
- **Intake** (`intake.py`): Arm deploy/retract with limit switches + roller
- **Hopper** (`hopper.py`): Conveyor from intake to shooter
- **Shooter** (`shooter.py`): Dual-motor shooter (leader/follower); hopper feeds balls in (optional dedicated feeder motor code is commented out with [FEEDER] tags)

### Vision
- **Fake Limelight** (in `physics.py`): Simulates a Limelight camera detecting AprilTags using the 2026 field layout. Publishes to NetworkTables with the same keys a real Limelight uses (tv, tid, tx, ty, botpose_wpiblue).
- **Limelight reader** (in `robot.py`): Reads from the `limelight` NetworkTables table. Works identically in sim and on real robot.

## Control Scheme

### Driver Controller (Port 0)
- Left stick: Forward/backward + strafe
- Right stick: Rotate
- D-pad: Precision movement at 1 m/s
- Start: Reset gyro heading
- X button: Lock wheels in X formation

### Operator Controller (Port 1)
- Left bumper (held): Deploy intake, run rollers + hopper
- Right bumper (held): Spin up shooter wheels
- Right trigger (held + right bumper): Shoot (hopper feeds ball into spinning wheels)
- B button: Emergency stop all scorer motors

## Autonomous Modes

### AprilTag Demo Auto (DEFAULT)
State machine: INIT → DRIVE_FORWARD → LOOK_FOR_TAG → ALIGN_TO_TAG → SHOOT_STUB → DONE
- Detects alliance (red/blue) using DriverStation
- Drives forward using speed × time = distance math
- Scans for AprilTags by rotating slowly
- Aligns to tag using proportional control on tx
- Scorer commands are stubs (ready for when scorer is wired into robot.py)
- Uses MagicBot injection (swerve_drive: SwerveDrive) — not the old self.robot pattern

### Drive Box (SIM) — legacy
Simple time-based box pattern. Uses old `self.robot.swerve.drive()` API from robot_SIM.py.

## Simulation
Workflow command: `cd RobotMain && python -m robotpy sim --nogui`

The physics engine (`physics.py`) simulates:
- Swerve drive modules (motor response, encoder feedback, gyro heading)
- Robot field position tracking (x, y from chassis speed integration)
- Fake Limelight camera (AprilTag detection based on 2026 field layout, FOV, distance)

## Important: constants.py is duplicated
Both `RobotMain/constants.py` and `RobotMain/components/constants.py` must be kept in sync. Always copy changes: `cp RobotMain/constants.py RobotMain/components/constants.py`

## Hardware Configuration

### Swerve Drive CAN IDs
- FL: drive=20, turn=21
- FR: drive=17, turn=23
- RL: drive=18, turn=22
- RR: drive=19, turn=24

### Scorer CAN IDs
- Intake deploy: 30, Intake roller: 31
- Hopper: 32 (also feeds balls into shooter)
- Shooter left: 33, Shooter right: 34 (follower, inverted)
- Optional feeder: 35 (commented out — see [FEEDER] tags in code)

### DIO Ports
- Intake open limit switch: DIO 0
- Intake closed limit switch: DIO 1

## Dependencies (Python)
- `robotpy[all]` - Full RobotPy suite including WPILib + apriltag
- `robotpy-wpilib-utilities` - MagicBot framework
- `robotpy-rev` - REV Spark MAX/Flex support
- `robotpy-navx` - navX2 gyro support
