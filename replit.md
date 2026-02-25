# ThunderVikes FRC Robot Code

## Project Overview
FRC (FIRST Robotics Competition) robot code for the ThunderVikes team, built with RobotPy and the MagicBot framework. This is a swerve drive robot simulation for the 2026 season.

## Architecture
- **Framework**: RobotPy (Python FRC library) + MagicBot
- **Language**: Python 3.12
- **Entry point**: `RobotMain/robot.py` (real robot) / `RobotMain/robot_SIM.py` (simulation variant)
- **Config**: `RobotMain/pyproject.toml`

## Project Structure
```
RobotMain/
  robot.py              # Main robot program (MagicBot entry point)
  robot_SIM.py          # Alternate simulation robot entry point
  physics.py            # Physics simulation for --nogui mode
  constants.py          # All robot constants (CAN IDs, PID gains, dimensions)
  pyproject.toml        # RobotPy configuration for roboRIO deployment
  autonomous/           # Autonomous mode programs
  components/           # Robot subsystem components (swerve drive, modules)
  resources/            # Field JSON for simulation
```

## Key Components
- **SwerveDrive** (`components/swerve_drive.py`): Main swerve drive controller
- **SwerveModule** (`components/swerve_module.py`): Individual wheel module (SparkMax + SparkFlex)
- **SwerveDriveSim** (`components/swerve_drive_sim.py`): Simulation-specific swerve drive
- **PhysicsEngine** (`physics.py`): Robotpy simulation physics (--nogui mode)

## Running the Simulation
The workflow command is: `cd RobotMain && python -m robotpy sim --nogui`

This runs the robot code in headless simulation mode. The simulation:
- Runs at 50Hz (20ms per cycle)
- Simulates NavX gyro, SparkMax/SparkFlex motor controllers
- Runs physics simulation for swerve drive kinematics
- Opens NetworkTables on NT3 port 1735, NT4 port 5810

## Dependencies (Python)
- `robotpy[all]` - Full RobotPy suite including WPILib
- `robotpy-wpilib-utilities` - MagicBot framework
- `robotpy-rev` - REV Spark MAX/Flex support
- `robotpy-navx` - navX2 gyro support

## Hardware Configuration (for real robot deployment)
- Motors: REV SparkMax (turning) + REV SparkFlex (driving)
- Gyro: navX2 on MXP port
- 4-wheel swerve drive (MAXSwerve modules, 13T pinion = 5.08:1 gear ratio)
- CAN bus IDs: FL=20/21, FR=17/23, RL=18/22, RR=19/24
