# How Swerve Drive Works - A Student Guide

## What is Swerve Drive?

Imagine you're at a grocery store pushing a shopping cart. Each wheel on the cart can spin freely in any direction. Now imagine if you could control exactly which direction each wheel points AND how fast it spins. That's essentially what swerve drive is!

A swerve drive robot has 4 **swerve modules** - one at each corner. Each module has:
1. A **drive motor** (SparkFlex) that spins the wheel forward/backward
2. A **turn motor** (SparkMax) that rotates the entire wheel left/right

Because each wheel can independently point in any direction and spin at any speed, the robot can:
- Drive forward/backward (like a normal car)
- Drive sideways (strafe) without turning
- Drive diagonally
- Spin in place
- Do ALL of these at the same time!

## Our Hardware Setup

```
        FRONT OF ROBOT
    ┌────────────────────┐
    │  FL              FR│   FL = Front Left Module
    │  (CAN 2,3)  (CAN 4,5)│
    │                    │   Each module has:
    │                    │   - SparkFlex (drive) = first CAN ID
    │      [ROBORIO]     │   - SparkMax (turn) = second CAN ID
    │                    │
    │                    │   Wheels: 3 inch diameter
    │  RL              RR│   Gear Ratio: 5.08 (medium)
    │  (CAN 6,7)  (CAN 8,9)│
    └────────────────────┘

    Robot is a 60cm x 60cm square
    (30cm from center to each module)
```

## How Field-Oriented Drive Works

This is the "killer feature" of our swerve drive. Here's the problem it solves:

**Without field-oriented drive (robot-oriented):**
- Push joystick forward → robot drives wherever its FRONT is pointing
- If the robot is turned sideways, "forward" on the joystick moves it sideways on the field
- This is confusing and hard to drive!

**With field-oriented drive:**
- Push joystick forward → robot ALWAYS moves toward the far end of the field
- It doesn't matter which way the robot is facing
- This is intuitive - like controlling a character in a video game!

**How does it work?**
1. The **NavX gyroscope** constantly measures which way the robot is facing
2. When you push the joystick, the code takes your input and rotates it by the gyro angle
3. This converts your "field-relative" command into a "robot-relative" command
4. The robot then figures out what each wheel needs to do

**Example:**
- Robot is facing 90 degrees to the right
- You push the joystick forward (wanting to go "north" on the field)
- The code sees the robot is turned 90 degrees
- It tells the robot to actually drive to its LEFT (which is "north" from the field's perspective)

## The Math Behind Swerve (Simplified)

### Step 1: Joystick to Robot Speed
The joystick gives us three values:
- **X speed**: Forward/backward (-1 to 1)
- **Y speed**: Left/right (-1 to 1)  
- **Rotation**: Spin speed (-1 to 1)

We multiply these by the maximum speed to get actual speeds in meters/second.

### Step 2: Field-Oriented Conversion
Using the gyroscope angle, we rotate the X/Y speeds so they're relative to the field instead of the robot.

### Step 3: Kinematics (The Hard Math)
WPILib's `SwerveDrive4Kinematics` takes the desired robot motion and calculates what each individual module needs to do. It considers:
- Where each module is on the robot (30cm from center)
- The desired translation (forward/sideways movement)
- The desired rotation

The output is a **speed** and **angle** for each of the 4 modules.

### Step 4: Module Optimization
Before sending commands to the motors, each module "optimizes" its movement. If a wheel needs to turn more than 90 degrees, it's faster to:
- Turn less than 90 degrees in the other direction
- Reverse the drive motor direction

This prevents the modules from doing unnecessary full rotations.

### Step 5: PID Control
Each motor has a **PID controller** running on the SparkMax/SparkFlex hardware:
- **Drive motor PID**: Controls wheel SPEED (velocity control)
- **Turn motor PID**: Controls module ANGLE (position control)

PID stands for Proportional-Integral-Derivative. Think of it like cruise control:
- **P (Proportional)**: "I'm going 40 mph but want 60, so press the gas harder"
- **I (Integral)**: "I've been slightly below my target for a while, push a bit more"
- **D (Derivative)**: "I'm approaching my target speed, ease off the gas"

## How Commands Flow Through the Code

```
1. DRIVER moves joystick
         │
         ▼
2. robot.py reads joystick values
   (teleopPeriodic)
         │
         ▼
3. robot.py calls swerve_drive.set_drive_command(x, y, rot)
         │
         ▼
4. swerve_drive.execute() runs (every 20ms)
   - Updates odometry (where are we?)
   - Applies rate limiting (smooth acceleration)
   - Converts to field-relative if needed
   - Calculates module states using kinematics
         │
         ▼
5. Each swerve_module.set_desired_state(speed, angle)
   - Applies angular offset
   - Optimizes to minimize rotation
   - Sends speed command to drive PID
   - Sends angle command to turn PID
         │
         ▼
6. SparkFlex/SparkMax PID controllers
   make the motors reach target speed/angle
         │
         ▼
7. WHEELS spin and turn → ROBOT MOVES!
```

## Controls Reference

| Input | Action |
|-------|--------|
| Left Stick Up/Down | Drive forward/backward |
| Left Stick Left/Right | Strafe left/right |
| Right Stick Left/Right | Rotate robot |
| D-Pad Up | Drive forward at 1 m/s |
| D-Pad Down | Drive backward at 1 m/s |
| D-Pad Left | Strafe left at 1 m/s |
| D-Pad Right | Strafe right at 1 m/s |
| Start Button | Reset gyro heading |
| X Button (hold) | Lock wheels in X pattern |

## Odometry: Tracking Robot Position

Odometry is how the robot keeps track of where it is on the field. Think of it like the odometer in a car, but in 2D.

**How it works:**
1. Every 20ms (50 times per second), we check:
   - How far each wheel has moved (from drive encoders)
   - Which direction each wheel is pointing (from turn encoders)
   - Which way the robot is facing (from the gyroscope)
2. Using math (kinematics), we calculate how much the robot moved
3. We add that movement to our running position estimate

**Why it matters:**
- Needed for autonomous routines (robot needs to know where it is)
- Can be combined with vision (Limelight camera) for even more accuracy
- Useful for driver assistance features

## Limelight Camera (Coming Soon)

The Limelight camera will use **AprilTags** (special printed targets placed around the field) to determine the robot's exact position. This helps correct any drift in our odometry.

When implemented, it will:
1. See an AprilTag on the field
2. Calculate our exact position from the tag
3. Update our odometry with this more accurate position

## File Structure

```
robot.py                    ← Main entry point, driver controls
constants.py                ← All robot settings and numbers
physics.py                  ← Simulation physics (sim only)
pyproject.toml              ← RobotPy configuration
components/
    __init__.py             ← Makes 'components' a Python package
    swerve_module.py        ← Controls ONE swerve module (motor+encoder)
    swerve_drive.py         ← Controls ALL 4 modules together
autonomous/
    __init__.py             ← Will contain auto routines
```

## How to Run

### Simulation (no robot needed):
```bash
python -m robotpy sim
```

### Deploy to Robot:
```bash
python -m robotpy sync    # Install packages on RoboRIO (first time)
python -m robotpy deploy  # Upload code to RoboRIO
```

## Troubleshooting

### Robot drives in wrong direction
- Check that the gyro heading is correct (press Start to reset)
- Make sure CAN IDs match the physical wiring
- Verify angular offsets in constants.py

### Module spins continuously
- The absolute encoder offset is probably wrong
- Point all modules straight forward and record the raw encoder values
- Update the `kAbsoluteEncoderOffset` values in constants.py

### Robot is sluggish
- Check current limits in constants.py
- Check PID gains (might need tuning)
- Check for mechanical issues (binding, loose connections)

### Simulation doesn't match real robot
- Simulation is approximate - it won't perfectly match reality
- PID behavior differs between sim and real hardware
- Use simulation for logic testing, not precise motion tuning
