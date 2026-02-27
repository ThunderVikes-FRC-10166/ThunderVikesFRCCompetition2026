# Constants Update Guide — Adding Intake, Hopper, and Shooter Settings

This guide walks you through the constants you need to add to get our ball-scoring system working. Constants are the "settings" for our robot — things like motor addresses, speeds, and safety limits. We keep them all in one file so they're easy to find and change.

---

# What Are Constants and Why Do They Matter?

Think of constants like the settings on your phone. Instead of burying important numbers deep inside our code, we put them all in one file called `constants.py`. That way, if we need to change how fast a motor spins or which address it lives at, we only change one place.

Our robot has three new mechanisms that need constants:

1. **Intake** — grabs balls from the ground (2 motors)
2. **Hopper** — moves balls through the robot like a conveyor belt (3 motors)
3. **Shooter** — launches balls at the target (3 motors)

That's **8 new motors** total, and each one needs its own settings.

---

# IMPORTANT: Two Files, Same Content

Our project has **two copies** of `constants.py`:

```
RobotMain/constants.py
RobotMain/components/constants.py
```

**Both files must always be identical.** Different parts of the code import from different locations, so if they don't match, you'll get confusing bugs where one part of the robot behaves differently from another.

Whenever you add or change a constant, **update BOTH files**.

---

# Understanding CAN IDs

Every motor controller on the robot is connected to a shared communication wire called the **CAN bus** (Controller Area Network). Think of it like a group chat — every motor controller is in the same chat, so each one needs a unique name (number) so the robot brain knows who it's talking to.

We assign CAN IDs using the **REV Hardware Client** software on a Windows laptop plugged into the robot. The IDs are just numbers — they don't mean anything special, but every motor must have a different one.

**IMPORTANT: The CAN IDs in this guide are defaults (starting at 30). Your robot's actual CAN IDs depend on how the motor controllers were configured when the robot was wired. You MUST check your robot's wiring and use the REV Hardware Client to see what CAN ID each motor controller is set to, then update the constants to match.** If the numbers in the code don't match the numbers on the physical motor controllers, the robot brain will be talking to the wrong motors (or to motors that don't exist).

Our existing swerve drive motors use CAN IDs 17–24. In this guide we'll use **CAN ID 30** as a starting point, but again — change these to match your actual wiring.

Here's an **example** CAN ID map (yours may be different):

| CAN ID (default) | Motor | Mechanism |
|--------|-------|-----------|
| 17–24 | Swerve drive/turn motors | Drivetrain |
| 30 | Arm motor | Intake |
| 31 | Roller motor | Intake |
| 32 | Bottom roller | Hopper |
| 33 | Middle roller | Hopper |
| 34 | Top roller | Hopper |
| 35 | Feeder motor | Shooter |
| 36 | Top flywheel | Shooter |
| 37 | Bottom flywheel | Shooter |

**How to find your actual CAN IDs:**
1. Plug a Windows laptop into the robot via USB
2. Open the **REV Hardware Client** software
3. It will show every SparkMax on the CAN bus and its assigned ID
4. Write down which ID goes to which motor
5. Update the constants in your code to match

---

# Understanding Motor Speeds

Motor speeds in our code are set as a percentage from **-1.0 to 1.0**:

- `1.0` = full speed forward (100%)
- `0.5` = half speed forward (50%)
- `0.0` = stopped
- `-0.5` = half speed backward
- `-1.0` = full speed backward (100%)

We don't always want motors running at 100%. For example, the intake arm only needs 50% power to open and close safely. Running it faster might slam it into the frame and break something.

---

# Understanding Current Limits

**Current** is how much electricity a motor is pulling, measured in **Amps**. A motor that's working really hard (pushing against something stuck, for example) draws more current. If it draws too much, it can:

- **Burn out the motor** (the wires inside overheat)
- **Trip a circuit breaker** (cutting power to that motor or others)
- **Brown out the robot** (the battery voltage drops so low the robot brain reboots)

Current limits tell the motor controller: "If you're drawing more than X amps, back off." It's a safety net. We set different limits for different motors based on how hard they need to work:

- Light-duty motors (turning, arm movement): **20 amps**
- Medium-duty motors (rollers, feeders): **25–30 amps**
- Heavy-duty motors (flywheels): **40 amps**

---

# Understanding Idle Modes

When you stop sending power to a motor, it can do one of two things:

- **Brake mode** (`kBrake`): The motor resists movement, like putting a car in park. Good for motors that need to hold position (like an arm or a conveyor holding balls).
- **Coast mode** (`kCoast`): The motor spins freely, like putting a car in neutral. Good for things like flywheels that you want to spin down gradually, or rollers where you don't want to fight the motor when feeding balls in.

---

# Step-by-Step: Adding the Constants

Open `RobotMain/constants.py` (and remember, you'll copy these same additions to `RobotMain/components/constants.py` when you're done).

All new constants go **at the bottom** of the file, after the existing D-PAD SPEED section.

---

## Step 1: Add Intake Constants

The intake has two motors:
- An **arm motor** that pivots the intake open (down to the ground) and closed (back up into the robot). It uses **limit switches** — physical switches that get pressed when the arm reaches its fully-open or fully-closed position, telling the code "stop, you've gone far enough."
- A **roller motor** that spins to sweep balls into the robot once the arm is open.

Add this block:

```python
# =============================================================================
# INTAKE CONSTANTS
# =============================================================================
# The intake has two motors:
# 1. ARM motor - moves the intake arm up/down using limit switches
# 2. ROLLER motor - spins to sweep balls into the robot

# CAN IDs for intake motors (CHANGE THESE to match your robot's actual wiring!)
kIntakeArmCanId = 30        # SparkMax controlling the arm pivot (default: 30)
kIntakeRollerCanId = 31     # SparkMax controlling the roller (default: 31)

# Motor speeds (percentage: -1.0 to 1.0)
kIntakeArmSpeed = 0.5       # How fast the arm opens/closes (50% power)
kIntakeRollerSpeed = 0.7    # How fast the roller spins to grab balls (70% power)

# Current limits (amps) - protects motors from burning out
kIntakeArmCurrentLimit = 20
kIntakeRollerCurrentLimit = 30

# Idle modes
kIntakeArmIdleMode = SparkMaxConfig.IdleMode.kBrake    # Hold position when stopped
kIntakeRollerIdleMode = SparkMaxConfig.IdleMode.kCoast  # Let roller spin freely when stopped
```

**Why these values?**
- The arm runs at 50% because it doesn't need to move fast, and going slower is safer with limit switches.
- The roller runs at 70% — fast enough to grab balls reliably but not so fast they bounce out.
- The arm uses Brake mode so it stays in position when you stop it. The roller uses Coast mode so balls can still roll through even when the roller motor is off.

---

## Step 2: Add Hopper Constants

The hopper is a conveyor belt system inside the robot with 3 rollers powered by 3 motors. It moves balls from the intake toward the shooter.

Here's the key detail: **the hopper sits on a slight incline**. The intake side is lower and the shooter side is higher. This means gravity naturally helps balls roll toward the shooter, but works against us when pulling balls in from the intake.

Because of this, we use **two different speeds**:
- **Higher speed** (0.6) when pulling from the intake — we're fighting gravity, so we need more power
- **Lower speed** (0.4) when feeding to the shooter — gravity is helping, so we need less power

Add this block:

```python
# =============================================================================
# HOPPER CONSTANTS
# =============================================================================
# The hopper is a conveyor belt system with 3 motors spinning 3 rollers.
# It sits on a slight incline so balls naturally roll toward the shooter side.
# This means we need MORE power pulling from intake (uphill) and LESS power
# pushing toward the shooter (gravity helps).

# CAN IDs for hopper motors (CHANGE THESE to match your robot's actual wiring!)
kHopperMotor1CanId = 32     # Bottom roller, closest to intake (default: 32)
kHopperMotor2CanId = 33     # Middle roller (default: 33)
kHopperMotor3CanId = 34     # Top roller, closest to shooter (default: 34)

# Motor speeds for each direction
kHopperIntakeSpeed = 0.6    # Speed when pulling balls FROM intake (against incline, needs more power)
kHopperShooterSpeed = 0.4   # Speed when pushing balls TO shooter (gravity helps, less power needed)

# Current limits (amps)
kHopperMotorCurrentLimit = 25

# Idle mode
kHopperMotorIdleMode = SparkMaxConfig.IdleMode.kBrake  # Hold balls in place when stopped
```

**Why these values?**
- All three motors share the same current limit (25A) and idle mode because they do similar work.
- Brake mode keeps balls from sliding around when the hopper is stopped.
- The two different speeds account for gravity — this is a real-world physics consideration that makes the robot work better.

---

## Step 3: Add Shooter Constants

The shooter has three motors working together:
1. A **feeder motor** that pushes balls into the flywheels
2. A **top flywheel motor** — this is the **leader** that sets the speed
3. A **bottom flywheel motor** — this is the **follower** that spins in the **opposite direction** from the leader

The two flywheels spin against each other (one clockwise, one counter-clockwise) to grip the ball and launch it out at high speed. Think of it like a pitching machine at a batting cage.

There's also a **spin-up threshold** (0.85 or 85%). This means we won't feed a ball into the flywheels until they're spinning at least 85% of their target speed. If we fed the ball too early, the flywheels wouldn't have enough energy and the shot would be weak.

Add this block:

```python
# =============================================================================
# SHOOTER CONSTANTS
# =============================================================================
# The shooter has 3 motors:
# 1. FEEDER motor - pushes balls into the flywheels
# 2. FLYWHEEL TOP motor - spins one direction (the LEADER)
# 3. FLYWHEEL BOTTOM motor - spins the OPPOSITE direction (follows the leader)
# The two flywheels spin against each other to launch the ball out.

# CAN IDs for shooter motors (CHANGE THESE to match your robot's actual wiring!)
kShooterFeederCanId = 35        # Feeder motor (default: 35)
kShooterFlywheelTopCanId = 36   # Top flywheel, leader (default: 36)
kShooterFlywheelBottomCanId = 37  # Bottom flywheel, follower inverted (default: 37)

# Motor speeds
kShooterFeederSpeed = 0.5       # Feeder speed (50% power)
kShooterFlywheelSpeed = 1.0     # Flywheel speed (100% power for max launch distance)

# How close the flywheel speed must be to the target before we feed the ball
# This is a percentage (0.85 = 85% of target speed)
kShooterSpinUpThreshold = 0.85

# Current limits (amps)
kShooterFeederCurrentLimit = 25
kShooterFlywheelCurrentLimit = 40  # Flywheels need more current for high-speed spinning

# Idle modes
kShooterFeederIdleMode = SparkMaxConfig.IdleMode.kBrake   # Hold ball position
kShooterFlywheelIdleMode = SparkMaxConfig.IdleMode.kCoast  # Let flywheels spin down naturally
```

**Why these values?**
- Flywheels run at 100% power because we want maximum launch distance.
- The feeder only runs at 50% — it just needs to push the ball into the flywheels, not launch it.
- Flywheels get a higher current limit (40A) because spinning heavy wheels at high speed draws a lot of current.
- Flywheels use Coast mode so they spin down naturally. Braking them would waste energy and put unnecessary stress on the motors.
- The feeder uses Brake mode so it can hold a ball in position until we're ready to shoot.

---

## Step 4: Copy Everything to the Second File

Now that you've added all the constants to `RobotMain/constants.py`, you need to make `RobotMain/components/constants.py` match exactly.

The easiest way is to copy the entire file:

1. Open `RobotMain/constants.py`
2. Select all the content (Ctrl+A)
3. Copy it (Ctrl+C)
4. Open `RobotMain/components/constants.py`
5. Select all the content (Ctrl+A)
6. Paste (Ctrl+V)
7. Save

**Both files should now be identical.**

---

# Quick Reference: All New Constants at a Glance

Here's a summary table of everything we added:

### Intake (2 motors)
| Constant | Value | What It Does |
|----------|-------|-------------|
| `kIntakeArmCanId` | 30 | CAN address of the arm motor |
| `kIntakeRollerCanId` | 31 | CAN address of the roller motor |
| `kIntakeArmSpeed` | 0.5 | Arm movement speed (50%) |
| `kIntakeRollerSpeed` | 0.7 | Roller spin speed (70%) |
| `kIntakeArmCurrentLimit` | 20 | Arm motor current limit (amps) |
| `kIntakeRollerCurrentLimit` | 30 | Roller motor current limit (amps) |
| `kIntakeArmIdleMode` | kBrake | Arm holds position when stopped |
| `kIntakeRollerIdleMode` | kCoast | Roller spins freely when stopped |

### Hopper (3 motors)
| Constant | Value | What It Does |
|----------|-------|-------------|
| `kHopperMotor1CanId` | 32 | CAN address of bottom roller |
| `kHopperMotor2CanId` | 33 | CAN address of middle roller |
| `kHopperMotor3CanId` | 34 | CAN address of top roller |
| `kHopperIntakeSpeed` | 0.6 | Speed pulling from intake (uphill) |
| `kHopperShooterSpeed` | 0.4 | Speed feeding to shooter (downhill) |
| `kHopperMotorCurrentLimit` | 25 | Current limit for all 3 motors |
| `kHopperMotorIdleMode` | kBrake | Hold balls in place when stopped |

### Shooter (3 motors)
| Constant | Value | What It Does |
|----------|-------|-------------|
| `kShooterFeederCanId` | 35 | CAN address of feeder motor |
| `kShooterFlywheelTopCanId` | 36 | CAN address of top flywheel |
| `kShooterFlywheelBottomCanId` | 37 | CAN address of bottom flywheel |
| `kShooterFeederSpeed` | 0.5 | Feeder speed (50%) |
| `kShooterFlywheelSpeed` | 1.0 | Flywheel speed (100%) |
| `kShooterSpinUpThreshold` | 0.85 | Min flywheel speed before feeding (85%) |
| `kShooterFeederCurrentLimit` | 25 | Feeder current limit (amps) |
| `kShooterFlywheelCurrentLimit` | 40 | Flywheel current limit (amps) |
| `kShooterFeederIdleMode` | kBrake | Hold ball position when stopped |
| `kShooterFlywheelIdleMode` | kCoast | Let flywheels spin down naturally |

---

# Checklist Before Moving On

- [ ] All intake constants are added to `RobotMain/constants.py`
- [ ] All hopper constants are added to `RobotMain/constants.py`
- [ ] All shooter constants are added to `RobotMain/constants.py`
- [ ] `RobotMain/components/constants.py` is an exact copy of `RobotMain/constants.py`
- [ ] No CAN IDs are duplicated (each motor has a unique number)
- [ ] **CAN IDs match your actual robot wiring** (use the REV Hardware Client to verify)
- [ ] The file imports without errors (test with `python -c "import RobotMain.constants"`)

---

# What's Next?

Now that all the constants are in place, you're ready to build the actual components that use them:

1. **Intake Component** (Guide 01) — Uses `kIntakeArmCanId`, `kIntakeRollerCanId`, and related constants
2. **Hopper Component** (Guide 02) — Uses `kHopperMotor1CanId`, `kHopperMotor2CanId`, `kHopperMotor3CanId`, and related constants
3. **Shooter Component** (Guide 03) — Uses `kShooterFeederCanId`, `kShooterFlywheelTopCanId`, `kShooterFlywheelBottomCanId`, and related constants

Each component guide will reference these constants, so make sure they're all in place before continuing.
