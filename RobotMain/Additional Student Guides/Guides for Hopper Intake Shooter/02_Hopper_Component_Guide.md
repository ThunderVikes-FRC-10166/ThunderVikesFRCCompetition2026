# Hopper Component Guide

This guide walks you through building the **Hopper** component for our robot. By the end, you will understand what a hopper does, how it works, and have a fully working `hopper.py` file.

---

## What Is the Hopper?

The hopper is a **conveyor belt system** inside our robot. Its job is to move balls from the **intake** (where balls enter the robot) to the **shooter** (where balls get launched out).

Think of it like a conveyor belt at a grocery store checkout. Items (balls) get placed on one end, and the belt moves them to the other end. Our hopper uses **3 motors** spinning **3 rollers** to push the balls along.

---

## The Physical Layout

Here is a simplified picture of how the hopper is arranged inside the robot:

```
Shooter Side (top of incline)
┌──────────────┐
│  ○ Motor 3   │  ← Top roller (closest to shooter)
│              │
│  ○ Motor 2   │  ← Middle roller
│              │     ↑ Balls roll this way (slight incline)
│  ○ Motor 1   │  ← Bottom roller (closest to intake)
└──────────────┘
Intake Side (bottom of incline)
```

**Important detail:** The hopper sits on a **slight incline** — the shooter side is a little higher than the intake side. Because of gravity, balls naturally want to roll **downhill toward the shooter**. This affects how much motor power we need:

- **Pulling balls FROM the intake** (uphill, against gravity) = needs **more power**
- **Pushing balls TO the shooter** (downhill, gravity helps) = needs **less power**

This is why we have two different speed constants instead of just one.

---

## What Constants Does the Hopper Need?

Before writing the hopper code, we need to define some settings in `constants.py`. These tell the code which motors to talk to and how fast to spin them.

Open `RobotMain/constants.py` (and `RobotMain/components/constants.py` — they must stay identical) and find the **HOPPER CONSTANTS** section. Here is what each constant means:

```python
# CAN IDs for hopper motors (CHANGE THESE to match your robot's actual wiring!)
kHopperMotor1CanId = 32     # Bottom roller, closest to intake (default: 32)
kHopperMotor2CanId = 33     # Middle roller (default: 33)
kHopperMotor3CanId = 34     # Top roller, closest to shooter (default: 34)
```

**CAN IDs** are like addresses on a network. Every motor controller on the robot is connected to a shared wire called the **CAN bus**. Each controller needs a unique number so the robot brain (RoboRIO) knows which motor it is talking to. The numbers 32, 33, and 34 are just defaults — **you must change these to match the actual CAN IDs assigned to your robot's hopper motors**. Use the REV Hardware Client software (on a laptop plugged into the robot) to check what ID each SparkMax is set to.

```python
# Motor speeds for each direction
kHopperIntakeSpeed = 0.6    # Speed when pulling balls FROM intake (against incline, needs more power)
kHopperShooterSpeed = 0.4   # Speed when pushing balls TO shooter (gravity helps, less power needed)
```

Motor speeds are a number from **-1.0 to 1.0**:
- `1.0` = full speed forward
- `0.0` = stopped
- `-1.0` = full speed backward

Notice the **intake speed (0.6) is higher than the shooter speed (0.4)**. That is because pulling balls uphill against gravity takes more effort than pushing them downhill where gravity helps.

```python
# Current limits (amps)
kHopperMotorCurrentLimit = 25
```

**Current limits** protect the motors from drawing too much electrical power. If a ball gets jammed, the motor would try to push harder and harder, pulling more and more current (amps). Without a limit, it could overheat and burn out, or trip a circuit breaker. Setting a limit of 25 amps means the motor will stop pushing harder once it hits that threshold.

```python
# Idle mode
kHopperMotorIdleMode = SparkMaxConfig.IdleMode.kBrake  # Hold balls in place when stopped
```

**Idle mode** controls what happens when we stop sending power to a motor:
- **Brake mode** (`kBrake`): The motor resists movement, like putting a car in park. This is what we want for the hopper because we do not want balls rolling around on their own when the hopper is stopped.
- **Coast mode** (`kCoast`): The motor spins freely, like putting a car in neutral.

---

## The Hopper Code — Step by Step

Now let's build the actual `hopper.py` file. Create (or open) the file at `RobotMain/components/hopper.py`.

### Step 1: The File Header and Imports

```python
"""
hopper.py - Hopper/Conveyor System (MagicBot Component)
========================================================

WHAT IS THE HOPPER?
The hopper is a conveyor belt system that moves balls from the intake to the
shooter. It uses 3 motors spinning 3 rollers to push balls along.

The hopper sits on a slight incline — balls naturally tend to roll toward the
shooter side due to gravity. This means:
- When pulling balls FROM the intake (against the incline), we need MORE power
- When feeding balls TO the shooter (gravity helps), we need LESS power

MAGICBOT COMPONENT:
- setup() is called once when the robot starts
- execute() is called every 20ms (50 times per second)
- will_reset_to variables automatically reset each cycle (safety feature)
"""

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants
```

**What are these imports?**

- `rev` — The library for talking to REV Robotics motor controllers (SparkMax).
- `SparkMax` — The class that represents a SparkMax motor controller.
- `SparkMaxConfig` — Used to configure settings on a SparkMax (like current limits and idle mode).
- `SparkBase` — The base class for SparkMax (we import it because `configure()` needs some of its constants).
- `will_reset_to` — A MagicBot feature. Any variable created with `will_reset_to(some_value)` will automatically reset back to `some_value` every single cycle (every 20ms). This is a **safety feature** — if your code crashes or stops calling methods, the variable goes back to its safe default.
- `constants` — Our constants file where all the motor IDs and speeds live.

### Step 2: The Class Definition

```python
class Hopper:
    """
    MagicBot component that controls the hopper conveyor system.

    The hopper moves balls between the intake and the shooter using
    3 roller motors arranged in a conveyor belt configuration.

    HOW MAGICBOT COMPONENTS WORK:
    - Variables set with will_reset_to() automatically reset each cycle
    - This means if nothing calls feed_from_intake() or feed_to_shooter(),
      the hopper stops automatically
    - This is a safety feature! If the code crashes, the hopper stops.
    """

    _motor_speed = will_reset_to(0.0)
```

**What is `_motor_speed = will_reset_to(0.0)`?**

This creates a variable called `_motor_speed` that starts at `0.0` (stopped). The magic part is that **every single cycle** (50 times per second), MagicBot automatically resets this variable back to `0.0`.

Why is this useful? Imagine your code calls `feed_to_shooter()` which sets `_motor_speed = 0.4`. The motors spin at 0.4. But on the **next** cycle (20ms later), if nothing calls `feed_to_shooter()` again, `_motor_speed` goes back to `0.0` and the motors stop.

This means the hopper **only runs when something is actively telling it to run**. If any part of the code crashes, the hopper stops on its own. Safety first!

The underscore `_` at the start of the name is a Python convention meaning "this variable is meant to be used internally by this class, not by outside code."

### Step 3: The `setup()` Method

```python
    def setup(self) -> None:
        """
        Called once when the robot starts up.

        Creates all 3 hopper motor controllers and configures them
        with the correct settings (current limits, idle mode, etc.).
        """

        self.motor1 = SparkMax(constants.kHopperMotor1CanId, SparkMax.MotorType.kBrushless)
        self.motor2 = SparkMax(constants.kHopperMotor2CanId, SparkMax.MotorType.kBrushless)
        self.motor3 = SparkMax(constants.kHopperMotor3CanId, SparkMax.MotorType.kBrushless)

        config = SparkMaxConfig()
        config.setIdleMode(constants.kHopperMotorIdleMode)
        config.smartCurrentLimit(constants.kHopperMotorCurrentLimit)

        self.motor1.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.motor2.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.motor3.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
```

Let's break this down piece by piece:

**Creating the motors:**

```python
self.motor1 = SparkMax(constants.kHopperMotor1CanId, SparkMax.MotorType.kBrushless)
```

This line creates a SparkMax motor controller object. It takes two arguments:
1. **CAN ID** (`constants.kHopperMotor1CanId` which defaults to `32`) — the address of this specific motor on the CAN bus. Remember, this must match your actual robot wiring.
2. **Motor type** (`SparkMax.MotorType.kBrushless`) — tells the controller what kind of motor is connected. NEO motors are brushless motors (they do not have brushes inside that make contact to spin — they use magnets instead, which makes them more efficient and longer-lasting).

We do this three times — once for each of our three rollers.

**Configuring the motors:**

```python
config = SparkMaxConfig()
config.setIdleMode(constants.kHopperMotorIdleMode)
config.smartCurrentLimit(constants.kHopperMotorCurrentLimit)
```

We create one configuration object and set two things:
1. **Idle mode** — Brake mode so the rollers hold still when stopped (balls do not roll around).
2. **Current limit** — 25 amps maximum to protect the motors.

Then we apply this same configuration to all three motors:

```python
self.motor1.configure(
    config,
    rev.ResetMode.kResetSafeParameters,
    rev.PersistMode.kPersistParameters,
)
```

The two extra parameters are:
- `kResetSafeParameters` — Clears any old settings on the motor controller before applying new ones. This ensures a clean starting state.
- `kPersistParameters` — Saves the settings to the motor controller's memory so they survive a power cycle.

Since all three motors have the same role (spinning a roller), they all get the same configuration. We reuse the same `config` object for all three.

### Step 4: The Control Methods

These are the methods that other parts of the robot code will call to tell the hopper what to do:

```python
    def feed_from_intake(self) -> None:
        """
        Run the hopper to pull balls from the intake toward the shooter.

        This runs against the incline so it uses the higher intake speed.
        """
        self._motor_speed = constants.kHopperIntakeSpeed

    def feed_to_shooter(self) -> None:
        """
        Run the hopper to push balls toward the shooter.

        Gravity assists in this direction so it uses the lower shooter speed.
        """
        self._motor_speed = constants.kHopperShooterSpeed

    def stop(self) -> None:
        """Stop all hopper motors immediately."""
        self._motor_speed = 0.0
```

**`feed_from_intake()`** — Sets the motor speed to `0.6` (the intake speed constant). This is the higher speed because the balls need to travel uphill against gravity.

**`feed_to_shooter()`** — Sets the motor speed to `0.4` (the shooter speed constant). This is the lower speed because gravity is helping the balls roll toward the shooter.

**`stop()`** — Sets the motor speed to `0.0`. Note that because of `will_reset_to(0.0)`, the hopper will stop on its own if nothing calls a method. But having an explicit `stop()` method is still useful when you want to make sure it stops **right now** (for example, an emergency stop button).

Notice how simple these methods are — they just set `_motor_speed` to a value. The actual motor control happens in `execute()`, which runs every cycle.

### Step 5: The `execute()` Method

```python
    def execute(self) -> None:
        """
        Called every 20ms by MagicBot.

        Sets all 3 motors to the requested speed. If no method was called
        this cycle, _motor_speed resets to 0.0 and the hopper stops.
        """
        self.motor1.set(self._motor_speed)
        self.motor2.set(self._motor_speed)
        self.motor3.set(self._motor_speed)
```

This is the method that MagicBot calls automatically 50 times per second. It takes whatever `_motor_speed` is currently set to and sends that value to all three motors.

The `.set()` method on a SparkMax takes a number from -1.0 to 1.0 and tells the motor to spin at that percentage of full power.

**The cycle goes like this:**
1. Some other code calls `feed_to_shooter()` → `_motor_speed` becomes `0.4`
2. MagicBot calls `execute()` → all three motors spin at 0.4
3. MagicBot resets `_motor_speed` back to `0.0` (because of `will_reset_to`)
4. If nothing calls a method again, `execute()` runs with `_motor_speed = 0.0` → motors stop

All three motors get the same speed because they are all part of the same conveyor belt. They all need to spin together at the same rate, or the balls would jam.

---

## The Complete File

Here is the entire `hopper.py` file all together:

```python
"""
hopper.py - Hopper/Conveyor System (MagicBot Component)
========================================================

WHAT IS THE HOPPER?
The hopper is a conveyor belt system that moves balls from the intake to the
shooter. It uses 3 motors spinning 3 rollers to push balls along.

The hopper sits on a slight incline — balls naturally tend to roll toward the
shooter side due to gravity. This means:
- When pulling balls FROM the intake (against the incline), we need MORE power
- When feeding balls TO the shooter (gravity helps), we need LESS power

PICTURE OF THE HOPPER:
    Shooter Side (top)
    ┌──────────────┐
    │  ○ Motor 3   │  ← Top roller (closest to shooter)
    │              │
    │  ○ Motor 2   │  ← Middle roller
    │              │     ↑ Balls roll this way (slight incline)
    │  ○ Motor 1   │  ← Bottom roller (closest to intake)
    └──────────────┘
    Intake Side (bottom)

MAGICBOT COMPONENT:
- setup() is called once when the robot starts
- execute() is called every 20ms (50 times per second)
- will_reset_to variables automatically reset each cycle (safety feature)
"""

import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants


class Hopper:
    """
    MagicBot component that controls the hopper conveyor system.

    The hopper moves balls between the intake and the shooter using
    3 roller motors arranged in a conveyor belt configuration.

    HOW MAGICBOT COMPONENTS WORK:
    - Variables set with will_reset_to() automatically reset each cycle
    - This means if nothing calls feed_from_intake() or feed_to_shooter(),
      the hopper stops automatically
    - This is a safety feature! If the code crashes, the hopper stops.
    """

    _motor_speed = will_reset_to(0.0)

    def setup(self) -> None:
        """
        Called once when the robot starts up.

        Creates all 3 hopper motor controllers and configures them
        with the correct settings (current limits, idle mode, etc.).
        """

        self.motor1 = SparkMax(constants.kHopperMotor1CanId, SparkMax.MotorType.kBrushless)
        self.motor2 = SparkMax(constants.kHopperMotor2CanId, SparkMax.MotorType.kBrushless)
        self.motor3 = SparkMax(constants.kHopperMotor3CanId, SparkMax.MotorType.kBrushless)

        config = SparkMaxConfig()
        config.setIdleMode(constants.kHopperMotorIdleMode)
        config.smartCurrentLimit(constants.kHopperMotorCurrentLimit)

        self.motor1.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.motor2.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.motor3.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def feed_from_intake(self) -> None:
        """
        Run the hopper to pull balls from the intake toward the shooter.

        This runs against the incline so it uses the higher intake speed.
        """
        self._motor_speed = constants.kHopperIntakeSpeed

    def feed_to_shooter(self) -> None:
        """
        Run the hopper to push balls toward the shooter.

        Gravity assists in this direction so it uses the lower shooter speed.
        """
        self._motor_speed = constants.kHopperShooterSpeed

    def stop(self) -> None:
        """Stop all hopper motors immediately."""
        self._motor_speed = 0.0

    def execute(self) -> None:
        """
        Called every 20ms by MagicBot.

        Sets all 3 motors to the requested speed. If no method was called
        this cycle, _motor_speed resets to 0.0 and the hopper stops.
        """
        self.motor1.set(self._motor_speed)
        self.motor2.set(self._motor_speed)
        self.motor3.set(self._motor_speed)
```

---

## Key Concepts Review

Here is a summary of the important ideas from this guide:

| Concept | What It Means |
|---------|---------------|
| **CAN Bus** | A shared wire connecting all motor controllers. Each one has a unique ID number (address). |
| **SparkMax** | A motor controller made by REV Robotics. It receives commands from the RoboRIO and controls the motor. |
| **Brushless Motor** | A type of motor that uses magnets instead of brushes. More efficient and longer-lasting. |
| **Current Limit** | A safety cap on how much electrical current (amps) a motor can draw. Prevents overheating. |
| **Idle Mode (Brake)** | When stopped, the motor resists movement. Keeps balls from rolling around. |
| **Idle Mode (Coast)** | When stopped, the motor spins freely. Not used for the hopper. |
| **will_reset_to()** | MagicBot feature that automatically resets a variable every cycle. Safety mechanism. |
| **setup()** | Called once at startup. Creates and configures hardware. |
| **execute()** | Called every 20ms (50 times/sec). Sends commands to motors. |
| **Incline** | The hopper is tilted so gravity helps balls move toward the shooter. |

---

## How the Hopper Connects to the Rest of the Robot

The hopper does not work alone. It is part of a chain:

```
Intake → Hopper → Shooter
```

1. The **Intake** grabs balls from the field.
2. The **Hopper** moves balls from the intake to the shooter.
3. The **Shooter** launches balls at the target.

The **ThunderVikesSuperScorer** component coordinates all three. It decides when to tell the hopper to `feed_from_intake()` or `feed_to_shooter()`. You will learn about that in a later guide.

---

## Testing Your Hopper

After writing `hopper.py`, you can verify it works by running the simulation:

```bash
cd RobotMain
python -m robotpy sim --nogui
```

If the code imports cleanly and the simulation starts without errors, your hopper component is working correctly. The simulation will create virtual SparkMax controllers for each motor so you can test without real hardware.
