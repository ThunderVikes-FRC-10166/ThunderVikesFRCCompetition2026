# Shooter Component Guide

This guide walks you through building the **Shooter** component step by step. By the end, you will understand every piece of the code and have a fully working shooter that can launch balls out of your robot.

---

## What Does the Shooter Do?

The shooter is the part of the robot that launches balls at a target. Think of it like a baseball pitching machine — two spinning wheels (called **flywheels**) grab the ball and fling it forward at high speed.

Our shooter has **3 motors** total:

1. **Feeder motor** — This motor pushes a ball from the hopper into the flywheels. Think of it as the "hand" that feeds a ball into the pitching machine.
2. **Top flywheel motor (Leader)** — One of the two spinning wheels. This is the "boss" motor — it decides the speed.
3. **Bottom flywheel motor (Follower)** — The other spinning wheel. It automatically copies whatever the top motor does, but spins in the **opposite direction**. The two wheels spin against each other to grip the ball and shoot it out.

### Why Two Flywheels Spinning Opposite Directions?

Imagine placing a ball between two spinning wheels:
- If both spin the same way, the ball just sits there.
- If they spin in **opposite** directions, they grip the ball and launch it forward.

That's exactly what our shooter does!

### What is Leader/Follower?

Instead of writing code to control both flywheel motors separately (and risking them getting out of sync), we tell the bottom motor to **follow** the top motor. This means:
- We only send speed commands to the **top** motor (the leader).
- The **bottom** motor automatically copies the leader's speed, but inverted (spinning the other way).
- They always stay perfectly synchronized.

This is a built-in feature of the REV SparkMax motor controller — it handles the following in hardware, so it's super fast and reliable.

---

## Before You Start

Make sure the constants have already been added to `RobotMain/constants.py` and `RobotMain/components/constants.py`. The shooter needs these constants:

```python
# CAN IDs for shooter motors (CHANGE THESE to match your robot's actual wiring!)
kShooterFeederCanId = 35        # default: 35
kShooterFlywheelTopCanId = 36   # default: 36
kShooterFlywheelBottomCanId = 37  # default: 37

# Motor speeds
kShooterFeederSpeed = 0.5       # Feeder speed (50% power)
kShooterFlywheelSpeed = 1.0     # Flywheel speed (100% power for max launch distance)

# How close the flywheel speed must be to the target before we feed the ball
kShooterSpinUpThreshold = 0.85

# Current limits (amps)
kShooterFeederCurrentLimit = 25
kShooterFlywheelCurrentLimit = 40  # Flywheels need more current for high-speed spinning

# Idle modes
kShooterFeederIdleMode = SparkMaxConfig.IdleMode.kBrake   # Hold ball position
kShooterFlywheelIdleMode = SparkMaxConfig.IdleMode.kCoast  # Let flywheels spin down naturally
```

### Quick Explanation of These Constants

| Constant | What It Means |
|---|---|
| `kShooterFeederCanId` | The address of the feeder motor on the CAN bus — **change this to match your robot's wiring** |
| `kShooterFlywheelTopCanId` | Address of the top flywheel motor — **change this to match your wiring** |
| `kShooterFlywheelBottomCanId` | Address of the bottom flywheel motor — **change this to match your wiring** |
| `kShooterFeederSpeed` | How fast the feeder pushes balls (0.5 = 50% power) |
| `kShooterFlywheelSpeed` | How fast the flywheels spin (1.0 = 100% full power) |
| `kShooterSpinUpThreshold` | The flywheels must reach 85% of target speed before we feed a ball — this prevents weak shots |
| `kShooterFeederCurrentLimit` | Max amps the feeder can draw (prevents burning out the motor) |
| `kShooterFlywheelCurrentLimit` | Max amps each flywheel can draw (higher because they work harder) |
| `kShooterFeederIdleMode` | Brake mode — when the feeder stops, it holds the ball in place |
| `kShooterFlywheelIdleMode` | Coast mode — when flywheels stop, they spin down naturally instead of braking hard |

---

## Step 1: Create the File and Add Imports

Create the file `RobotMain/components/shooter.py` and add these imports at the top:

```python
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants
```

### What Each Import Does

- **`rev`** — The REV Robotics library. This lets us talk to SparkMax motor controllers.
- **`SparkMax`** — The class that represents a single SparkMax motor controller.
- **`SparkMaxConfig`** — Used to configure settings on a SparkMax (like current limits and idle mode).
- **`SparkBase`** — The parent class of SparkMax (we import it because `configure()` needs some of its constants).
- **`will_reset_to`** — A MagicBot feature that automatically resets a variable back to a default value every robot cycle (every 20 milliseconds). This is a safety feature — if your code stops calling methods, the motors stop.
- **`constants`** — Our constants file with all the settings.

---

## Step 2: Create the Class and State Variables

```python
class Shooter:

    _spin_up = will_reset_to(False)
    _feed = will_reset_to(False)
```

### What Are These State Variables?

These two variables control what the shooter does each cycle:

- **`_spin_up`** — When `True`, the flywheels spin up to full speed. When `False`, they stop.
- **`_feed`** — When `True`, the feeder motor pushes a ball into the flywheels. When `False`, the feeder stops.

The underscore `_` at the beginning is a Python convention meaning "this is private — only this class should touch it directly."

### Why `will_reset_to(False)`?

Every 20 milliseconds (50 times per second), MagicBot automatically sets these back to `False`. This means:

- If your code calls `spin_up()` every cycle, the flywheels keep spinning.
- If your code **stops** calling `spin_up()` (maybe something crashed), the flywheels automatically stop.
- This is a **safety feature** — the robot won't keep doing something dangerous if the code breaks.

---

## Step 3: The `setup()` Method — Creating and Configuring Motors

This is the biggest section. The `setup()` method runs once when the robot starts up. It creates all the motor objects and configures them.

```python
    def setup(self) -> None:
        self.feeder_spark = SparkMax(
            constants.kShooterFeederCanId, SparkMax.MotorType.kBrushless
        )
        self.flywheel_top_spark = SparkMax(
            constants.kShooterFlywheelTopCanId, SparkMax.MotorType.kBrushless
        )
        self.flywheel_bottom_spark = SparkMax(
            constants.kShooterFlywheelBottomCanId, SparkMax.MotorType.kBrushless
        )
```

### Creating the Motors

Each `SparkMax(...)` call creates a connection to a physical motor controller:
- The first argument is the **CAN ID** — the unique address of the motor on the robot's CAN bus network.
- The second argument is the **motor type** — we use `kBrushless` because NEO motors are brushless motors. (Brushless motors are more efficient and last longer than brushed motors.)

We create three motor objects:
1. `feeder_spark` — controls the feeder
2. `flywheel_top_spark` — controls the top flywheel (the leader)
3. `flywheel_bottom_spark` — controls the bottom flywheel (the follower)

### Configuring the Feeder Motor

```python
        feeder_config = SparkMaxConfig()
        feeder_config.setIdleMode(constants.kShooterFeederIdleMode)
        feeder_config.smartCurrentLimit(constants.kShooterFeederCurrentLimit)
```

Here we create a configuration object and set two things:
- **Idle mode (Brake)** — When the feeder motor gets no power, it actively resists movement. This holds the ball in place so it doesn't roll into the flywheels accidentally.
- **Current limit (25 amps)** — This is like a fuse. If the motor tries to draw more than 25 amps (maybe it's stalled or jammed), the SparkMax will limit the current to protect the motor from burning out.

### Configuring the Top Flywheel Motor (Leader)

```python
        flywheel_top_config = SparkMaxConfig()
        flywheel_top_config.setIdleMode(constants.kShooterFlywheelIdleMode)
        flywheel_top_config.smartCurrentLimit(constants.kShooterFlywheelCurrentLimit)
```

Similar to the feeder, but with different settings:
- **Idle mode (Coast)** — When the flywheels stop getting power, they spin down naturally. We don't want to brake them because that would cause unnecessary wear and there's no safety reason to stop them instantly.
- **Current limit (40 amps)** — Higher than the feeder because flywheels need more power to spin up to high speeds.

### Configuring the Bottom Flywheel Motor (Follower) — The Interesting One!

```python
        flywheel_bottom_config = SparkMaxConfig()
        flywheel_bottom_config.setIdleMode(constants.kShooterFlywheelIdleMode)
        flywheel_bottom_config.smartCurrentLimit(constants.kShooterFlywheelCurrentLimit)
        flywheel_bottom_config.follow(
            constants.kShooterFlywheelTopCanId, invert=True
        )
```

This motor has one extra line that makes it special:

**`follow(kShooterFlywheelTopCanId, invert=True)`** — This is the leader/follower magic. We tell the bottom motor: "Whatever motor with CAN ID 36 (the top flywheel) does, you do the same thing but inverted." The `invert=True` parameter means the follower spins in the opposite direction of the leader. This single line handles both the following AND the inversion — no need to call `inverted(True)` separately.

After the follow is set up, we **never need to send commands to the bottom motor** — it automatically mirrors the top motor. We only control the top motor in our code!

### Applying the Configurations

```python
        self.feeder_spark.configure(
            feeder_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.flywheel_top_spark.configure(
            flywheel_top_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.flywheel_bottom_spark.configure(
            flywheel_bottom_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
```

Each `configure()` call sends the configuration to the physical motor controller. The two extra parameters:
- **`kResetSafeParameters`** — First, reset the motor controller to factory defaults. This ensures no old settings interfere.
- **`kPersistParameters`** — Save these settings to the motor controller's memory so they survive a power cycle (if the robot reboots, it remembers these settings).

### Getting the Flywheel Encoder

```python
        self.flywheel_encoder = self.flywheel_top_spark.getEncoder()
```

The encoder is a sensor built into the motor that measures how fast it's spinning. We get it from the **top** flywheel (the leader) because that's the motor we're directly controlling. We'll use this to check if the flywheels have reached full speed before feeding a ball.

---

## Step 4: The Control Methods

These are the methods that other parts of the code (like the ThunderVikesSuperScorer) will call to control the shooter.

### `spin_up()` — Start the Flywheels

```python
    def spin_up(self) -> None:
        self._spin_up = True
```

This sets the `_spin_up` flag to `True`. In the `execute()` method (which runs every cycle), this flag tells the flywheels to spin. Remember, this flag resets to `False` automatically, so `spin_up()` must be called every cycle to keep the flywheels running.

### `feed()` — Push a Ball into the Flywheels

```python
    def feed(self) -> None:
        self._feed = True
```

Same idea — sets the `_feed` flag so the feeder motor runs this cycle.

### `stop()` — Emergency Stop

```python
    def stop(self) -> None:
        self._spin_up = False
        self._feed = False
```

Explicitly sets both flags to `False`. Even though they reset automatically, this method lets you stop everything immediately within the same cycle.

### `is_at_speed()` — Are the Flywheels Ready?

```python
    def is_at_speed(self) -> bool:
        current_speed = abs(self.flywheel_encoder.getVelocity())
        target_rpm = abs(constants.kShooterFlywheelSpeed) * 5676.0 / 60.0
        if target_rpm == 0:
            return False
        return current_speed >= (target_rpm * constants.kShooterSpinUpThreshold)
```

This method answers the question: "Are the flywheels spinning fast enough to shoot?"

Here's what it does step by step:
1. **Get the current speed** — `getVelocity()` returns how fast the flywheel is actually spinning right now in rotations per second. We use `abs()` to get the absolute value (ignore direction, we just care about magnitude).
2. **Calculate the target RPM** — `kShooterFlywheelSpeed` is a percentage (0 to 1.0), so we multiply by the NEO motor's max speed (5676 RPM) and divide by 60 to get rotations per second. This gives us a real speed to compare against.
3. **Safety check** — If the target speed is 0, we can never be "at speed", so return `False`.
4. **Compare** — The flywheel is "at speed" if it's reached at least 85% (`kShooterSpinUpThreshold = 0.85`) of the target speed.

### Why Not Wait for Exactly 100%?

Motors never reach exactly the target speed — they fluctuate slightly above and below. If we waited for exactly 100%, we might never shoot! Using 85% means the flywheels are "close enough" to launch the ball with good power. You can tune this threshold to find the sweet spot between shot power and shot speed.

---

## Step 5: The `execute()` Method — Making Things Actually Move

```python
    def execute(self) -> None:
        if self._spin_up:
            self.flywheel_top_spark.set(constants.kShooterFlywheelSpeed)
        else:
            self.flywheel_top_spark.set(0.0)

        if self._feed:
            self.feeder_spark.set(constants.kShooterFeederSpeed)
        else:
            self.feeder_spark.set(0.0)
```

This method runs every 20 milliseconds. It checks the state flags and controls the motors:

**Flywheel control:**
- If `_spin_up` is `True`: Set the top flywheel to full speed (1.0 = 100%). The bottom flywheel automatically follows because of the leader/follower setup.
- If `_spin_up` is `False`: Set the top flywheel to 0 (stop). The bottom flywheel also stops automatically.

**Feeder control:**
- If `_feed` is `True`: Run the feeder at 50% power to push a ball into the flywheels.
- If `_feed` is `False`: Stop the feeder.

Notice we **never** send commands to `flywheel_bottom_spark` in `execute()`. That's the beauty of leader/follower — the hardware handles it automatically.

---

## The Complete Code

Here's the full `shooter.py` file, all together:

```python
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants


class Shooter:

    _spin_up = will_reset_to(False)
    _feed = will_reset_to(False)

    def setup(self) -> None:
        self.feeder_spark = SparkMax(
            constants.kShooterFeederCanId, SparkMax.MotorType.kBrushless
        )
        self.flywheel_top_spark = SparkMax(
            constants.kShooterFlywheelTopCanId, SparkMax.MotorType.kBrushless
        )
        self.flywheel_bottom_spark = SparkMax(
            constants.kShooterFlywheelBottomCanId, SparkMax.MotorType.kBrushless
        )

        feeder_config = SparkMaxConfig()
        feeder_config.setIdleMode(constants.kShooterFeederIdleMode)
        feeder_config.smartCurrentLimit(constants.kShooterFeederCurrentLimit)

        flywheel_top_config = SparkMaxConfig()
        flywheel_top_config.setIdleMode(constants.kShooterFlywheelIdleMode)
        flywheel_top_config.smartCurrentLimit(constants.kShooterFlywheelCurrentLimit)

        flywheel_bottom_config = SparkMaxConfig()
        flywheel_bottom_config.setIdleMode(constants.kShooterFlywheelIdleMode)
        flywheel_bottom_config.smartCurrentLimit(constants.kShooterFlywheelCurrentLimit)
        flywheel_bottom_config.follow(
            constants.kShooterFlywheelTopCanId, invert=True
        )

        self.feeder_spark.configure(
            feeder_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.flywheel_top_spark.configure(
            flywheel_top_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.flywheel_bottom_spark.configure(
            flywheel_bottom_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.flywheel_encoder = self.flywheel_top_spark.getEncoder()

    def spin_up(self) -> None:
        self._spin_up = True

    def feed(self) -> None:
        self._feed = True

    def stop(self) -> None:
        self._spin_up = False
        self._feed = False

    def is_at_speed(self) -> bool:
        current_speed = abs(self.flywheel_encoder.getVelocity())
        target_rpm = abs(constants.kShooterFlywheelSpeed) * 5676.0 / 60.0
        if target_rpm == 0:
            return False
        return current_speed >= (target_rpm * constants.kShooterSpinUpThreshold)

    def execute(self) -> None:
        if self._spin_up:
            self.flywheel_top_spark.set(constants.kShooterFlywheelSpeed)
        else:
            self.flywheel_top_spark.set(0.0)

        if self._feed:
            self.feeder_spark.set(constants.kShooterFeederSpeed)
        else:
            self.feeder_spark.set(0.0)
```

---

## How It All Connects

Here's how the shooter fits into the bigger picture:

1. **ThunderVikesSuperScorer** decides it's time to shoot.
2. It calls `shooter.spin_up()` every cycle to get the flywheels going.
3. It checks `shooter.is_at_speed()` each cycle — "Are the flywheels ready?"
4. Once the flywheels are at speed, it calls `shooter.feed()` to push the ball in.
5. The ball gets launched!
6. When done, it stops calling the methods, and `will_reset_to(False)` safely stops everything.

---

## Key Concepts to Remember

### MagicBot Component Pattern
Every MagicBot component follows this pattern:
- **`setup()`** — Runs once at startup. Create and configure hardware here.
- **`execute()`** — Runs every 20ms. Read state flags and control motors here.
- **`will_reset_to()`** — State variables that automatically reset. Safety first!
- **Control methods** (`spin_up()`, `feed()`, `stop()`) — Set flags for `execute()` to act on.

### Leader/Follower
- Only send commands to the **leader** motor.
- The **follower** automatically copies the leader (inverted in our case).
- This keeps two motors perfectly synchronized without extra code.

### Spin-Up Threshold
- Always wait for flywheels to reach speed before feeding a ball.
- Using a threshold (85%) instead of exact match prevents the robot from waiting forever.

### Idle Modes
- **Brake** = Motor actively resists movement when stopped (good for holding position).
- **Coast** = Motor spins freely when stopped (good for flywheels that should spin down naturally).

### Current Limits
- Protect motors from drawing too much power.
- Prevents burned-out motors and tripped breakers.
- Set higher for motors that need more power (flywheels = 40A vs feeder = 25A).
