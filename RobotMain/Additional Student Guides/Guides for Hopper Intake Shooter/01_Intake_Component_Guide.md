# Intake Component Guide

Welcome! In this guide, you will build the **Intake** component for our robot. By the end, you'll have a working piece of code that controls an arm that opens and closes, and a roller that sweeps balls into the robot.

---

## What Is an Intake?

An intake is the part of the robot that **grabs game pieces** (in our case, balls) from the field and brings them inside the robot. Think of it like a hand reaching out, grabbing something, and pulling it in.

Our intake has two parts:

1. **Arm** - A motor-powered arm that swings open (to reach out toward the ground) and swings closed (to tuck back into the robot). It knows when it's fully open or fully closed because of **limit switches**.
2. **Roller** - A spinning cylinder covered in grippy material. When the arm is open and the roller spins, it sweeps balls off the ground and into the robot.

---

## What Are Limit Switches?

A limit switch is a tiny button that gets pressed when something touches it. We mount two of them on the intake:

- **Forward limit switch** - Gets pressed when the arm reaches the **fully open** position.
- **Reverse limit switch** - Gets pressed when the arm reaches the **fully closed** position.

When the arm motor is opening and the forward limit switch gets pressed, we know to stop the motor because the arm is all the way open. Same idea for closing.

Think of limit switches like the bumpers at a bowling alley - they stop you from going too far.

---

## What Is a SparkMax?

A **REV SparkMax** is a motor controller. The robot brain (RoboRIO) can't power motors directly - it sends signals to the SparkMax, and the SparkMax delivers the right amount of power to the motor. Every motor on our robot has its own SparkMax, and each one has a unique **CAN ID** (like a phone number) so the robot brain knows which motor it's talking to.

---

## What Is the MagicBot Component Pattern?

MagicBot is a framework that organizes our robot code. Every component follows the same pattern:

- **`setup()`** - Runs once when the robot starts. This is where we create motor controllers and configure them.
- **`execute()`** - Runs 50 times per second (every 20 milliseconds). This is where the motors actually get told what to do.
- **`will_reset_to`** - A special MagicBot feature that automatically resets variables every cycle. This is a safety feature: if the code calling our component crashes, the motors automatically stop because the "run" flags reset to `False`.

The key idea: other code (like `robot.py`) calls methods like `open_arm()` to **request** an action. Then `execute()` **carries out** that action. If nobody calls `open_arm()` next cycle, the request resets and the motor stops. Safe!

---

## Step 1: The Constants You Need

Before writing the intake code, make sure these constants exist in **both** `RobotMain/constants.py` and `RobotMain/components/constants.py` (they must be identical files). These should already be added for you, but here's what each one means:

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

### What Do These Mean?

- **CAN IDs (30, 31)**: Each motor controller on the robot has a unique address on the CAN bus (the wiring network that connects everything). The numbers 30 and 31 are just defaults — **you must change these to match the actual CAN IDs assigned to your robot's intake motors**. Use the REV Hardware Client software (on a laptop plugged into the robot) to see what ID each SparkMax is set to.
- **Motor speeds (0.5, 0.7)**: These are percentages. `0.5` means 50% power, `0.7` means 70% power. We don't need full speed for the arm, but we want the roller spinning faster to grab balls.
- **Current limits (20, 30 amps)**: These prevent motors from drawing too much electrical current and overheating or tripping circuit breakers. The roller gets a higher limit because it works harder when grabbing balls.
- **Idle modes**:
  - `kBrake` for the arm means when the motor stops, it actively resists movement. This keeps the arm from flopping around.
  - `kCoast` for the roller means when the motor stops, it spins freely. There's no reason to lock the roller when it's off.

---

## Step 2: The Imports

Open (or create) the file `RobotMain/components/intake.py` and start with these imports:

```python
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants
```

### What Each Import Does

- **`rev`** - The REV Robotics library. This gives us access to SparkMax motor controllers and their configuration.
- **`SparkMax`** - The class that represents a REV SparkMax motor controller.
- **`SparkMaxConfig`** - Used to build a configuration (settings) that we apply to a SparkMax.
- **`SparkBase`** - The parent class of SparkMax (we import it in case we need shared types).
- **`will_reset_to`** - MagicBot's safety feature that auto-resets variables every cycle.
- **`constants`** - Our file full of robot settings (CAN IDs, speeds, etc.).

---

## Step 3: The Class and State Variables

```python
class Intake:

    _arm_opening = will_reset_to(False)
    _arm_closing = will_reset_to(False)
    _roller_running = will_reset_to(False)
```

### What's Happening Here

We define three **state flags** using `will_reset_to(False)`:

- `_arm_opening` - Is someone requesting the arm to open? Resets to `False` every cycle.
- `_arm_closing` - Is someone requesting the arm to close? Resets to `False` every cycle.
- `_roller_running` - Is someone requesting the roller to spin? Resets to `False` every cycle.

The underscore `_` at the start is a Python convention meaning "this is private - other code shouldn't touch these directly." Instead, other code calls our public methods like `open_arm()`.

**Why `will_reset_to(False)`?** Safety! If the code that calls `open_arm()` crashes or stops running, these flags automatically go back to `False` on the next cycle, and `execute()` will stop the motors. The robot won't keep running out of control.

---

## Step 4: The `setup()` Method

```python
    def setup(self) -> None:
        self.arm_spark = SparkMax(constants.kIntakeArmCanId, SparkMax.MotorType.kBrushless)
        self.roller_spark = SparkMax(constants.kIntakeRollerCanId, SparkMax.MotorType.kBrushless)

        arm_config = SparkMaxConfig()
        arm_config.setIdleMode(constants.kIntakeArmIdleMode)
        arm_config.smartCurrentLimit(constants.kIntakeArmCurrentLimit)

        roller_config = SparkMaxConfig()
        roller_config.setIdleMode(constants.kIntakeRollerIdleMode)
        roller_config.smartCurrentLimit(constants.kIntakeRollerCurrentLimit)

        self.arm_spark.configure(
            arm_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.roller_spark.configure(
            roller_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.arm_forward_limit = self.arm_spark.getForwardLimitSwitch()
        self.arm_reverse_limit = self.arm_spark.getReverseLimitSwitch()
```

### Breaking It Down Piece by Piece

**Creating the motor controllers:**
```python
self.arm_spark = SparkMax(constants.kIntakeArmCanId, SparkMax.MotorType.kBrushless)
self.roller_spark = SparkMax(constants.kIntakeRollerCanId, SparkMax.MotorType.kBrushless)
```
- We create two SparkMax objects, one for each motor.
- The first argument is the CAN ID (from constants) - this tells the code which physical motor controller to talk to.
- `SparkMax.MotorType.kBrushless` tells the SparkMax that a brushless motor (NEO) is connected. Brushless motors are more efficient and last longer than brushed motors.

**Building the configurations:**
```python
arm_config = SparkMaxConfig()
arm_config.setIdleMode(constants.kIntakeArmIdleMode)
arm_config.smartCurrentLimit(constants.kIntakeArmCurrentLimit)
```
- We create a configuration object, set the idle mode (brake or coast), and set the current limit.
- Think of this as filling out a settings form before submitting it.

**Applying the configurations:**
```python
self.arm_spark.configure(
    arm_config,
    rev.ResetMode.kResetSafeParameters,
    rev.PersistMode.kPersistParameters,
)
```
- `kResetSafeParameters` means "start from factory defaults before applying our settings." This ensures no leftover settings from a previous program cause problems.
- `kPersistParameters` means "save these settings to the SparkMax's memory." Even if power is lost, the settings survive.

**Getting the limit switches:**
```python
self.arm_forward_limit = self.arm_spark.getForwardLimitSwitch()
self.arm_reverse_limit = self.arm_spark.getReverseLimitSwitch()
```
- The SparkMax has built-in support for limit switches. We grab references to the forward and reverse limit switch objects so we can check if they're pressed later.
- Forward limit = arm is fully open.
- Reverse limit = arm is fully closed.

---

## Step 5: The Public Methods (What Other Code Calls)

```python
    def open_arm(self) -> None:
        self._arm_opening = True
        self._arm_closing = False

    def close_arm(self) -> None:
        self._arm_closing = True
        self._arm_opening = False

    def run_roller(self) -> None:
        self._roller_running = True

    def stop(self) -> None:
        self._arm_opening = False
        self._arm_closing = False
        self._roller_running = False

    def is_open(self) -> bool:
        return self.arm_forward_limit.get()

    def is_closed(self) -> bool:
        return self.arm_reverse_limit.get()
```

### What Each Method Does

- **`open_arm()`** - Sets the "opening" flag to `True` and makes sure the "closing" flag is `False`. You can't open and close at the same time!
- **`close_arm()`** - The opposite: sets "closing" to `True` and "opening" to `False`.
- **`run_roller()`** - Sets the roller flag to `True`. The roller will spin in `execute()`.
- **`stop()`** - Emergency stop! Sets all flags to `False` so everything stops.
- **`is_open()`** - Returns `True` if the forward limit switch is pressed (arm is fully open). Other code can check this to know if the arm is ready.
- **`is_closed()`** - Returns `True` if the reverse limit switch is pressed (arm is fully closed).

Notice how these methods are simple - they just set flags. The actual motor control happens in `execute()`. This separation is important in MagicBot: **request behavior in methods, apply behavior in execute()**.

---

## Step 6: The `execute()` Method

```python
    def execute(self) -> None:
        if self._arm_opening and not self.is_open():
            self.arm_spark.set(constants.kIntakeArmSpeed)
        elif self._arm_closing and not self.is_closed():
            self.arm_spark.set(-constants.kIntakeArmSpeed)
        else:
            self.arm_spark.set(0.0)

        if self._roller_running:
            self.roller_spark.set(constants.kIntakeRollerSpeed)
        else:
            self.roller_spark.set(0.0)
```

### Breaking Down the Logic

**Arm control:**
1. If `_arm_opening` is `True` AND the arm is NOT already fully open → spin the arm motor forward at 50% power.
2. Else if `_arm_closing` is `True` AND the arm is NOT already fully closed → spin the arm motor backward (negative speed) at 50% power.
3. Otherwise → stop the arm motor (set to 0).

The limit switch checks (`not self.is_open()`, `not self.is_closed()`) are critical safety features. Without them, the motor would keep trying to push the arm past its physical limits, which could strip gears or burn out the motor.

**Roller control:**
1. If `_roller_running` is `True` → spin the roller at 70% power.
2. Otherwise → stop the roller.

The `set()` method on the SparkMax takes a value from -1.0 to 1.0:
- `1.0` = full speed forward
- `-1.0` = full speed backward
- `0.0` = stopped

---

## Full Working Code

Here is the complete `intake.py` file. You can type this into `RobotMain/components/intake.py`:

```python
import rev
from rev import SparkMax, SparkMaxConfig, SparkBase
from magicbot import will_reset_to
import constants


class Intake:

    _arm_opening = will_reset_to(False)
    _arm_closing = will_reset_to(False)
    _roller_running = will_reset_to(False)

    def setup(self) -> None:
        self.arm_spark = SparkMax(constants.kIntakeArmCanId, SparkMax.MotorType.kBrushless)
        self.roller_spark = SparkMax(constants.kIntakeRollerCanId, SparkMax.MotorType.kBrushless)

        arm_config = SparkMaxConfig()
        arm_config.setIdleMode(constants.kIntakeArmIdleMode)
        arm_config.smartCurrentLimit(constants.kIntakeArmCurrentLimit)

        roller_config = SparkMaxConfig()
        roller_config.setIdleMode(constants.kIntakeRollerIdleMode)
        roller_config.smartCurrentLimit(constants.kIntakeRollerCurrentLimit)

        self.arm_spark.configure(
            arm_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )
        self.roller_spark.configure(
            roller_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.arm_forward_limit = self.arm_spark.getForwardLimitSwitch()
        self.arm_reverse_limit = self.arm_spark.getReverseLimitSwitch()

    def open_arm(self) -> None:
        self._arm_opening = True
        self._arm_closing = False

    def close_arm(self) -> None:
        self._arm_closing = True
        self._arm_opening = False

    def run_roller(self) -> None:
        self._roller_running = True

    def stop(self) -> None:
        self._arm_opening = False
        self._arm_closing = False
        self._roller_running = False

    def is_open(self) -> bool:
        return self.arm_forward_limit.get()

    def is_closed(self) -> bool:
        return self.arm_reverse_limit.get()

    def execute(self) -> None:
        if self._arm_opening and not self.is_open():
            self.arm_spark.set(constants.kIntakeArmSpeed)
        elif self._arm_closing and not self.is_closed():
            self.arm_spark.set(-constants.kIntakeArmSpeed)
        else:
            self.arm_spark.set(0.0)

        if self._roller_running:
            self.roller_spark.set(constants.kIntakeRollerSpeed)
        else:
            self.roller_spark.set(0.0)
```

---

## How It All Connects

Here's the big picture of how the intake fits into the robot:

1. **Constants** (`constants.py`) define the CAN IDs, speeds, and limits.
2. **Intake** (`intake.py`) uses those constants to create and control the motors.
3. **ThunderVikesSuperScorer** (`thunder_vikes_super_scorer.py`) calls `intake.open_arm()`, `intake.run_roller()`, etc. to coordinate the intake with the hopper and shooter.
4. **robot.py** declares the intake as a MagicBot component. MagicBot automatically creates the `Intake` object and calls `setup()` once, then `execute()` every 20ms.

You don't need to worry about steps 3 and 4 yet - those are covered in later guides. For now, just understand that your intake code will be called automatically by MagicBot.

---

## Key Concepts to Remember

| Concept | What It Means |
|---|---|
| **SparkMax** | Motor controller that delivers power to a motor based on commands from the robot brain |
| **CAN ID** | Unique address for each motor controller on the robot's wiring network |
| **Brushless motor** | A more efficient type of motor (NEO motors are brushless) |
| **Limit switch** | A physical button that tells us when something has reached a boundary |
| **`will_reset_to`** | MagicBot safety feature - variables reset every cycle so motors stop if code crashes |
| **`setup()`** | Runs once at startup - create and configure hardware here |
| **`execute()`** | Runs 50 times per second - control motors here based on the current state flags |
| **Idle mode (Brake)** | Motor resists movement when stopped - holds position |
| **Idle mode (Coast)** | Motor spins freely when stopped |
| **Current limit** | Maximum amps a motor can draw - prevents overheating and blown breakers |

---

## Checklist: Your Intake Is Working When...

- [ ] The file imports without errors
- [ ] `open_arm()` makes the arm motor spin forward
- [ ] The arm stops when the forward limit switch is pressed (fully open)
- [ ] `close_arm()` makes the arm motor spin backward
- [ ] The arm stops when the reverse limit switch is pressed (fully closed)
- [ ] `run_roller()` makes the roller spin
- [ ] `stop()` stops everything
- [ ] `is_open()` returns `True` when the arm is fully open
- [ ] `is_closed()` returns `True` when the arm is fully closed
- [ ] If no methods are called for a cycle, all motors stop (thanks to `will_reset_to`)

---

## Next Up

Now that you understand the intake, move on to the **Hopper Component Guide** to learn how the conveyor belt system moves balls from the intake toward the shooter!
