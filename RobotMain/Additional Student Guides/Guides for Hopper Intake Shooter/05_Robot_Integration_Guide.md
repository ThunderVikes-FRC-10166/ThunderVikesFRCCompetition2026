# Robot.py Integration Guide — Connecting Everything Together

This guide shows you how to update `robot.py` to wire up the entire scoring system (Intake, Hopper, Shooter, and ThunderVikesSuperScorer) and add operator controller button bindings so a second driver can control it all.

By the end of this guide, your robot will have:
- A **driver controller** (port 0) for driving the robot around
- An **operator controller** (port 1) for controlling the scoring system
- All four new components automatically managed by MagicBot

---

# 0) What is robot.py?

`robot.py` is the **entry point** for your robot code — the first file that runs when you turn on the robot or start simulation. Think of it like the "main menu" of a video game.

We use the **MagicBot framework**, which does a lot of heavy lifting for us:
- It **creates** our components automatically
- It **calls** their `setup()` and `execute()` methods for us
- It **resets** safety variables every cycle so the robot stops if code crashes

Our job in `robot.py` is simple:
1. **Declare** which components we want (MagicBot creates them)
2. **Create** input devices like Xbox controllers
3. **Read** button presses and joystick inputs
4. **Tell** components what to do based on those inputs

---

# 1) Imports — Telling Python What We Need

At the top of `robot.py`, we need to import all the component classes we want to use. Here is what the full imports section looks like:

```python
import wpilib
import wpimath
import wpimath.filter
import magicbot

from components.swerve_drive import SwerveDrive
from components.intake import Intake
from components.hopper import Hopper
from components.shooter import Shooter
from components.thunder_vikes_super_scorer import ThunderVikesSuperScorer
import constants
```

### What each import does

| Import | Why we need it |
|--------|----------------|
| `wpilib` | Core FRC library — gives us `XboxController` and other hardware classes |
| `wpimath` / `wpimath.filter` | Math utilities like `applyDeadband` and `SlewRateLimiter` |
| `magicbot` | The MagicBot framework that manages our components |
| `SwerveDrive` | Our drivetrain component (already existed before this guide) |
| `Intake` | The intake component (arm + roller) |
| `Hopper` | The hopper/conveyor component (3 rollers) |
| `Shooter` | The shooter component (feeder + flywheels) |
| `ThunderVikesSuperScorer` | The super component that coordinates intake, hopper, and shooter |
| `constants` | All our configuration numbers (CAN IDs, speeds, etc.) |

The four new imports (`Intake`, `Hopper`, `Shooter`, `ThunderVikesSuperScorer`) are the components we built in the previous guides. Now we are plugging them into the robot.

---

# 2) Component Declarations — Telling MagicBot What to Create

Inside the `SwerveRobot` class, we declare our components as **class-level type annotations**. This is the magic of MagicBot — just by writing these lines, MagicBot will automatically:

1. Create an instance of each class
2. Call their `setup()` method
3. Call their `execute()` method every 20ms during operation

```python
class SwerveRobot(magicbot.MagicRobot):

    swerve_drive: SwerveDrive

    intake: Intake
    hopper: Hopper
    shooter: Shooter
    super_scorer: ThunderVikesSuperScorer
```

### Why the variable names matter

The variable names are **critical** because MagicBot uses them for **dependency injection**. Here's what that means:

Remember in `thunder_vikes_super_scorer.py`, we wrote:
```python
class ThunderVikesSuperScorer:
    intake: Intake
    hopper: Hopper
    shooter: Shooter
```

MagicBot sees that the `ThunderVikesSuperScorer` needs an `Intake` object called `intake`. It looks at `robot.py` and finds `intake: Intake` declared there. It automatically gives the Super Scorer a reference to that same Intake instance.

This is like a factory: the manager (Super Scorer) says "I need the intake department", and MagicBot says "Here it is — same one the robot is using."

### Declaration order matters

MagicBot creates components in the order they are declared, and it also **executes** them in that order. Components that depend on other components (like `super_scorer` depends on `intake`, `hopper`, and `shooter`) must be declared **after** their dependencies.

That's why `super_scorer` is listed last — it needs the other three to exist first.

---

# 3) createObjects() — Setting Up Input Devices

The `createObjects()` method is called once when the robot starts up. This is where we create things that are NOT components — like Xbox controllers and slew rate limiters.

```python
    def createObjects(self) -> None:
        self.driver_controller = wpilib.XboxController(
            constants.kDriverControllerPort
        )

        self.operator_controller = wpilib.XboxController(
            constants.kOperatorControllerPort
        )

        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)
```

### What's new here

The **operator controller** is the new addition. This is a second Xbox controller plugged into the Driver Station on port 1. The operator sits next to the driver and controls the scoring system while the driver focuses on driving.

| Controller | Port | Who uses it | What it controls |
|------------|------|-------------|------------------|
| Driver controller | 0 | The driver | Driving, rotating, D-pad movement |
| Operator controller | 1 | The operator | Intake, shooting, emergency stop |

### What are slew rate limiters?

The slew rate limiters smooth out joystick inputs so the robot doesn't jerk around. The number `3` means the value can change by 3 units per second max. So going from stopped to full speed takes about 1/3 of a second. These are only used for driving — the scoring system doesn't need them.

### Where does kOperatorControllerPort come from?

It's defined in `constants.py`:
```python
kDriverControllerPort = 0
kOperatorControllerPort = 1
```

Port numbers match what's shown in the FRC Driver Station software. Make sure your second controller is plugged in and shows up on port 1.

---

# 4) teleopPeriodic() — Reading Buttons and Controlling the Robot

`teleopPeriodic()` is called every 20ms (50 times per second) during the driver-controlled period of the match. This is where we read controller inputs and tell components what to do.

The first part handles driving (already existed). The new part is the **operator controls** section at the bottom.

### The Operator Controls Section

```python
        if self.operator_controller.getXButtonPressed():
            self.super_scorer.stop_all()
        elif self.operator_controller.getAButton():
            self.super_scorer.intake_ball()
        elif self.operator_controller.getBButton():
            self.super_scorer.shoot_ball()
```

### Button mapping explained

| Button | Method | Behavior | Type |
|--------|--------|----------|------|
| **A** (held) | `intake_ball()` | Opens the intake arm, runs the roller and hopper to collect a ball | `getAButton()` — returns `True` while held |
| **B** (held) | `shoot_ball()` | Spins up the flywheels, feeds the ball when ready | `getBButton()` — returns `True` while held |
| **X** (pressed) | `stop_all()` | Emergency stop — halts everything immediately | `getXButtonPressed()` — returns `True` only once per press |

### Why "held" vs "pressed" matters

- **`getAButton()`** and **`getBButton()`** return `True` **every cycle** while the button is held down. This is important because MagicBot's `will_reset_to` variables reset every cycle. If we only detected a single press, the command would only last for one 20ms cycle (way too short to do anything).

- **`getXButtonPressed()`** returns `True` **only once** — the first cycle the button goes down. This is perfect for the emergency stop because we want it to trigger once, set the state to IDLE, and stay there.

### Why we check X first

Notice the `if/elif` chain starts with the X button (emergency stop). This is intentional — if the operator presses X at the same time as A or B, the stop always wins. Safety first!

### How the flow works

Here's what happens when the operator holds the A button:

1. `teleopPeriodic()` runs (every 20ms)
2. `getAButton()` returns `True` (button is held)
3. `self.super_scorer.intake_ball()` is called
4. Inside the Super Scorer, `_want_intake` is set to `True`
5. MagicBot calls `super_scorer.execute()`
6. The state machine sees `_want_intake` is `True` and transitions to INTAKING
7. The Super Scorer calls `intake.open_arm()`, `intake.run_roller()`, `hopper.feed_from_intake()`
8. MagicBot calls `intake.execute()` and `hopper.execute()`
9. The motors actually spin

When the operator releases the A button:
1. `getAButton()` returns `False`
2. `intake_ball()` is NOT called
3. `_want_intake` resets to `False` (via `will_reset_to`)
4. The state machine transitions to LOADED
5. Everything stops safely

---

# 5) The Full robot.py File

Here is the complete `robot.py` with all scoring system integration:

```python
import wpilib
import wpimath
import wpimath.filter
import magicbot

from components.swerve_drive import SwerveDrive
from components.intake import Intake
from components.hopper import Hopper
from components.shooter import Shooter
from components.thunder_vikes_super_scorer import ThunderVikesSuperScorer
import constants


class SwerveRobot(magicbot.MagicRobot):

    swerve_drive: SwerveDrive

    intake: Intake
    hopper: Hopper
    shooter: Shooter
    super_scorer: ThunderVikesSuperScorer

    def createObjects(self) -> None:
        self.driver_controller = wpilib.XboxController(
            constants.kDriverControllerPort
        )

        self.operator_controller = wpilib.XboxController(
            constants.kOperatorControllerPort
        )

        self.x_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.y_speed_limiter = wpimath.filter.SlewRateLimiter(3)
        self.rot_limiter = wpimath.filter.SlewRateLimiter(3)

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        # --- Driver controls (swerve drive) ---

        if self.driver_controller.getStartButtonPressed():
            self.swerve_drive.zero_heading()

        if self.driver_controller.getXButton():
            self.swerve_drive.set_x_formation()
            return

        pov = self.driver_controller.getPOV()

        if pov != -1:
            import math
            pov_rad = math.radians(pov)

            dpad_x = math.cos(pov_rad) * (constants.kDpadSpeed / constants.kMaxSpeed)
            dpad_y = -math.sin(pov_rad) * (constants.kDpadSpeed / constants.kMaxSpeed)

            self.swerve_drive.set_drive_command(
                dpad_x,
                dpad_y,
                0.0,
                True,
                False,
            )
            return

        x_speed = -self.x_speed_limiter.calculate(
            wpimath.applyDeadband(
                self.driver_controller.getLeftY(), constants.kDriveDeadband
            )
        )

        y_speed = -self.y_speed_limiter.calculate(
            wpimath.applyDeadband(
                self.driver_controller.getLeftX(), constants.kDriveDeadband
            )
        )

        rot = -self.rot_limiter.calculate(
            wpimath.applyDeadband(
                self.driver_controller.getRightX(), constants.kDriveDeadband
            )
        )

        self.swerve_drive.set_drive_command(
            x_speed, y_speed, rot, True, True
        )

        # --- Operator controls (scoring system) ---

        if self.operator_controller.getXButtonPressed():
            self.super_scorer.stop_all()
        elif self.operator_controller.getAButton():
            self.super_scorer.intake_ball()
        elif self.operator_controller.getBButton():
            self.super_scorer.shoot_ball()

    def autonomousInit(self) -> None:
        pass

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    SwerveRobot.main()
```

---

# 6) How MagicBot Wires It All Together

Let's walk through exactly what happens when the robot starts up. This is the "big picture" of how MagicBot connects everything.

### Step 1: Robot powers on

Python runs `SwerveRobot.main()` which starts the MagicBot framework.

### Step 2: MagicBot reads the class-level annotations

MagicBot looks at:
```python
swerve_drive: SwerveDrive
intake: Intake
hopper: Hopper
shooter: Shooter
super_scorer: ThunderVikesSuperScorer
```

It knows it needs to create one instance of each of these classes.

### Step 3: MagicBot creates the components (in order)

1. Creates a `SwerveDrive` instance, stores it as `self.swerve_drive`
2. Creates an `Intake` instance, stores it as `self.intake`
3. Creates a `Hopper` instance, stores it as `self.hopper`
4. Creates a `Shooter` instance, stores it as `self.shooter`
5. Creates a `ThunderVikesSuperScorer` instance, stores it as `self.super_scorer`

### Step 4: MagicBot does dependency injection

The `ThunderVikesSuperScorer` class declares:
```python
intake: Intake
hopper: Hopper
shooter: Shooter
```

MagicBot sees these and fills them in:
- `super_scorer.intake` = the same `Intake` instance from step 2
- `super_scorer.hopper` = the same `Hopper` instance from step 3
- `super_scorer.shooter` = the same `Shooter` instance from step 4

Now the Super Scorer can call methods on the intake, hopper, and shooter.

### Step 5: MagicBot calls createObjects()

Our `createObjects()` runs, creating the Xbox controllers and slew rate limiters.

### Step 6: MagicBot calls setup() on each component

1. `swerve_drive.setup()` — configures the swerve modules
2. `intake.setup()` — creates and configures the arm and roller motors
3. `hopper.setup()` — creates and configures the 3 conveyor motors
4. `shooter.setup()` — creates and configures the feeder and flywheel motors
5. `super_scorer.setup()` — initializes the state machine to IDLE

### Step 7: During teleop, every 20ms

1. MagicBot calls `teleopPeriodic()` — we read buttons and set commands
2. MagicBot calls `execute()` on each component in order:
   - `swerve_drive.execute()` — drives the wheels
   - `intake.execute()` — runs arm/roller motors
   - `hopper.execute()` — runs conveyor motors
   - `shooter.execute()` — runs feeder/flywheel motors
   - `super_scorer.execute()` — runs the state machine and coordinates everything
3. MagicBot resets all `will_reset_to` variables back to their defaults

This cycle repeats 50 times per second until the match ends.

---

# 7) Common Mistakes and How to Avoid Them

### Mistake 1: Forgetting to declare a component

If you forget to add `intake: Intake` in `robot.py`, the Super Scorer will crash because MagicBot can't inject it.

**Fix:** Make sure every component is declared as a class-level annotation in `SwerveRobot`.

### Mistake 2: Wrong variable names

If `robot.py` declares `my_intake: Intake` but `ThunderVikesSuperScorer` expects `intake: Intake`, the injection will fail.

**Fix:** The variable name in `robot.py` must match the variable name in any component that needs it.

### Mistake 3: Wrong declaration order

If you declare `super_scorer` before `intake`, `hopper`, and `shooter`, MagicBot might not inject them correctly.

**Fix:** Always declare dependencies before the components that use them.

### Mistake 4: Creating motors in createObjects()

MagicBot components create their own motors in `setup()`. Don't create motors in `createObjects()` — that's only for input devices and non-component objects.

### Mistake 5: Using getAButtonPressed() instead of getAButton()

`getAButtonPressed()` only returns `True` for one cycle. Since `will_reset_to` resets every cycle, the intake would only run for 20ms (way too short). Use `getAButton()` (held) for continuous actions.

---

# 8) Testing in Simulation

To test everything works:

```bash
cd RobotMain
python -m robotpy sim --nogui
```

If everything is set up correctly, you should see all components initialize without errors. The simulation will create all the SparkMax motor controllers and the robot will be ready to accept controller inputs.

### What to look for
- No import errors (all component files exist and import correctly)
- No missing constant errors (all CAN IDs and speeds are defined in `constants.py`)
- All components initialize their motors successfully
- The robot enters teleop mode without crashing

---

# 9) Quick Reference — Control Scheme

```
DRIVER CONTROLLER (Port 0):
  Left Stick    → Drive forward/backward and strafe left/right
  Right Stick   → Rotate the robot
  D-Pad         → Precise movement at 1 m/s
  Start Button  → Reset gyroscope heading
  X Button      → Lock wheels in X formation

OPERATOR CONTROLLER (Port 1):
  A Button (hold)    → Intake a ball (open arm, run roller + hopper)
  B Button (hold)    → Shoot a ball (spin up flywheels, feed)
  X Button (press)   → Emergency stop (halt everything)
```

---

# 10) Summary — What We Did

In this guide, we:

1. **Added imports** for the four new component classes (Intake, Hopper, Shooter, ThunderVikesSuperScorer)
2. **Declared components** as class-level type annotations so MagicBot creates and manages them
3. **Added an operator controller** on port 1 for the scoring system
4. **Added button bindings** in `teleopPeriodic()` to connect controller buttons to Super Scorer methods
5. **Learned how MagicBot wires everything together** — from creating components to dependency injection to the execute loop

The key takeaway: `robot.py` is the **glue** that connects human inputs (controllers) to robot actions (components). MagicBot handles all the wiring and safety features automatically. Our job is just to declare what we want and map buttons to actions.
