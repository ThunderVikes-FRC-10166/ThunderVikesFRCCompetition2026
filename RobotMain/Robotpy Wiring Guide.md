# Real Robot `robot.py` (MagicBot Wiring) — FRC 2026 / RobotPy 2026  
### REV Spark MAX + navX + (future) Limelight

This guide teaches students how to write the **real robot** `robot.py` file using **MagicBot**.

✅ Goal: Students understand how to “wire” hardware into code (motors, encoders, gyro), and how MagicBot injection connects everything.

This is NOT about writing swerve math (we already did that).  
This is about:
- creating hardware objects in `createObjects()`
- injecting them into components
- understanding what will later be added for navX and Limelight

---

# 1) What `robot.py` is responsible for

Think of `robot.py` as the robot’s **main program**:

✅ creates hardware objects  
✅ reads the controller in teleop  
✅ calls your drive component (`swerve.drive(...)`)  
✅ sets up anything that must exist at robot startup  

In MagicBot, `robot.py` is also where **hardware injection starts**.

---

# 2) MagicBot injection (the most important idea)

MagicBot injects objects by **matching names**.

### Example
If your drive component has:

```python
class SwerveDriveReal:
    fl = None
    gyro = None
````

Then your robot must create these exact names:

```python
self.fl = ...
self.gyro = ...
```

MagicBot will automatically assign them.

✅ Same exact name = injection works
❌ different name = injection fails (and you get NoneType errors)

---

# 3) What hardware objects we create in `createObjects()`

For real swerve we need:

## A) Driver controller

* Xbox controller

## B) navX gyro

* read robot yaw (heading)

## C) Limelight (future vision)

* networktables values
* target detection / AprilTag info

## D) Swerve modules (4)

For each module we need:

* drive motor (Spark MAX)
* turn motor (Spark MAX)
* absolute encoder (DutyCycleEncoder)

Then we create 4 `SwerveModuleRev` objects and give them those motors/encoders using MagicBot injection.

---

# 4) File layout (recommended)

Your repo will typically have:

```
RobotMain/
  robot.py
  components/
    swerve_drive_real.py
    swerve_module_rev.py
```

---

# 5) Imports students should start with

At the top of `robot.py`, students type imports.

```python
import math
import wpilib
import magicbot

import rev

from wpilib import DutyCycleEncoder
```

For our components:

```python
from components.swerve_drive_real import SwerveDriveReal
from components.swerve_module_rev import SwerveModuleRev
```

For navX, many RobotPy installs use a navX library. Depending on your team’s install, it might be one of these:

* `from navx import AHRS`
* or a vendor-specific wrapper

**How to confirm in PyCharm:**
Search your venv for “AHRS” or “navx”.

---

# 6) Helpful helper functions (deadband)

Same idea as simulation.

Students type:

```python
def deadband(x: float, db: float = 0.08) -> float:
    return 0.0 if abs(x) < db else x
```

---

# 7) The main robot class

MagicBot robot class starts like:

```python
class MyRobot(magicbot.MagicRobot):
```

Then we declare the components we want MagicBot to manage:

```python
class MyRobot(magicbot.MagicRobot):
    swerve: SwerveDriveReal
```

This tells MagicBot:

* “create a `SwerveDriveReal` component and store it in `self.swerve`”

---

# 8) `createObjects()` — where hardware gets created

MagicBot expects a method called:

```python
def createObjects(self):
```

This runs ONCE at robot startup.

### Students should type:

```python
    def createObjects(self):
        # 1) Driver controller
        self.driver = wpilib.XboxController(0)
```

---

## 8.1 Create the navX gyro (real sensor)

Your swerve drive expects `self.gyro` to exist (injection slot name `gyro`).

So in `createObjects()` you create:

```python
        # 2) navX gyro (common setup)
        # NOTE: exact import/class may vary depending on installed navX library.
        # Example:
        # self.gyro = AHRS.create_spi()

        # For now, keep the idea:
        self.gyro = ...  # navX AHRS object
```

### What the gyro must provide

Our `SwerveDriveReal` expects something like:

```python
yaw = self.gyro.get_yaw()
```

So we will likely wrap navX with a small “adapter” class.

Example concept:

```python
from wpimath.geometry import Rotation2d

class NavXWrapper:
    def __init__(self, ahrs):
        self.ahrs = ahrs

    def get_yaw(self) -> Rotation2d:
        # navX usually returns degrees
        # WPILib wants Rotation2d (radians)
        return Rotation2d.fromDegrees(self.ahrs.getYaw())
```

Then in `createObjects()`:

```python
self.gyro = NavXWrapper(AHRS.create_spi())
```

✅ This keeps your swerve drive code clean.

---

## 8.2 Create the Limelight (vision) — future hook

Limelight communicates over NetworkTables.
In RobotPy, you can access:

```python
from ntcore import NetworkTableInstance
```

Then:

```python
inst = NetworkTableInstance.getDefault()
table = inst.getTable("limelight")
```

Common Limelight values:

* `tv` = target valid (0 or 1)
* `tx` = horizontal offset
* `ty` = vertical offset
* `ta` = target area

AprilTag-specific pipelines also publish info depending on Limelight mode.

For now, in `createObjects()` you can store:

```python
        self.limelight_table = ...  # NetworkTables table for limelight
```

We won’t use it yet, but we keep the hook.

---

## 8.3 Create Spark MAX motors for each module

Each Spark MAX needs:

* CAN ID
* motor type (NEO = brushless)

Students should define CAN IDs clearly.

Example pattern (your IDs will be different):

```python
        # Front Left CAN IDs
        fl_drive_id = 1
        fl_turn_id = 2
```

Then create motors:

```python
        self.fl_drive_motor = rev.SparkMax(fl_drive_id, rev.SparkMax.MotorType.kBrushless)
        self.fl_turn_motor  = rev.SparkMax(fl_turn_id,  rev.SparkMax.MotorType.kBrushless)
```

Repeat for FR/BL/BR.

---

## 8.4 Create absolute encoders (DutyCycleEncoder)

If using REV Through Bore Encoder wired to a roboRIO DIO port:

```python
        self.fl_abs_encoder = DutyCycleEncoder(0)  # DIO port number
```

Repeat for each module with correct DIO channels.

---

# 9) Create the module objects and inject hardware into them

There are two common ways.

## Option A (easy to understand): create module objects and assign hardware directly

This works for beginners and is very explicit.

Example:

```python
        self.fl = SwerveModuleRev()
        self.fl.drive_motor = self.fl_drive_motor
        self.fl.turn_motor = self.fl_turn_motor
        self.fl.abs_encoder = self.fl_abs_encoder
```

Repeat for FR/BL/BR.

✅ Then your `SwerveDriveReal` can have `fl/fr/bl/br` injected.

---

## Option B (more “MagicBot style”): use injection slots inside module components

This is a bit more advanced and we’ll teach it later.

For now, Option A is clearer.

---

# 10) Create / inject the SwerveDriveReal component needs

Your `SwerveDriveReal` declares:

```python
fl = None
fr = None
bl = None
br = None
gyro = None
```

So your robot must create:

```python
self.fl = ...
self.fr = ...
self.bl = ...
self.br = ...
self.gyro = ...
```

We already created `self.fl` etc as module objects.

So injection will work.

---

# 11) teleopPeriodic() — reading the Xbox and calling swerve.drive

This is nearly identical to your sim version.

Students type:

```python
    def teleopPeriodic(self):
        x = -self.driver.getLeftY()
        y = -self.driver.getLeftX()
        rot = -self.driver.getRightX()

        x = deadband(x)
        y = deadband(y)
        rot = deadband(rot)

        x = math.copysign(x * x, x)
        y = math.copysign(y * y, y)
        rot = math.copysign(rot * rot, rot)

        vx = x * self.swerve.max_speed_mps
        vy = y * self.swerve.max_speed_mps
        omega = rot * self.swerve.max_omega_radps

        self.swerve.drive(vx, vy, omega)
```

---

# 12) Full “teaching reference” outline (not copy/paste)

After you finish, your `robot.py` will have these sections:

1. Imports
2. deadband helper
3. `MyRobot` class
4. `createObjects()` creates:

   * Xbox controller
   * navX wrapper as `self.gyro`
   * Limelight table (optional future hook)
   * Spark MAX objects
   * DutyCycleEncoders
   * four `SwerveModuleRev` objects assigned as `self.fl`, `self.fr`, `self.bl`, `self.br`
5. `teleopPeriodic()` converts controller → vx/vy/omega → `swerve.drive(...)`
6. `wpilib.run(MyRobot)` at the bottom

---

# 13) Limelight “starter info” (how it will fit later)

Later, once you have pose estimation / auto-align, you will do something like:

* read limelight values in teleop/autonomous
* compute an “aim correction”
* adjust omega command or target pose

Example idea:

* if Limelight says the target is 10° to the right,
* command omega to rotate right until error is near 0.

Limelight is not required to drive swerve, but it’s extremely useful for:

* auto-alignment
* AprilTag localization
* aiming shooters

---

# 14) What students should understand after this

Students should be able to answer:

1. What goes in `createObjects()` and why?
2. Why do names like `self.fl` matter?
3. What does MagicBot injection do?
4. Why do we wrap navX into a `get_yaw()` method?
5. What will Limelight be used for later?

---

# 15) Next steps once robot hardware exists

1. Confirm CAN IDs for all Spark MAX controllers
2. Confirm DIO ports for all absolute encoders
3. Confirm navX is detected and yaw updates
4. Bring up one module at a time
5. Then bring up full swerve drive

