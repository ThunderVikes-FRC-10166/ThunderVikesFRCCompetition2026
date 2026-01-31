# Updating `robot.py` for the REAL Robot 
## Using the new `SwerveDrive` + `SwerveModule` components (field-oriented + odometry + AprilTags)

This guide shows students how to take your existing **simulation robot.py** and make a **real-robot robot.py** that works with the new component guides:

- `RobotMain/components/swervemodule.py` → `class SwerveModule`
- `RobotMain/components/swervedrive.py` → `class SwerveDrive` (FIELD-ORIENTED)

✅ We will keep things beginner-simple and explain **why** every change exists.

---

# 0) How we will manage SIM vs REAL files (simple workflow)

Right now your `robot.py` is the SIM version.

We want two versions saved:

### A) Simulation robot file
- Keep your current file as:
```
RobotMain/robot_sim.py
```

### B) Real robot file
- Create a new file:
```
RobotMain/robot_real.py
```

### How to switch
The roboRIO always runs a file named **`robot.py`**.

So when you want to deploy:

- If you want SIM:
  - rename `robot_sim.py` → `robot.py`
- If you want REAL ROBOT:
  - rename `robot_real.py` → `robot.py`

✅ This is a simple “rename to robot.py” workflow students can follow.

---

# 1) What changes between SIM and REAL robot.py?

Your SIM robot.py did:

- Imports `SwerveDriveSim`
- Does joystick math
- Calls `self.swerve.drive(vx, vy, omega)`

Your REAL robot.py must ALSO:

- Import the REAL `SwerveDrive` component
- Create real hardware objects:
  - 8 motors (drive+turn per module)
  - 4 absolute encoders
  - navX2 gyro is created inside the drivetrain component (as in the guide)
- Provide those hardware objects so MagicBot can inject them into the correct components

The teleop joystick code can stay **almost the same**.

---

# 2) IMPORTANT: Where your components live (absolute import path)

Your project layout (what we assume):

```
RobotMain/
  robot_sim.py
  robot_real.py
  robot.py              <-- whichever one you rename into place
  components/
    swervemodule.py     <-- SwerveModule class
    swervedrive.py      <-- SwerveDrive class
```

Because your root package is `RobotMain`, your imports should be ABSOLUTE like:

```python
from RobotMain.components.swervedrive import SwerveDrive
from RobotMain.components.swervemodule import SwerveModule
```

---

# 3) The MagicBot injection problem (and the simple naming solution)

## What MagicBot needs
Your `SwerveDrive` class has these injected module objects:

- `fl`, `fr`, `bl`, `br`

And each `SwerveModule` needs injected hardware fields:

- `drive_motor`
- `turn_motor`
- `abs_encoder`

## How do we inject 4 different sets of hardware?
We give each module its own set of hardware variables in `robot_real.py`
using a **prefix**:

- `fl_drive_motor`, `fl_turn_motor`, `fl_abs_encoder`
- `fr_drive_motor`, `fr_turn_motor`, `fr_abs_encoder`
- ... etc

Then MagicBot can match:
- module name (`fl`) + field name (`drive_motor`) → `fl_drive_motor`

✅ This keeps the module class simple (it still just says `drive_motor`, etc.)
✅ Students can clearly see which hardware belongs to which module

---

# 4) Create the REAL file: `RobotMain/robot_real.py`

Below is a **student-friendly template**.
Students should type it in and replace the CAN IDs / DIO ports.

> Note: This is not “final perfect competition code” — it’s a clean working starting point.

---

## Step 4.1 — Imports (REAL robot needs these)

```python
import magicbot
import wpilib
import math

from ntcore import NetworkTableInstance

import rev

# Real drivetrain component (field-oriented + estimator)
from RobotMain.components.swervedrive import SwerveDrive

# Optional: only for type hints on module names (not required for runtime)
from RobotMain.components.swervemodule import SwerveModule
```

Why:
- `rev` is needed to create Spark MAX motor objects
- `SwerveDrive` is the new drivetrain component

---

## Step 4.2 — Keep your deadband helper (same as SIM)

```python
def deadband(x: float, db: float = 0.08) -> float:
    return 0.0 if abs(x) < db else x
```

---

## Step 4.3 — Robot class with component injection

We replace `SwerveDriveSim` with the real `SwerveDrive`.

```python
class MyRobot(magicbot.MagicRobot):
    swerve: SwerveDrive

    def createObjects(self):
        # Driver controller
        self.driver = wpilib.XboxController(0)

        # Limelight table (we will still print/debug like before)
        self.ll = NetworkTableInstance.getDefault().getTable("limelight")

        # -------------------------
        # Driver scaling constants
        # -------------------------
        # These are "how fast should the robot be allowed to go"
        # Students can tune these later.
        self.max_speed_mps = 4.5          # typical swerve top speed
        self.max_omega_radps = 2.5 * math.pi  # about 1.25 rotations/sec

        # -------------------------
        # CREATE HARDWARE for ALL 4 modules
        # -------------------------
        # Replace CAN IDs and DIO channels with YOUR real wiring.
        #
        # CAN IDs (example):
        #   drive motors: 1,2,3,4
        #   turn motors:  5,6,7,8
        #
        # DIO ports (example):
        #   abs encoders: 0,1,2,3
        #
        # IMPORTANT: Spark MAX motor type for NEO/NEO550 is Brushless.
        motor_type = rev.SparkLowLevel.MotorType.kBrushless

        # ---- Front Left (fl) ----
        self.fl_drive_motor = rev.SparkMax(1, motor_type)
        self.fl_turn_motor  = rev.SparkMax(5, motor_type)
        self.fl_abs_encoder = wpilib.DutyCycleEncoder(0)

        # ---- Front Right (fr) ----
        self.fr_drive_motor = rev.SparkMax(2, motor_type)
        self.fr_turn_motor  = rev.SparkMax(6, motor_type)
        self.fr_abs_encoder = wpilib.DutyCycleEncoder(1)

        # ---- Back Left (bl) ----
        self.bl_drive_motor = rev.SparkMax(3, motor_type)
        self.bl_turn_motor  = rev.SparkMax(7, motor_type)
        self.bl_abs_encoder = wpilib.DutyCycleEncoder(2)

        # ---- Back Right (br) ----
        self.br_drive_motor = rev.SparkMax(4, motor_type)
        self.br_turn_motor  = rev.SparkMax(8, motor_type)
        self.br_abs_encoder = wpilib.DutyCycleEncoder(3)
```

### Why we do this in `createObjects()`
MagicBot wants all hardware created here ONE TIME.
Then it can inject those objects into components.

---

## Step 4.4 — Teleop code (field-oriented from day 1)

Your drivetrain guide already makes `swerve.drive(vx, vy, omega)` field-oriented.

So in teleop:
- `vx` means FIELD forward
- `vy` means FIELD left

This is the same joystick mapping you already use (just remember the meaning).

```python
    def teleopPeriodic(self):
        # Xbox inputs
        x = -self.driver.getLeftY()    # joystick forward/back
        y = -self.driver.getLeftX()    # joystick left/right strafe
        rot = -self.driver.getRightX() # joystick rotate

        # Deadband (ignore tiny joystick noise)
        x = deadband(x)
        y = deadband(y)
        rot = deadband(rot)

        # Square inputs for finer control at low speed
        x = math.copysign(x * x, x)
        y = math.copysign(y * y, y)
        rot = math.copysign(rot * rot, rot)

        # Convert joystick [-1..1] into real units
        vx = x * self.max_speed_mps
        vy = y * self.max_speed_mps
        omega = rot * self.max_omega_radps

        # FIELD-ORIENTED drive command
        self.swerve.drive(vx, vy, omega)

        # Optional: print Limelight debug like before
        tv = self.ll.getNumber("tv", 0)
        if tv >= 1:
            tid = int(self.ll.getNumber("tid", -1))
            tx = self.ll.getNumber("tx", 0.0)
            botpose = self.ll.getNumberArray("botpose_wpiblue", [])
            print(f"[robot] sees tag tid={tid} tx={tx:.1f} botpose={botpose}")
```

---

## Step 4.5 — Run the robot

```python
if __name__ == "__main__":
    wpilib.run(MyRobot)
```

---

# 5) What you do NOT need to create in robot_real.py

Because you already added it in `SwerveDrive.setup()` (from the guide):

- navX2 gyro (created inside `SwerveDrive`)
- pose estimator (created inside `SwerveDrive`)
- Limelight table (created inside `SwerveDrive` too — your teleop debug uses it separately, which is fine)

---

# 6) Checklist before deploying to the real robot

### Wiring + IDs
- CAN IDs match your Spark MAX wiring
- DIO ports match your encoder wiring
- Motors are Brushless

### Module offsets
- Each module needs its own `abs_offset_rad` value (from the module calibration step)

### Field-oriented sanity test
- Point robot forward
- Push joystick forward → robot drives toward the same field direction
- Rotate robot 90 degrees and push forward → robot strafes to keep field-forward

---

# 7) What comes next (after it boots)

Once the robot boots and teleop drives:
1) Verify yaw is correct (navX)
2) Verify each module turns correctly
3) Verify odometry pose numbers change on dashboard
4) Verify vision updates happen when tags are seen

---

# 8) QUICK SUMMARY (students memorize this)

- `robot_real.py` creates hardware objects
- MagicBot injects them into:
  - 4 module components (`fl`, `fr`, `bl`, `br`)
  - drivetrain component (`swerve`)
- `teleopPeriodic()` only sends **commands**
- The drivetrain component applies motors + updates pose

