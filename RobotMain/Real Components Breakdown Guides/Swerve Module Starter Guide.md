# Real Swerve Module (REV) — Step-by-step (FRC 2026 / RobotPy 2026)

This guide teaches how to write a **real swerve module** class that controls:
- a **drive motor** (wheel speed)
- a **turn motor** (wheel angle)
- an **absolute encoder** (wheel angle at boot)

We will compare it to our simulation module so you understand what changes in the real world.

✅ The goal is to understand the logic, not copy/paste.

---

# 1) Quick comparison: SIM module vs REAL module

## SIM: `SwerveModuleSim`
What it did:
- stored `speed_mps` and `angle_rad`
- displayed values on SmartDashboard

It **did not**:
- read sensors
- control motors
- do PID

**Sim version idea:**
> “Pretend the module did exactly what we commanded.”

---

## REAL: `SwerveModuleRev`
What it must do:
- read **absolute angle** from encoder (so we know wheel direction)
- command **turn motor** to reach target angle (PID)
- command **drive motor** to reach target speed (velocity control)
- optimize turning path (“shortest path” trick)

**Real version idea:**
> “Use sensors + motor control to actually reach the target.”

---

# 2) What hardware the module needs (inputs + outputs)

Each module needs:

## Outputs (we control)
1) `drive_motor` (REV Spark MAX + NEO)
2) `turn_motor`  (REV Spark MAX + NEO 550 or NEO)

## Inputs (we read)
3) `abs_encoder` (absolute angle sensor)
   - common: REV Through Bore Encoder (DutyCycle)
4) (optional) internal motor encoders
   - Spark MAX built-in encoders from `getEncoder()`

---

# 3) The design goal: keep the same API as simulation

In sim you had:

```python
def set(self, speed_mps: float, angle_rad: float):
    ...

def execute(self):
    ...
````

We will keep the same method names for real modules.

✅ That way `SwerveDrive` can call modules the same way in sim and real life.

---

# 4) Step-by-step: building `SwerveModuleRev`

## Step 4.1 — Start the file and imports

Create a new file:

* `components/swerve_module_rev.py`

Type:

```python
# components/swerve_module_rev.py
import math
import wpilib
import rev
```

Notes:

* `math` for pi, degrees/radians
* `wpilib` for SmartDashboard + DutyCycleEncoder types
* `rev` for Spark MAX motor controllers

---

## Step 4.2 — Add a helper: wrap angle to -pi..+pi

Type:

```python
def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad
```

Why?

* angles should stay “normal” and not grow forever
* swerve math is easier when angles wrap cleanly

---

## Step 4.3 — Write the class header + MagicBot injection slots

In MagicBot, for real hardware components we declare slots like this:

Type:

```python
class SwerveModuleRev:
    # MagicBot will inject these from robot.py createObjects()
    drive_motor = None
    turn_motor = None
    abs_encoder = None
```

**Important idea:**

* We do NOT create the motors inside this class
* `robot.py` creates hardware objects
* MagicBot injects them into these slots before `setup()`

---

## Step 4.4 — Add constants (wheel + gearing + encoder offset)

Each module needs constants.
Type these inside the class (usually set in `setup()` so it’s explicit):

We need:

* wheel diameter (meters)
* drive gear ratio
* turn gear ratio
* absolute encoder offset (radians)

**What is “offset”?**
Your absolute encoder gives a number, but “0 radians” in our code should mean:

> “wheel is pointing straight forward”

Because modules are mounted differently, each module needs its own offset.

We will store:

```python
    def setup(self):
        # --- Constants (edit for your robot) ---
        self.wheel_diameter_m = 0.1016  # 4 inches in meters
        self.drive_gear_ratio = 6.75    # motor revs per wheel rev (example)
        self.turn_gear_ratio = 12.8     # motor revs per module rev (example)

        self.abs_offset_rad = 0.0       # YOU will calibrate this per module
```

(We will add more inside setup next.)

---

## Step 4.5 — In `setup()`, get encoders + controllers (REV objects)

Now we add the “REV plumbing”:

### What we want

* drive encoder for measuring drive motor velocity
* turn encoder for measuring turn motor position
* a closed-loop controller for each

Type:

```python
        # --- REV sensors/controllers ---
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getEncoder()

        self.drive_ctrl = self.drive_motor.getClosedLoopController()
        self.turn_ctrl = self.turn_motor.getClosedLoopController()
```

### Note about REV API names (RobotPy 2026)

These are the most common RobotPy 2026 method names.
If PyCharm underlines them, use:

* Ctrl+Click (or Cmd+Click) on the class
* or type `self.drive_motor.` and look at autocomplete

---

## Step 4.6 — Convert between units (m/s and radians)

### 1) Speed conversion: m/s → wheel RPM → motor RPM

We will write a helper method inside the class:

```python
    def mps_to_motor_rpm(self, speed_mps: float) -> float:
        wheel_circumference = math.pi * self.wheel_diameter_m
        wheel_rps = speed_mps / wheel_circumference
        wheel_rpm = wheel_rps * 60.0
        motor_rpm = wheel_rpm * self.drive_gear_ratio
        return motor_rpm
```

What’s happening?

* speed (m/s) → wheel turns/second
* wheel turns/sec → wheel RPM
* wheel RPM → motor RPM using gear ratio

### 2) Angle handling: absolute encoder (0..1) → radians

We also need a method to read the absolute encoder:

```python
    def get_abs_angle_rad(self) -> float:
        # abs encoder returns 0.0..1.0 (one full rotation)
        turns = self.abs_encoder.getAbsolutePosition()
        rad = turns * 2.0 * math.pi

        # Apply offset so "0" means wheel forward
        rad = wrap_to_pi(rad - self.abs_offset_rad)
        return rad
```

---

## Step 4.7 — The “shortest path” optimization (very important)

If your target is far away (more than 90°), it’s faster to:

* rotate wheel 180°
* reverse drive direction

This makes swerve feel responsive.

Type this helper:

```python
    def optimize(self, target_angle_rad: float, target_speed_mps: float, current_angle_rad: float):
        target = wrap_to_pi(target_angle_rad)
        current = wrap_to_pi(current_angle_rad)
        error = wrap_to_pi(target - current)

        # If turning more than 90 degrees, flip direction
        if abs(error) > (math.pi / 2.0):
            target = wrap_to_pi(target + math.pi)
            target_speed_mps = -target_speed_mps

        return target, target_speed_mps
```

---

## Step 4.8 — Steering: how we command the turn motor

We want to command:

* a target steering angle

But the turn motor’s internal encoder might be in **motor rotations**, not radians.

There are two beginner-friendly options:

### Option A (recommended later): configure conversion factors

You configure the turn encoder so that:

* position is in radians
  Then you can send radians directly.

### Option B (simplest to understand now): do a manual conversion

We’ll keep it simple conceptually:

We’ll treat:

* module radians → module rotations → motor rotations

Because:

* 1 full module rotation = 2π radians
* motor rotations = module rotations * `turn_gear_ratio`

Add this method:

```python
    def angle_rad_to_turn_motor_rotations(self, angle_rad: float) -> float:
        module_rotations = angle_rad / (2.0 * math.pi)
        motor_rotations = module_rotations * self.turn_gear_ratio
        return motor_rotations
```

Now steering command becomes:

```python
    def set_angle(self, target_angle_rad: float):
        motor_rot = self.angle_rad_to_turn_motor_rotations(target_angle_rad)

        # Command Spark MAX to position target (motor rotations)
        self.turn_ctrl.setReference(
            motor_rot,
            rev.SparkClosedLoopController.ControlType.kPosition
        )
```

---

## Step 4.9 — Drive: how we command the drive motor

We want wheel speed in m/s, but Spark MAX velocity loop wants motor RPM.

So:

```python
    def set_speed(self, target_speed_mps: float):
        motor_rpm = self.mps_to_motor_rpm(target_speed_mps)

        self.drive_ctrl.setReference(
            motor_rpm,
            rev.SparkClosedLoopController.ControlType.kVelocity
        )
```

---

## Step 4.10 — The main `set(speed_mps, angle_rad)` method

This mirrors simulation, but now it:

* reads current angle
* optimizes path
* commands motors

Type:

```python
    def set(self, speed_mps: float, angle_rad: float):
        current = self.get_abs_angle_rad()
        opt_angle, opt_speed = self.optimize(angle_rad, speed_mps, current)

        self.set_angle(opt_angle)
        self.set_speed(opt_speed)
```

This is the “core” of the real module.

---

## Step 4.11 — `execute()` (dashboard debug)

In simulation, `execute()` published speed/angle.

In real life, it’s even more useful:

* show target angle/speed
* show current absolute angle

Type:

```python
    def execute(self):
        wpilib.SmartDashboard.putNumber("module/abs_angle_deg", math.degrees(self.get_abs_angle_rad()))
```

Later you can add:

* drive motor RPM from encoder
* turn motor position
* targets

---

# 5) The full “skeleton” file (structure reference)

This is NOT meant to be copied. It’s a reference of what the class contains.

Students should be able to explain each method:

* `setup()`
* `get_abs_angle_rad()`
* `optimize()`
* `set_angle()`
* `set_speed()`
* `set()`
* `execute()`

---

# 6) What’s missing right now (and why)

For a fully working real module, you will later add:

## A) REV configuration and PID tuning

You must configure PID gains for:

* drive velocity loop
* turn position loop

Without PID values, the controller will not behave correctly.

## B) “Seeding” the turn motor encoder on boot

Many teams do this:

* read absolute angle at startup
* set the turn motor encoder position to match it

That way the internal turn encoder and absolute encoder agree.

In REV config, this can be done by setting encoder position.

## C) Per-module encoder offset calibration

Each module gets its own `abs_offset_rad`.
You find it by:

* pointing wheel forward
* reading abs encoder angle
* storing that value as offset

---

# 7) How this differs from simulation (clear summary)

## Simulation:

* `set()` just stores values
* `execute()` just displays values

## Real:

* `set()` reads sensors + commands motors
* `execute()` is for debugging sensor readings and targets
* you must convert real units (m/s, rad) into motor units (RPM, rotations)
* you must tune PID values

---

# 8) How students can verify REV method names in RobotPy 2026

If your code errors because a method name is wrong:

✅ In PyCharm:

1. type `self.drive_motor.` and look at autocomplete
2. Ctrl+Click the class to open the stub/type hints
3. search for `getEncoder`, `getClosedLoopController`, `setReference`

This is a normal part of real robot programming.

---

# 9) Next steps (what we do after this works)

Once **one module** works:

1. make all 4 modules work the same way
2. build `SwerveDriveReal` to compute module states and call module `.set()`
3. connect gyro for field-oriented driving
4. tune and calibrate

The real drivebase will feel almost identical to sim because we kept the same API.

