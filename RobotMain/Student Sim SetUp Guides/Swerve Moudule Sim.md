# Learning `swerve_module_sim.py` (Simulated Swerve Module)

This guide teaches you how to build a **simulation-only swerve module** in Python, step by step.

A **swerve module** is one “wheel unit” on a swerve robot. A real module has:
- a **drive motor** (makes the wheel roll forward/backward)
- a **turn motor** (rotates the wheel to point a direction)

In simulation, we don’t have real motors.  
So we create a class that **pretends** to be a module:

✅ it remembers the **target speed** and **target angle** we asked for  
✅ it shows those values on the dashboard so we can debug  
❌ it does not control real hardware

The point is learning the *logic* and creating a clean interface we can reuse later.

---

## What this module needs to do

We want this class to support two actions:

1) **Set the target** for the module:
```python
module.set(speed_mps, angle_rad)
````

2. **Show values to the dashboard** (so we can see what’s happening):

```python
module.execute()
```

That’s it.

---

# Step 1 — Imports (use libraries we need)

```python
import math
import wpilib
```

### Why do we need these?

* `math` gives us:

  * `math.pi` (π)
  * `math.degrees()` (radians → degrees conversion)

* `wpilib` gives us:

  * `SmartDashboard.putNumber(...)`
    (a way to show numbers while the robot code is running)

Think of SmartDashboard as a “debug window”.

---

# Step 2 — Write a helper function to keep angles “normal”

Angles can grow forever if you keep adding to them.
Example: if code keeps turning, you might end up with something like `23.5 radians` which is confusing.

So we create a helper that always keeps angles between:

* **-π and +π** radians

This makes angles easier to compare and debug.

```python
def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad
```

### What is happening here?

* A full circle is `2π` radians
* If the angle is too big (> π), we subtract `2π` until it’s back in range
* If it’s too small (< -π), we add `2π` until it’s back in range

### Example

* `3.5 rad` is bigger than π (~3.14159)

  * subtract `2π` → now it becomes a negative angle (same direction, easier to read)

This is like converting:

* 370° → 10°
* 350° → -10°

Same direction, simpler number.

---

# Step 3 — Create the class

A class is like a “blueprint” for a module.

```python
class SwerveModuleSim:
```

We name it `SwerveModuleSim` because this version is **for simulation**.

---

## Step 3.1 — Docstring (plain English description)

```python
"""
A simulation-only swerve module.
Stores the commanded speed/angle and publishes them to SmartDashboard.
"""
```

This is just a note to humans reading the file. It explains what the class is for.

---

# Step 4 — `__init__`: store the module name and starting values

```python
def __init__(self, name: str):
    self.name = name
    self.speed_mps = 0.0
    self.angle_rad = 0.0
```

### What is `__init__`?

This runs once when you create the module, like:

```python
module = SwerveModuleSim("FL")
```

### What are these variables?

* `self.name`

  * Just a label, like `"FL"` for front-left
  * Helps organize dashboard values

* `self.speed_mps`

  * The current *commanded* wheel speed in **meters per second**
  * Starts at 0 because we aren’t driving yet

* `self.angle_rad`

  * The current *commanded* wheel angle in **radians**
  * Starts at 0 by default

**Important:** In simulation, these aren’t “measured” values — they are “what we told it to do”.

---

# Step 5 — `set()`: accept new target commands

```python
def set(self, speed_mps: float, angle_rad: float):
    self.speed_mps = speed_mps
    self.angle_rad = wrap_to_pi(angle_rad)
```

This is the “main input” function of the module.

### What happens here?

* We store the target speed directly
* We store the target angle, but first we **wrap it** so it stays between -π and +π

### Why wrap the angle here?

Because whoever calls `set()` might give you:

* 7.2 radians
* -9.0 radians
* etc.

We want the module to always store a clean, easy-to-read angle.

---

# Step 6 — `execute()`: publish values to SmartDashboard

```python
def execute(self):
    wpilib.SmartDashboard.putNumber(f"{self.name}/speed_mps", self.speed_mps)
    wpilib.SmartDashboard.putNumber(f"{self.name}/angle_deg", math.degrees(self.angle_rad))
```

### Why have `execute()`?

Robot code runs in a loop ~50 times per second.

We use `execute()` to:

* repeatedly publish numbers so the dashboard stays updated

### What will students see?

If `name` is `"FL"`:

* `FL/speed_mps`
* `FL/angle_deg`

### Why convert radians → degrees here?

Degrees are easier for humans to understand quickly:

* 0° = forward
* 90° = left
* 180° = backward
* -90° = right

---

# Final goal: students should be able to write this file themselves

By the end, you should be able to recreate this module from memory:

✅ import the right libraries
✅ write the angle wrapping helper
✅ store name/speed/angle
✅ implement `set()`
✅ implement `execute()` to show values

---

## Quick self-check questions

1. What does `speed_mps` represent?
2. Why do we wrap angles to -π..+π?
3. What is the difference between **commanded** values and **measured** values?
4. Why do we show `angle_deg` instead of `angle_rad` on the dashboard?

---



