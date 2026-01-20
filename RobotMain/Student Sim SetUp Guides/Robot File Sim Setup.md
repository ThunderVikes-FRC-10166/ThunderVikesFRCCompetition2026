# Learning `robot.py` (MagicBot Robot + Xbox Controller Input)

This guide teaches you how to write the main robot file: **`robot.py`**.

This is the file that WPILib actually runs. It is responsible for:

✅ starting the robot program  
✅ creating joystick/controller objects  
✅ reading driver input during Teleop  
✅ sending movement commands to our swerve drive code  

At this stage we are using **SwerveDriveSim** (simulation version), but later the same robot code will work with the real robot drive once hardware is ready.

---

# What is `robot.py`?

Think of `robot.py` as the robot’s **main entry point**.

If you asked:

> “What file is the robot *actually running*?”

The answer is:
✅ `robot.py`

---

# Step 1 — Imports

At the top of the file, we import the libraries we need:

```python
import math
import wpilib
import magicbot

from components.swerve_drive_sim import SwerveDriveSim
````

### What each import does

* `math`

  * used for `math.copysign()` which helps square joystick values but keep the sign

* `wpilib`

  * FRC Python library
  * gives us things like `XboxController`

* `magicbot`

  * framework that helps us organize robot code into “components”

* `SwerveDriveSim`

  * our simulation swerve drive class (built earlier)

---

# Step 2 — Create a helper function: deadband

Joystick sticks are never perfectly centered.
Even if you are not touching the stick, it might output something like:

* 0.02
* -0.04

That can cause the robot to slowly drift.

To fix that, we use a deadband function.

### How to type the function

```python
def deadband(x: float, db: float = 0.08) -> float:
    return 0.0 if abs(x) < db else x
```

### What it means

* If input is small (within ±0.08), force it to zero
* Otherwise keep the value

✅ Prevents drift
✅ Makes driving smoother

---

# Step 3 — Create the robot class

In Python, a class starts like:

```python
class MyRobot(magicbot.MagicRobot):
```

This means:

* `MyRobot` is the main robot program
* it uses MagicBot

### Full class header students should type:

```python
class MyRobot(magicbot.MagicRobot):
    swerve: SwerveDriveSim
```

---

## Important: What is this line doing?

```python
swerve: SwerveDriveSim
```

This tells MagicBot:

> “This robot has a component named `swerve` of type `SwerveDriveSim`.”

MagicBot will automatically:

* create the component
* call its methods like `execute()` repeatedly

Even though we don’t manually create `self.swerve` in this file,
MagicBot handles it for us.

---

# Step 4 — Create robot objects in `createObjects()`

MagicBot expects a method named:

```python
def createObjects(self):
```

This runs once when the robot starts.

### Students should type:

```python
    def createObjects(self):
        self.driver = wpilib.XboxController(0)
```

### What is this doing?

This creates the Xbox controller on port 0:

* `0` means:

  * “first joystick/controller plugged into the computer”

---

# Step 5 — Teleop control: `teleopPeriodic()`

This method runs repeatedly in teleop (many times per second).

You type it like:

```python
    def teleopPeriodic(self):
        ...
```

This is where we read controller values and command the drivebase.

---

## Step 5.1 — Read Xbox controller sticks

Xbox stick controls:

* **left stick** = translation (movement)
* **right stick X** = rotation (turning)

Students should type:

```python
        x = -self.driver.getLeftY()      # forward/back
        y = -self.driver.getLeftX()      # strafe
        rot = -self.driver.getRightX()   # rotate
```

### Why the negative sign `-`?

WPILib often reports:

* pushing the stick forward = negative number

But for humans it makes more sense:

* forward should be positive

So we invert it using `-`.

---

## Step 5.2 — Apply deadband to inputs

Students should type:

```python
        x = deadband(x)
        y = deadband(y)
        rot = deadband(rot)
```

This stops drift and accidental movement.

---

## Step 5.3 — Optional: square the joystick inputs

This makes driving easier for beginners:

* small stick movements become even smaller
* big stick movements still reach full speed

Students should type:

```python
        x = math.copysign(x * x, x)
        y = math.copysign(y * y, y)
        rot = math.copysign(rot * rot, rot)
```

### Why use `math.copysign()`?

Because `x * x` removes negative sign.

Example:

* `(-0.5 * -0.5) = 0.25`  (sign is lost)

So `copysign()` keeps the negative sign if needed.

---

## Step 5.4 — Convert joystick values into real units

Xbox stick values are between:

* -1.0 and +1.0

We convert those into real robot speeds using max speed constants:

Students should type:

```python
        vx = x * self.swerve.max_speed_mps
        vy = y * self.swerve.max_speed_mps
        omega = rot * self.swerve.max_omega_radps
```

Now:

* `vx` is in meters/second
* `vy` is in meters/second
* `omega` is in radians/second

---

## Step 5.5 — Command the swerve drive

Finally, send the speeds to swerve:

```python
        self.swerve.drive(vx, vy, omega)
```

This means:

> “SwerveDriveSim, please make the robot move with these speeds.”

---

# Step 6 — Running the robot

At the bottom we have:

```python
if __name__ == "__main__":
    wpilib.run(MyRobot)
```

### What does this do?

It tells Python:

* “If this file is being run directly, start WPILib with `MyRobot`.”

This is the line that actually starts the robot code.

---

# Full reference file (final result)

After building it step-by-step, your `robot.py` should look like this:

```python
# robot.py
import math
import wpilib
import magicbot

from components.swerve_drive_sim import SwerveDriveSim


def deadband(x: float, db: float = 0.08) -> float:
    return 0.0 if abs(x) < db else x


class MyRobot(magicbot.MagicRobot):
    swerve: SwerveDriveSim

    def createObjects(self):
        self.driver = wpilib.XboxController(0)

    def teleopPeriodic(self):
        # Xbox: left stick = translation, right stick X = rotation
        x = -self.driver.getLeftY()      # forward/back
        y = -self.driver.getLeftX()      # strafe
        rot = -self.driver.getRightX()   # rotate

        x = deadband(x)
        y = deadband(y)
        rot = deadband(rot)

        # Optional: square for finer low-speed control
        x = math.copysign(x * x, x)
        y = math.copysign(y * y, y)
        rot = math.copysign(rot * rot, rot)

        vx = x * self.swerve.max_speed_mps
        vy = y * self.swerve.max_speed_mps
        omega = rot * self.swerve.max_omega_radps

        self.swerve.drive(vx, vy, omega)


if __name__ == "__main__":
    wpilib.run(MyRobot)
```

---

# What students should understand after this file ✅

Students should be able to answer:

1. What is the job of `robot.py`?
2. What is `teleopPeriodic()` and how often does it run?
3. Why do we use deadband?
4. Why do we square joystick input?
5. Why do we multiply by `max_speed_mps` and `max_omega_radps`?

---

# Hint: How this will change for the real robot later

Right now we are using:

```python
from components.swerve_drive_sim import SwerveDriveSim
```

Later, when the robot hardware is ready, we will likely use:

```python
from components.swerve_drive_real import SwerveDriveReal
```

But the best part is:

* ✅ `robot.py` may barely change at all
* ✅ we still read joystick input the same way
* ✅ we still call `swerve.drive(vx, vy, omega)` the same way

Only the swerve drive component will change from simulation → real hardware.

That is a big reason why we are learning simulation first.


