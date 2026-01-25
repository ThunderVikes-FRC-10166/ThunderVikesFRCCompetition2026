
# MagicBot Autonomous in SIM (RobotPy 2026) — Step-by-Step Guide for Students  
### Goal: Your autonomous shows up in the GUI, you select it, press Enable, and the robot drives (time-based).

This guide is written for students who are **new to Python**. We’ll go slowly, explain why each part exists, and show the exact files you must modify.

✅ We are focusing on **SIMULATOR first**.  
✅ We are focusing on **time-based** autonomous (drive for X seconds, then stop).  
✅ This approach will also work on the **real robot** later with the same structure (notes at the end).

---

## 0) What you’re building

In Teleop, joystick controls the robot.

In Autonomous, **your code** controls the robot.

Either way, your drivetrain is always driven by the same call:

```python
swerve.drive(vx_mps, vy_mps, omega_radps)
````

In SIM, you’ll see the robot move on the Field2d / sim view when you enable autonomous.

---

## 1) Required project structure

Your RobotMain folder must look like this:

```
RobotMain/
  robot.py
  physics.py
  components/
    swerve_drive_sim.py
    swerve_module_sim.py
  autonomous/
    __init__.py
    drive_box_auto.py
```

### Why do we need `autonomous/__init__.py`?

Python only treats a folder as a package if it contains `__init__.py`.
If you forget it, the sim GUI will usually show **AutonomousMode: None** only.

✅ Create the file:

* `RobotMain/autonomous/__init__.py`
* Leave it empty

---

## 2) The Autonomous class file (the mode students select in the GUI)

Create this file:

* `RobotMain/autonomous/drive_box_auto.py`

### 2.1 Imports

Type:

```python
import wpilib
```

### 2.2 Make the autonomous class

Type:

```python
class DriveBoxAuto:
```

### 2.3 Give it a name (so it appears in the GUI)

MagicBot discovers auto modes by looking for a class variable called `MODE_NAME`.

✅ Must be written like this (NO type annotation):

```python
    MODE_NAME = "Drive Box (SIM)"
```

Optional (recommended): auto-select it by default:

```python
    DEFAULT = True
```

### 2.4 Add the methods MagicBot calls

MagicBot calls these methods during Auto:

* `on_enable()` — runs once when Auto starts
* `on_iteration(time_elapsed)` — runs repeatedly while Auto is enabled
* `on_disable()` — runs once when Auto stops

Type the full class like this:

```python
import wpilib


class DriveBoxAuto:
    MODE_NAME = "Drive Box (SIM)"
    DEFAULT = True

    def __init__(self):
        # This will be filled in by robot.py (we do that in robotInit)
        self.robot = None

        self.timer = wpilib.Timer()
        self.state = "FORWARD_1"

    def on_enable(self):
        self.timer.restart()
        self.state = "FORWARD_1"

    def on_iteration(self, time_elapsed: float):
        # We will use our own timer because it’s easier for beginners to reset per step
        t = self.timer.get()

        # Safety: if robot isn’t attached yet, do nothing
        if self.robot is None:
            return

        # A simple "state machine" auto:
        # forward -> left -> forward -> stop
        if self.state == "FORWARD_1":
            self.robot.swerve.drive(1.0, 0.0, 0.0)  # 1 m/s forward
            if t > 1.5:
                self.state = "LEFT"
                self.timer.restart()

        elif self.state == "LEFT":
            self.robot.swerve.drive(0.0, 1.0, 0.0)  # 1 m/s left
            if t > 1.0:
                self.state = "FORWARD_2"
                self.timer.restart()

        elif self.state == "FORWARD_2":
            self.robot.swerve.drive(1.0, 0.0, 0.0)
            if t > 1.0:
                self.state = "DONE"

        else:
            # DONE state: stop
            self.robot.swerve.drive(0.0, 0.0, 0.0)

    def on_disable(self):
        if self.robot is not None:
            self.robot.swerve.drive(0.0, 0.0, 0.0)
```

### Why we use `self.robot.swerve` instead of `swerve: SwerveDriveSim` here

In some setups, injecting components directly into autonomous mode classes can be confusing or fail for beginners.

So we use a simpler rule:

✅ The robot always has the drivetrain at `robot.swerve`
✅ Autonomous modes get a reference to the robot (`mode.robot = robot`)
✅ Then autos can call `self.robot.swerve.drive(...)`

To make that happen, we must update `robot.py`.

---

## 3) Update `robot.py` so autonomous modes get a robot reference

This is the missing piece that makes `self.robot` stop being `None`.

### 3.1 Keep your normal drivetrain component annotation

Do NOT remove this from `robot.py`:

```python
class MyRobot(magicbot.MagicRobot):
    swerve: SwerveDriveSim
```

This is still correct and should remain.

---

### 3.2 Add `robotInit` + `_attach_robot_to_auto_modes`

Inside your `MyRobot` class (in `robot.py`), add:

```python
import wpilib
import magicbot

class MyRobot(magicbot.MagicRobot):
    # keep your existing component annotations
    # swerve: SwerveDriveSim

    def robotInit(self):
        super().robotInit()
        self._attach_robot_to_auto_modes()

    def _attach_robot_to_auto_modes(self):
        """
        Give each autonomous mode a reference to this robot instance.
        This allows autonomous code to call:
            self.robot.swerve.drive(...)
        """
        selector = getattr(self, "_automodes", None)
        if selector is None:
            selector = getattr(self, "autonomous", None)

        if selector is None:
            wpilib.reportWarning("No autonomous selector found; cannot attach robot to auto modes")
            return

        modes = getattr(selector, "modes", None)
        if not isinstance(modes, dict) or not modes:
            wpilib.reportWarning("Autonomous selector has no modes (did you create autonomous/__init__.py?)")
            return

        for mode in modes.values():
            mode.robot = self

        wpilib.reportWarning(f"Attached robot reference to {len(modes)} autonomous mode(s)")
```

### Why `robotInit`?

`robotInit()` runs once at robot startup (SIM and real robot).
By calling `_attach_robot_to_auto_modes()` there, we guarantee the autonomous classes get `mode.robot = self` before Auto starts.

✅ This works in SIM
✅ This also works on the real robot (the same robot lifecycle exists on the roboRIO)

---

## 4) How to run Autonomous in the simulator (student checklist)

### Step 1 — Start simulation

In your terminal (from the project folder), run:

```bash
robotpy sim
```

Leave it running.

### Step 2 — Open the sim GUI (halsim_gui window)

You should see:

* Robot State controls
* An **AutonomousMode** chooser

### Step 3 — Select the autonomous mode

In the GUI, find **AutonomousMode** and select:

* `Drive Box (SIM)`

If you only see “None”:

* confirm `RobotMain/autonomous/__init__.py` exists
* confirm your class has `MODE_NAME = "..."` (no `: str`)
* restart the sim after edits

### Step 4 — Enable Autonomous

In the GUI:

* set robot mode to **Autonomous**
* click **Enable**

### Step 5 — Confirm it’s working

You should see the robot:

* drive forward
* strafe left
* drive forward
* stop

If nothing moves:

* check sim output logs
* confirm you added `robotInit()` + `_attach_robot_to_auto_modes()` in `robot.py`
* confirm you see a warning like:

  * “Attached robot reference to X autonomous mode(s)”

---

## 5) Common mistakes (and what they look like)

### “AutonomousMode only shows None”

Usually means:

* missing `autonomous/__init__.py`
* missing `MODE_NAME`
* import error in the autonomous file

### “No autonomous modes selected”

Means:

* the dropdown is still “None”
* select your auto mode in the GUI

### `self.robot is None` inside autonomous

Means:

* you did not attach the robot reference in `robotInit`
* or the attach code didn’t find any modes (package not loaded)

---

## 6) Will this work on the real robot later?

✅ Yes, the same structure works:

* `robotInit` runs on the roboRIO
* autonomous modes still exist
* attaching `mode.robot = self` still works
* Auto runs the same way (just no sim GUI; Driver Station enables Auto)

What changes on real robot:

* `robot.swerve` will be your **real swerve component**, not the sim one
* motor commands go to hardware instead of physics simulation

What stays the same:

* the autonomous state machine logic
* the method names (`on_enable`, `on_iteration`, `on_disable`)
* selecting which auto to run (in real matches you typically set a default or use DS/dashboard chooser)

---

## 7) What students should do next

Once this works:

* create a second auto file (example: only drive forward)
* change times and see behavior
* add new states (turn, wait, etc.)
* practice debugging with print statements or SmartDashboard

Time-based autos are a perfect training tool before learning distance-based autos.

