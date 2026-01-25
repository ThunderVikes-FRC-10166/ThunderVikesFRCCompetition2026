
# MagicBot Autonomous Guide (FRC 2026) — Step-by-step for Beginners (RobotPy 2026)
### Works in Simulator first (SwerveDriveSim), then upgrades to real robot later

This guide teaches you how to build autonomous programs **one step at a time** using **MagicBot**:

1) Start with a **time-based** autonomous (easiest to debug)
2) Upgrade to a **distance-based** autonomous (more consistent)
3) Build a **state machine** auto: forward → left → forward
4) Learn different kinds of autonomous states (turning, waiting, aligning, scoring)
5) Understand how to *think about* maximizing points during Auto for the 2026 game

✅ We will write autos assuming you are running the **SIMULATOR** first.  
That means in the autonomous class we will type:

```python
swerve: SwerveDriveSim
````

Later on the real robot you will change it to:

```python
swerve: SwerveDriveReal
```

---

## 0) One important concept before we code

Whether it is Teleop or Autonomous, driving always comes down to calling:

```python
self.swerve.drive(vx_mps, vy_mps, omega_radps)
```

In Teleop:

* the driver joystick decides `vx, vy, omega`

In Auto:

* **your code** decides `vx, vy, omega`

Same drivetrain code. Different “who is controlling it”.

---

## 1) How MagicBot loads Autonomous modes

MagicBot looks for a Python package named `autonomous`.

✅ Create this folder:

```
RobotMain/
  robot.py
  autonomous/
    __init__.py
    drive_box_auto.py
```

### `autonomous/__init__.py`

Create the file and leave it empty.
It tells Python “this is a package”.

If you don’t have this folder/package, MagicBot may print:

> Cannot load the 'autonomous' package

---

## 2) Time-based vs Distance-based Auto

### A) Time-based Auto (easy to learn)

Example:

* “Drive forward for 1.5 seconds”

Pros: easiest to get working
Cons: not perfectly consistent (battery, carpet, wheel slip)

### B) Distance-based Auto (better for competition)

Example:

* “Drive forward 2.0 meters”

Pros: more consistent and repeatable
Cons: needs a way to measure distance (pose / odometry or encoders)

✅ We will do time-based first, then upgrade to distance-based.

---

# PART 1 — Time-based Auto (Starter version)

## 3) What a state machine is (beginner definition)

A **state machine** is just:

* a variable that stores “what step are we on?”

Example states:

* `"FORWARD_1"`
* `"LEFT"`
* `"FORWARD_2"`
* `"DONE"`

In each loop, you:

1. check the state
2. do the action
3. switch to the next state when it’s time

---

## 4) Create your first autonomous file

Create:
`autonomous/drive_box_auto.py`

At the top:

```python
import wpilib

from components.swerve_drive_sim import SwerveDriveSim
```

Why import `SwerveDriveSim`?
So you can type-hint it in the class. It helps beginners understand what `swerve` is.

---

## 5) Write the autonomous class (students should type this)

### 5.1 Class header + type hint for drivetrain (SIM version)

Type:

```python
class DriveBoxAuto:
    swerve: SwerveDriveSim
```

✅ This tells MagicBot:

* “This autonomous routine will use the robot’s swerve drive component.”

**Later (real robot):**
Change to:

```python
from components.swerve_drive_real import SwerveDriveReal

class DriveBoxAuto:
    swerve: SwerveDriveReal
```

---

### 5.2 `on_enable()` — runs once when Auto starts

Type:

```python
    def on_enable(self):
        self.timer = wpilib.Timer()
        self.timer.restart()
        self.state = "FORWARD_1"
```

What this does:

* makes a timer
* starts it at 0
* sets the first state

---

### 5.3 `execute()` — runs repeatedly during Auto

Type:

```python
    def execute(self):
        t = self.timer.get()

        if self.state == "FORWARD_1":
            # Drive forward for 1.5 seconds
            self.swerve.drive(1.0, 0.0, 0.0)  # 1 m/s forward
            if t > 1.5:
                self.state = "LEFT"
                self.timer.restart()

        elif self.state == "LEFT":
            # Strafe left for 1.0 seconds
            self.swerve.drive(0.0, 1.0, 0.0)  # 1 m/s left
            if t > 1.0:
                self.state = "FORWARD_2"
                self.timer.restart()

        elif self.state == "FORWARD_2":
            # Drive forward for 1.0 seconds
            self.swerve.drive(1.0, 0.0, 0.0)
            if t > 1.0:
                self.state = "DONE"

        else:
            # DONE: stop the robot
            self.swerve.drive(0.0, 0.0, 0.0)
```

✅ This creates a simple path:
Forward → Left → Forward → Stop

---

## 6) Why this works in the simulator right away

In your simulator setup:

* `robot.py` calls `swerve.drive(...)` for Teleop
* In Auto, this autonomous class calls `swerve.drive(...)` instead

`physics.py` reads `swerve.last_cmd` and moves the robot pose.

So as long as:

* your `SwerveDriveSim` stores `last_cmd`
* and `execute()` is being called

you’ll see the robot move.

---

# PART 2 — Distance-based Auto (More consistent)

Time-based is good for learning.
Distance-based is what teams prefer for real matches.

To do distance-based auto, you need to measure distance.

---

## 7) The simplest way to measure distance in SIM

In sim, we already track a robot pose in `physics.py` and publish it to Field2d.

For distance-based auto, we want the drivetrain to provide:

```python
def get_pose(self) -> Pose2d:
    ...
```

### What students should understand

* Pose = (x meters, y meters, heading)
* If we know pose at start and pose now, we can compute distance moved.

---

## 8) How to compute distance moved

If:

* start pose is `(x0, y0)`
* current pose is `(x1, y1)`

Distance moved is:

```python
dx = x1 - x0
dy = y1 - y0
dist = sqrt(dx*dx + dy*dy)
```

---

## 9) Distance-based version of the same state machine

### 9.1 Store “step start pose” when entering a state

In `on_enable()`:

```python
    def on_enable(self):
        self.state = "FORWARD_1"
        self.step_start_pose = self.swerve.get_pose()
```

### 9.2 In execute, drive until distance is reached

Example for forward 2.0 meters:

```python
        if self.state == "FORWARD_1":
            self.swerve.drive(1.0, 0.0, 0.0)

            current = self.swerve.get_pose()
            start = self.step_start_pose

            dx = current.X() - start.X()
            dy = current.Y() - start.Y()
            dist = (dx*dx + dy*dy) ** 0.5

            if dist >= 2.0:
                self.state = "LEFT"
                self.step_start_pose = self.swerve.get_pose()
```

Then for LEFT 1.0 meter:

```python
        elif self.state == "LEFT":
            self.swerve.drive(0.0, 1.0, 0.0)

            current = self.swerve.get_pose()
            start = self.step_start_pose

            dx = current.X() - start.X()
            dy = current.Y() - start.Y()
            dist = (dx*dx + dy*dy) ** 0.5

            if dist >= 1.0:
                self.state = "FORWARD_2"
                self.step_start_pose = self.swerve.get_pose()
```

And forward 1.0 again, then DONE.

✅ This produces repeatable movement based on meters, not seconds.

---

# PART 3 — What kinds of Autonomous “states” can you build?

A state machine becomes powerful when each state has:

* a clear job
* a clear “done condition”

Here are common state types:

## A) Drive-to-distance state

* ends when distance >= target

## B) Turn-to-angle state

* ends when yaw error is small
  (use navX in real robot, simple gyro in sim)

## C) Wait state

* ends when timer >= target
  (useful to pause before shooting)

## D) Align-to-target state (Limelight / AprilTags)

* ends when tx error is small (target centered)

## E) Action state (shooter/intake)

* ends when you’ve fired a ball or reached shooter RPM

---

# PART 4 — “Maximizing points in Auto” (what students should focus on)

Even without knowing every detail, students should understand this general strategy:

### In Auto, you want:

✅ something consistent
✅ something safe
✅ something that helps the alliance

Examples of early-season “high value” auto goals:

* leave the starting area reliably
* reach a scoring spot reliably
* align to an AprilTag quickly
* score one game piece reliably

Later, more advanced:

* score multiple pieces
* do an auto-climb / auto-task
* coordinate with alliance partners

**Key beginner message:**

> Consistency beats fancy. A simple auto that works every match is better than a complex one that fails.

---

# PART 5 — Running Autonomous in SIM

1. Start simulation:

```bash
robotpy sim
```

2. Use the sim GUI to:

* set robot mode to Autonomous
* enable

3. Watch Field2d:

* robot should move in your forward-left-forward path

If it doesn’t:

* check the sim output log for autonomous package loading messages
* confirm your `autonomous/` folder exists and has `__init__.py`

---

# PART 6 — What changes for the real robot?

When moving from SIM to real robot:

## In your autonomous class:

Change:

```python
swerve: SwerveDriveSim
```

to:

```python
swerve: SwerveDriveReal
```

## For distance-based auto:

* SIM pose might come from physics/Field2d
* REAL pose should come from **odometry**:

  * wheel encoder distances
  * gyro yaw
  * (optionally) AprilTag vision pose estimates

But the state machine logic stays the same:

* start pose
* drive until distance reached
* switch states

---

## Final reminder

Autonomous is just “driving without the driver”.

If you can:

* command a drivetrain
* measure when you’ve reached a goal
* switch steps

You can build any auto routine you want.

