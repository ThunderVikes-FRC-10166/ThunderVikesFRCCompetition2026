# Learning `physics.py` (Simulation Physics + Field2d)

This guide teaches you how to write **`physics.py`**, the file that makes your robot *look like it’s moving* in simulation.

In real life:
- motors spin
- wheels push the carpet
- the robot actually drives around

In simulation:
- nothing physical moves
- so we need code that **pretends the robot moved** based on the commands we sent

That is what `physics.py` does.

---

# What this file is responsible for

✅ Reads the swerve drive command (`vx`, `vy`, `omega`)  
✅ Updates a “pretend robot position” called **pose**  
✅ Draws the robot on a field using **Field2d**  
✅ Updates the simulated gyro so field-oriented driving works  

---

# Important: This file depends on the other files you wrote

This file assumes you already have:

1) `swerve_module_sim.py`
2) `swerve_drive_sim.py`
3) `robot.py`

Specifically, it expects:
- your swerve drive object has a variable called `last_cmd`
- `last_cmd` is a `ChassisSpeeds(vx, vy, omega)`
- your swerve drive has a gyro object with `set_sim_yaw_rad(...)`

So physics can do:
- “Read how the robot *wanted* to move”
- “Pretend it moved that way”

---

# Step 1 — Imports

At the top of the file, students should type:

```python
import math
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from wpilib import Field2d, SmartDashboard
````

### What each import means (simple)

* `math`

  * used for angle wrap math and `pi`

* `Pose2d`

  * a pose is a robot’s position **and** direction:

    * X location (meters)
    * Y location (meters)
    * heading angle

* `Rotation2d`

  * represents an angle/heading

* `ChassisSpeeds`

  * stores:

    * `vx` (forward/back speed)
    * `vy` (left/right speed)
    * `omega` (rotation speed)

* `Field2d`

  * a dashboard widget that shows the robot moving on a field

* `SmartDashboard`

  * lets us publish the field widget so we can see it

---

# Step 2 — Helper function: `wrap_to_pi`

Angles can grow forever if you keep adding to them.

We want yaw always between:

* `-pi` and `+pi`

Students should type:

```python
def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad
```

This keeps angles nice and readable.

---

# Step 3 — Create the PhysicsEngine class

Python class definition:

```python
class PhysicsEngine:
```

In RobotPy simulation, a class named exactly `PhysicsEngine` inside `physics.py` is special.

pyfrc (RobotPy sim tools) will automatically use it when simulation runs.

---

## Step 3.1 — Understand the docstring

Students should include:

```python
class PhysicsEngine:
    """
    pyfrc will create this class automatically when running `robot.py sim`.
    It calls update_sim(now, tm_diff) repeatedly.
    """
```

Meaning:

* you do NOT manually create PhysicsEngine
* simulation creates it for you
* it calls `update_sim(...)` many times per second

---

# Step 4 — Write the constructor: `__init__`

Constructor method format:

```python
def __init__(self, physics_controller, robot):
```

Students should type:

```python
    def __init__(self, physics_controller, robot):
        self.robot = robot
```

### What is `robot`?

This is your real robot object from `robot.py`.

That means in physics we can access:

* `self.robot.swerve`
* `self.robot.driver`
* etc.

---

## Step 4.1 — Create a starting pose

Students should type:

```python
        self.pose = Pose2d(1.0, 1.0, Rotation2d(0.0))  # start somewhere not at (0,0)
        self.yaw_rad = 0.0
```

### What is a pose?

A pose is:

* X position
* Y position
* rotation

We start at `(1.0, 1.0)` so we aren’t exactly at the corner.

---

## Step 4.2 — Set up Field2d so we can see the robot

Students should type:

```python
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)
```

This publishes a widget named `"Field"`.

In Glass / SmartDashboard, you should see a field with your robot drawn on it.

---

# Step 5 — The main simulation loop: `update_sim`

Method header students should type:

```python
    def update_sim(self, now: float, tm_diff: float):
```

### What are these parameters?

* `now` = current time
* `tm_diff` = how much time passed since last update (seconds)

`tm_diff` is super important because:

* movement = speed × time

If time is 0.02 seconds and speed is 2 m/s, robot moves:

* 2 × 0.02 = 0.04 meters

---

## Step 5.1 — Read the commanded chassis speeds from swerve

Students should type:

```python
        swerve = getattr(self.robot, "swerve", None)
        cmd = getattr(swerve, "last_cmd", ChassisSpeeds(0.0, 0.0, 0.0))
```

### Why use `getattr`?

Because it is safer.

If something isn’t set up yet, it won’t crash.
It will just use zeros.

`cmd` is the movement the drive code requested:

* `cmd.vx`
* `cmd.vy`
* `cmd.omega`

---

## Step 5.2 — Update yaw (turning)

Students should type:

```python
        self.yaw_rad = wrap_to_pi(self.yaw_rad + cmd.omega * tm_diff)
        heading = Rotation2d(self.yaw_rad)
```

### What is happening?

* `cmd.omega` is how fast we rotate (rad/s)
* multiply by time to get how much we rotated this frame

So yaw changes each loop.

We wrap it so it stays between -pi and +pi.

---

## Step 5.3 — Convert robot-relative speeds into field-relative speeds

This is the part that can confuse people at first.

* The drive code uses robot-relative speeds:

  * “forward relative to robot”
  * “left relative to robot”

But the field position is field-relative:

* “up on the field”
* “left on the field”

So we rotate the velocity vector by the robot heading.

Students should type:

```python
        cosA = heading.cos()
        sinA = heading.sin()
        field_vx = cmd.vx * cosA - cmd.vy * sinA
        field_vy = cmd.vx * sinA + cmd.vy * cosA
```

This converts:

* robot forward/back and strafe
  into
* field X/Y motion

---

## Step 5.4 — Update position

Students should type:

```python
        new_x = self.pose.X() + field_vx * tm_diff
        new_y = self.pose.Y() + field_vy * tm_diff
        self.pose = Pose2d(new_x, new_y, heading)
```

This is basic physics:

* position = position + (velocity × time)

---

## Step 5.5 — Draw the robot on the field

Students should type:

```python
        self.field.setRobotPose(self.pose)
```

Now the Field2d widget updates.

---

## Step 5.6 — Update the simulated gyro (important for field-oriented)

Field-oriented driving relies on gyro yaw.

In simulation, we must “feed” yaw into the swerve gyro object.

Students should type:

```python
        if swerve is not None and hasattr(swerve, "gyro"):
            swerve.gyro.set_sim_yaw_rad(self.yaw_rad)
```

This keeps:

* `swerve.gyro.get_yaw()` correct
* so field-oriented math behaves properly

---

# Full reference file (final result)

After building step-by-step, your `physics.py` should look like:

```python
# physics.py
import math
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from wpilib import Field2d, SmartDashboard


def wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class PhysicsEngine:
    """
    pyfrc will create this class automatically when running `robot.py sim`.
    It calls update_sim(now, tm_diff) repeatedly.
    """

    def __init__(self, physics_controller, robot):
        self.robot = robot

        self.pose = Pose2d(1.0, 1.0, Rotation2d(0.0))  # start somewhere not at (0,0)
        self.yaw_rad = 0.0

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def update_sim(self, now: float, tm_diff: float):
        # Read commanded chassis speeds (robot-relative)
        swerve = getattr(self.robot, "swerve", None)
        cmd = getattr(swerve, "last_cmd", ChassisSpeeds(0.0, 0.0, 0.0))

        # Integrate yaw
        self.yaw_rad = wrap_to_pi(self.yaw_rad + cmd.omega * tm_diff)
        heading = Rotation2d(self.yaw_rad)

        # Convert robot-relative (vx, vy) to field-relative
        cosA = heading.cos()
        sinA = heading.sin()
        field_vx = cmd.vx * cosA - cmd.vy * sinA
        field_vy = cmd.vx * sinA + cmd.vy * cosA

        # Integrate position
        new_x = self.pose.X() + field_vx * tm_diff
        new_y = self.pose.Y() + field_vy * tm_diff
        self.pose = Pose2d(new_x, new_y, heading)

        # Publish robot pose to dashboard field widget
        self.field.setRobotPose(self.pose)

        # Feed the sim gyro (so field-oriented stays correct)
        if swerve is not None and hasattr(swerve, "gyro"):
            swerve.gyro.set_sim_yaw_rad(self.yaw_rad)
```

---

# How to run the simulator (after `physics.py` is set up)

## 1) Make sure you installed RobotPy sim tools

Your Python environment should have RobotPy 2026 installed.

If you already ran sim before, you’re good.

## 2) Run simulation from your project folder

In the terminal (inside your project), run:

```bash
robotpy sim
```

That will:

* start the simulation GUI
* run your robot code
* load `physics.py` automatically (if it exists)

## 3) Open Glass / Field2d view

When sim is running:

* open the simulation GUI window
* you should see the `"Field"` widget (Field2d)
* the robot should move when you drive

---

# Strong hints for the future (vision + autonomous)

This file is currently “simple physics”:

* it assumes the robot instantly moves exactly as commanded

Later, you can make simulation MUCH more realistic and powerful.

## 1) Adding AprilTag “vision”

To simulate AprilTags, you will eventually want to:

* know the “real field tag positions”
* compare your robot pose to tag poses
* generate fake camera measurements

Conceptually, it looks like:

* you already have robot pose: `self.pose`
* you will add a list/dictionary of AprilTag poses on the field
* if the robot is near a tag and facing it:

  * publish a fake “detected tag id”
  * publish a fake “tag relative pose”
  * publish fake “latency” (vision delay)

That would allow your robot code to test:

* auto-align
* pose estimation
* “drive to tag” behavior

## 2) Autonomous state machines (self positioning)

Once you can simulate pose and vision, you can build autonomous logic like:

* “drive to a shooting spot”
* “turn to face the target”
* “aim until error is small”
* “shoot”

A simple way to structure this later is a state machine:

* State 1: Drive to target pose
* State 2: Turn to aim
* State 3: Hold position & confirm alignment
* State 4: Shoot
* State 5: Done

Even in simulation, you can test if your state machine logic makes sense.

---

## For now: focus on the basics

Right now the win is:

* ✅ robot moves on Field2d
* ✅ gyro updates correctly
* ✅ field-oriented driving works
* ✅ your swerve code pipeline is proven

Once that is solid, vision + autonomous becomes much easier.

