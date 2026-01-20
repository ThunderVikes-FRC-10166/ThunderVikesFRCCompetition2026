# Autonomous + Auto-Aim (AprilTags) in MagicBot — Beginner Guide (FRC 2026 / RobotPy 2026)

This guide teaches the **big picture** of how autonomous works in **MagicBot** and how students can build:

1) **Autonomous mode** (robot drives by itself for a planned routine)  
2) **Auto-aim / self-position mode** in Teleop (robot helps driver aim using AprilTags)

It’s written for beginners, so we go slowly and focus on understanding.

---

# 0) Key idea: Autonomous is just “code that drives without a human”

In Teleop:
- driver moves sticks
- code converts sticks → `swerve.drive(vx, vy, omega)`

In Autonomous:
- no sticks
- code generates its own vx/vy/omega commands
- still calls the SAME method:
  ```python
    swerve.drive(vx, vy, omega)
  ```

✅ Autonomous and Teleop both use the same drivetrain API.
The difference is: “who decides vx/vy/omega?”

---

# 1) The simplest autonomous you can build (Timer Auto)

Before doing anything fancy (paths, AprilTags), start with a simple timer auto:

## Example goal:

* drive forward for 2 seconds
* stop

This helps students learn the pattern:

* start auto
* do action
* stop

### The mental model:

* Autonomous is like a recipe:

  1. do step A
  2. then do step B
  3. then stop

---

# 2) MagicBot autonomous: how it’s usually structured

MagicBot commonly uses an “autonomous mode switcher” system.

**Typical team structure:**

* create an `autonomous/` folder
* each autonomous routine is a Python class in that folder
* MagicBot loads them at startup
* you can choose which mode to run

If your sim output said:

> “Cannot load the 'autonomous' package”

that means the folder didn’t exist yet, or it was empty.

✅ So the first step is: create the folder.

---

# 3) Create the `autonomous` folder and files

In your robot project, create:

```
RobotMain/
  robot.py
  autonomous/
    __init__.py
    drive_forward_auto.py
```

## 3.1 The `__init__.py` file

This file can be empty, but it must exist so Python treats the folder as a package.

Create:

`autonomous/__init__.py`

and leave it empty.

---

# 4) A beginner Autonomous routine (Drive Forward)

Create:

`autonomous/drive_forward_auto.py`

Now, here’s the structure students should understand and build.

## 4.1 Define the class

A class in Python starts like:

```python
class DriveForwardAuto:
```

In MagicBot-style autonomous, you usually need:

* access to the `swerve` component
* a way to track time or steps

So conceptually:

* you’ll have `swerve` injected (like components)
* you’ll run a routine for a few seconds

---

## 4.2 What the autonomous class *needs*

It needs to:

* start when auto begins
* update each loop
* stop at the end

Think of it like a mini-robot program with 3 phases:

1. init
2. periodic
3. done

---

# 5) “State machines” — the easiest way to build real autonomous

A **state machine** is just:

* a variable that tracks “what step are we on?”

Example states:

* `"DRIVE_FORWARD"`
* `"STOP"`

Then each loop:

* check what state you’re in
* do the action for that state
* switch to the next state when finished

✅ This is beginner-friendly
✅ This is how many teams build reliable autos
✅ This is also how auto-aim modes are often done

---

# 6) Simple Autonomous State Machine Example (Step-by-step)

## Goal:

* State 1: drive forward for 2 seconds
* State 2: stop forever

### What students will write

They will create:

* a `state` string
* a `timer`

---

## Step A — Create the timer

WPILib provides a timer:

```python
import wpilib
```

You can use:

```python
timer = wpilib.Timer()
```

---

## Step B — Autonomous class skeleton

Students should understand this structure:

1. `on_enable()` (or “autonomousInit” idea)
2. `execute()` called repeatedly during auto

---

# 7) Auto-Aim / Self-Position mode in Teleop (AprilTags)

Now the cool part.

## What is auto-aim?

Auto-aim means:

* driver can still drive
* but when they press a button (like “A” or “Right Bumper”)
* the robot *helps* rotate (or move) to align to the goal

Example:

* driver moves near the scoring area
* presses auto-aim
* robot rotates to face AprilTag ID 7
* driver shoots

---

# 8) How AprilTag aiming works (conceptually)

AprilTags give us information like:

* where the tag is
* where the robot is (estimated)
* or at least:

  * “tag is X degrees to the left/right”

Two common approaches:

## Approach 1 (simpler): rotate until Limelight `tx` is near 0

Limelight often provides:

* `tx`: horizontal angle error to target
  So:
* if `tx` is +10°, target is to the right
* rotate right until `tx` becomes ~0

This is *very beginner-friendly*.

## Approach 2 (advanced): full pose estimation + drive to pose

If you know your robot pose and tag pose:

* compute where you want robot to stand
* drive to that pose (x, y) + face tag
  This is more accurate but harder.

---

# 9) The simplest auto-aim: “turn to target” with Limelight

### Idea:

* When button is held:

  * read `tx`
  * compute a rotation speed from it
  * override driver rotation stick
* Translation (x/y) can still come from driver (optional)

### P-control (very simple controller)

A simple control rule is:

```python
omega = -kP * tx
```

Where:

* `kP` is a small number (like 0.02 or 0.03)
* `tx` is in degrees
* omega should be in radians/sec, so later you convert

Beginner rule:

* bigger tx → rotate faster
* smaller tx → rotate slower
* when tx is near 0 → stop rotating

✅ This is the easiest “auto-align” to build.

---

# 10) Self-position mode (drive to a shooting spot)

Once you have an estimated robot pose, you can do:

* “go to this pose near the tag”
* “face the tag”
* stop when close enough

That requires:

## A) Robot pose estimate (odometry)

You need to track robot position on field.
Usually done with:

* gyro yaw
* wheel encoders
* odometry classes (WPILib)

## B) Tag pose (field layout)

WPILib provides AprilTag field layouts for each season (your mentors will load it).
Then you can know:

* where tag ID 7 is on the field

## C) A controller to drive to pose

Usually:

* separate PID for X, Y, and rotation
* or a holonomic controller

This is “Phase 2” once basic driving is stable.

---

# 11) How to connect these ideas to MagicBot

In MagicBot, you generally separate code into:

✅ Components (drivetrain, shooter, intake)
✅ Autonomous modes (a routine that commands components)
✅ Teleop logic (driver + optional “assist modes”)

So for aiming you might have:

* `swerve` component always available
* a new “assist component” or a small chunk of logic in robot.py:

  * if button pressed: override omega

For autonomous you will have:

* an autonomous class that calls `swerve.drive(...)` repeatedly using a state machine

---

# 12) A recommended learning path for students

## Level 1: Simple time-based auto

* drive forward 2 seconds
* stop

## Level 2: Add rotation auto

* rotate in place for 1 second
* stop

## Level 3: Two-step auto (state machine)

* drive forward 2 seconds
* rotate 90 degrees
* stop

## Level 4: Auto-aim in teleop using Limelight tx

* hold button → robot rotates to face target

## Level 5: Self-position using pose + AprilTags

* drive to pose near tag
* rotate to face it
* stop and shoot

---

# 13) Student “recipe” for an autonomous routine

When students build a new auto, tell them:

### Step 1

Write down the routine in English:

* Drive forward 2m
* Turn toward tag
* Drive to shooting location
* Aim
* Shoot

### Step 2

Convert it into states:

* STATE_DRIVE_FORWARD
* STATE_TURN
* STATE_DRIVE_TO_SHOT
* STATE_AIM
* STATE_SHOOT
* STATE_DONE

### Step 3

For each state decide “how it ends”

* time ended?
* distance reached?
* angle error small?
* shooter at speed?

### Step 4

Implement and test in sim

---

# 14) What to remember about units (very important)

In your drivetrain code:

* `vx` and `vy` are in meters/second
* `omega` is in radians/second

But Limelight gives angles in degrees (tx), so you must convert or just treat it as “error”.

A safe approach:

* compute omega as a fraction of max rotation speed:

```python
omega = (tx / 30.0) * max_omega
```

Because Limelight tx might be around -30..+30 degrees.

---

# 15) How this will look in code later (conceptual, not copy/paste)

## Teleop with auto-aim button concept:

* read sticks
* compute vx/vy
* if button pressed:

  * replace `omega` with auto-aim omega
* call `swerve.drive(vx, vy, omega)`

## Autonomous concept:

* state machine decides vx/vy/omega
* call `swerve.drive(vx, vy, omega)` each loop
* switch states as goals are met

---

# 16) Final mindset

Autonomous is not magic.
Auto-aim is not magic.

It’s always the same pipeline:

1. figure out what motion you want (vx/vy/omega)
2. call `swerve.drive(vx, vy, omega)`
3. let swerve math + modules handle the wheels

If students understand that, they can build any routine.


