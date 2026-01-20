# A simple Guide to some of the code needed for the hardware
## 1) WPILib + RobotPy basics you’ll use constantly

### XboxController (driver input)

Create it:

```python
driver = wpilib.XboxController(0)
```

Most-used methods (read sticks):

* `driver.getLeftY()`
* `driver.getLeftX()`
* `driver.getRightX()`

RobotPy docs: `wpilib.XboxController` ([RobotPy][1])

---

### SmartDashboard (quick debugging)

Push numbers/booleans:

```python
wpilib.SmartDashboard.putNumber("vx", vx)
wpilib.SmartDashboard.putBoolean("hasTarget", has_target)
```

(You already used this pattern in sim.)

---

### Timer (simple autonomous / delays)

```python
t = wpilib.Timer()
t.restart()
seconds = t.get()
```

Good for “drive forward for 2 seconds” style autos.

---

### Field2d (visual field widget in Glass)

Create + publish:

```python
field = wpilib.Field2d()
wpilib.SmartDashboard.putData("Field", field)
```

Update pose:

```python
field.setRobotPose(pose2d)
```

Docs: Field2d widget overview ([FIRST Robotics Competition Documentation][2])

---

### WPIMath geometry/kinematics (swerve math types)

Most-used types for swerve:

* `ChassisSpeeds(vx, vy, omega)`
* `SwerveDrive4Kinematics(...)`
* `Pose2d(x, y, Rotation2d(...))`
* `Rotation2d.fromDegrees(deg)` / `.radians()` / `.degrees()`

(These let you stay in **meters** and **radians**, which is the WPILib standard.)

---

## 2) REV (Spark MAX + NEO) — the important methods

RobotPy REV 2026 wrapper docs ([RobotPy][3])
REV’s own closed-loop overview (concepts apply even if code examples are Java/C++) ([REV Robotics Documentation][4])

### Create a Spark MAX motor controller

Most teams do (conceptually):

```python
motor = rev.SparkMax(device_id, rev.SparkMax.MotorType.kBrushless)
```

Key idea: **NEO is brushless**, so you use `kBrushless`.

Docs show SparkMax exists and is how you create the controller ([RobotPy][5])

---

### `motor.getEncoder()` (read motor position/velocity)

Used to get the internal NEO encoder:

```python
enc = motor.getEncoder()
```

Typical things you’ll read:

* `enc.getPosition()` → rotations (unless you set conversion factor)
* `enc.getVelocity()` → RPM (unless you set conversion factor)

Why students care:

* drive velocity feedback (for m/s conversion)
* steering motor position feedback (if you use integrated encoder)

---

### Closed-loop controller (PID inside the Spark)

REV’s closed-loop control is done through a controller object (concept is consistent across languages) ([REV Robotics Documentation][4]).

In RobotPy REV, you’ll typically:

```python
ctrl = motor.getClosedLoopController()
```

Then command setpoints with:

```python
ctrl.setReference(value, rev.SparkClosedLoopController.ControlType.kVelocity)
ctrl.setReference(value, rev.SparkClosedLoopController.ControlType.kPosition)
```

**What those control types mean:**

* `kVelocity` → “try to spin at this speed”
* `kPosition` → “try to move to this position”

This matches how we described drive vs steer:

* **drive motor** → velocity control
* **turn motor** → position control

---

### “Real robot reality” note for students

On the real robot, you must also **configure**:

* PID gains (P/I/D) for each loop
* current limits
* idle mode (brake/coast)
* inversion
* conversion factors (so encoder units are meaningful)

Those settings are what make the robot behave “predictably” instead of “random”.

(Students don’t need to memorize all config calls immediately—just the idea that *hardware needs configuration*.)

---

## 3) Absolute encoders (common with REV Through Bore)

Many teams read the REV through-bore absolute encoder using WPILib `DutyCycleEncoder`:

Create:

```python
abs_enc = wpilib.DutyCycleEncoder(dio_channel)
```

Most-used method:

* `abs_enc.getAbsolutePosition()` → returns **0.0 to 1.0** (one full rotation)

Then convert to radians:

```python
angle_rad = abs_enc.getAbsolutePosition() * 2.0 * math.pi
```

And apply an offset.

Why this matters:

* When robot boots, you immediately know the steering angle (no guessing).

---

## 4) navX (gyro) — the important methods

RobotPy navX docs ([RobotPy][6])

### Create navX (common pattern)

RobotPy navX includes factory helpers like `create_spi()` (and other connection methods). ([RobotPy][7])

Conceptually:

```python
from navx import AHRS
navx = AHRS.create_spi()
```

### Most-used methods

* `navx.getYaw()` → yaw angle (usually degrees)
* `navx.getRotation2d()` → WPILib Rotation2d directly (super handy for swerve field-oriented) ([RobotPy][7])
* `navx.zeroYaw()` → reset heading at the start of a match (common)

**Why gyro matters in swerve:**

* Field-oriented driving: joystick “up” means “field forward” even if robot is rotated.
* Autonomous turning: you need a reliable heading signal.

---

## 5) Limelight (AprilTags) — what you read and how

Limelight AprilTag tracking overview ([Limelight Documentation][8])
Limelight complete NetworkTables API ([Limelight Documentation][9])

### The simplest “auto-aim” signals

Limelight publishes to NetworkTables. The big 3 values students use first:

* `tv` → target valid (0 or 1)
* `tx` → horizontal offset angle (degrees)
* `ty` → vertical offset angle (degrees)
* `ta` → target area (often used as a rough distance clue)

Limelight confirms AprilTags still use `tx/ty/ta`. ([Limelight Documentation][8])

### How to read NetworkTables in Python (concept)

You’ll access the Limelight table (often named `"limelight"` or `"limelight-front"` depending on your config). Limelight docs explain the table naming and NT4 behavior. ([Limelight Documentation][9])

Students will typically do:

* get the table
* read `tx`, `tv`, etc.

### AprilTag extras (later)

For pose estimation, Limelight can publish things like:

* `botpose` (estimated robot pose)
* tag IDs
* more advanced JSON results ([Limelight Documentation][8])

That’s “Phase 2” after basic aiming.

---

## 6) What students should memorize vs “look up”

### Memorize (daily use)

* `XboxController.getLeftY/getLeftX/getRightX` ([RobotPy][1])
* `SmartDashboard.putNumber/putBoolean`
* `ChassisSpeeds(vx, vy, omega)`
* `Rotation2d` (degrees ↔ radians)
* `navx.getRotation2d()` (or `getYaw()` + conversion) ([RobotPy][7])
* Limelight: `tv`, `tx`, `ty` ([Limelight Documentation][8])

### Look up (when implementing)

* REV configuration calls + PID slot settings (because they’re detailed and version-specific) ([RobotPy][3])

---

## 7) Where to find “official” docs quickly (students can bookmark)

* RobotPy WPILib API (Python classes like XboxController) ([RobotPy][1])
* RobotPy REV 2026 wrapper docs (Python API for Spark MAX) ([RobotPy][3])
* REV closed-loop concepts and setup (vendor docs) ([REV Robotics Documentation][4])
* RobotPy navX docs (Python API methods like `create_spi`, `getYaw`, `getRotation2d`) ([RobotPy][6])
* Limelight docs (NetworkTables keys + AprilTag usage) ([Limelight Documentation][9])

---

## 8) A “what you’ll do with each device” summary

### REV motors

* Drive motor: **velocity control** (`kVelocity`) to hit target m/s
* Turn motor: **position control** (`kPosition`) to hit target angle
* Encoders: measure how fast / where you are (feedback)

### navX gyro

* Provide robot heading for:

  * field-oriented drive
  * accurate turning in auto

### Limelight

* Provide “where is the tag relative to camera?”
* Easy aim mode:

  * if `tv==1`, rotate until `tx≈0`
* Advanced:

  * use pose (`botpose`) to self-position

---


[1]: https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib.html?utm_source=chatgpt.com "wpilib Package - RobotPy Documentation - Read the Docs"
[2]: https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html?utm_source=chatgpt.com "The Field2d Widget - WPILib Docs"
[3]: https://robotpy.readthedocs.io/projects/rev/en/latest/?utm_source=chatgpt.com "RobotPy REV 2026.0.0 documentation"
[4]: https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started?utm_source=chatgpt.com "Closed Loop Control Getting Started | REVLib"
[5]: https://robotpy.readthedocs.io/projects/rev/en/stable/rev.html?utm_source=chatgpt.com "rev Package — RobotPy REV 2025.0.3.1 documentation"
[6]: https://robotpy.readthedocs.io/projects/navx/en/stable/?utm_source=chatgpt.com "robotpy-navx library"
[7]: https://robotpy.readthedocs.io/projects/navx/en/stable/genindex.html?utm_source=chatgpt.com "Index — robotpy-navx 2025.0.1.2 documentation"
[8]: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags?utm_source=chatgpt.com "Tracking AprilTags"
[9]: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api?utm_source=chatgpt.com "NetworkTables API - Limelight Documentation"
