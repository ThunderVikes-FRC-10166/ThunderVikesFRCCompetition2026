
# Real Swerve Drive (MagicBot Component) — Step-by-step (FRC 2026 / RobotPy 2026)

This guide teaches how to write the **real swerve drive component** that:
- receives driver commands (vx, vy, omega)
- uses WPILib kinematics to compute each wheel’s target
- sends each wheel a `(speed_mps, angle_rad)` command
- supports **field-oriented** driving using a real gyro

It is the “brain” that coordinates the 4 real swerve modules.

We assume you already wrote (or are writing) a real module class like:
- `SwerveModuleRev` (drive motor + turn motor + absolute encoder)

✅ Goal: Students should be able to write this themselves by understanding each method.

---

# 1) Compare: SIM drive vs REAL drive

## SIM `SwerveDriveSim`
- created its own modules (`SwerveModuleSim`)
- created its own gyro (`SimpleGyro`)
- stored a `last_cmd` so `physics.py` could move the robot

## REAL `SwerveDriveReal`
- does NOT create hardware inside the class
- instead, MagicBot injects:
  - `fl`, `fr`, `bl`, `br` modules
  - `gyro` sensor
- still uses the same kinematics math
- still calls `module.set(speed_mps, angle_rad)`
- still can store `last_cmd` for logging/telemetry (even if physics isn’t used on robot)

**Big idea:**
> The math is basically the same. The “plumbing” is different because real hardware is injected.

---

# 2) What this component needs (inputs and outputs)

## Inputs
1) `vx_mps` (forward/back command)
2) `vy_mps` (strafe command)
3) `omega_radps` (rotate command)
4) gyro yaw (for field oriented)

## Outputs
Commands to each module:
- wheel speed (m/s)
- wheel angle (rad)

---

# 3) Create a new file

Create:

- `components/swerve_drive_real.py`

---

# 4) Step-by-step: writing `SwerveDriveReal`

## Step 4.1 — Imports

At the top of the file, type:

```python
# components/swerve_drive_real.py
import math

from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
````

### What we imported and why

* `math` gives us `pi`
* `Translation2d` defines wheel positions on robot frame
* `ChassisSpeeds` stores (vx, vy, omega)
* `SwerveDrive4Kinematics` converts chassis speeds → wheel states

---

## Step 4.2 — Write the class header

In Python, classes look like:

```python
class SwerveDriveReal:
```

So type:

```python
class SwerveDriveReal:
```

---

## Step 4.3 — Add MagicBot injection slots

We want MagicBot to inject:

* 4 modules
* gyro

At the top of the class, type:

```python
class SwerveDriveReal:
    # Injected modules (from robot.py createObjects)
    fl = None
    fr = None
    bl = None
    br = None

    # Injected gyro (navX / Pigeon / etc)
    gyro = None
```

**Important:**

* In the Robot class, we must create objects named exactly:

  * `self.fl`, `self.fr`, `self.bl`, `self.br`, `self.gyro`
* Then MagicBot will assign them to these slots

---

## Step 4.4 — Use `setup()` (not `__init__`)

Real MagicBot components should not use `__init__` for hardware work.

Type:

```python
    def setup(self):
        ...
```

Inside setup, we define:

* max speeds
* robot geometry
* the kinematics object
* field oriented flag
* storage variables for drive commands

---

## Step 4.5 — Add max speeds (constants)

Inside `setup()`, type:

```python
        self.max_speed_mps = 4.0
        self.max_omega_radps = 2.0 * math.pi
```

These values are robot-dependent and can be tuned later.

---

## Step 4.6 — Define robot wheel positions (geometry)

Inside `setup()`, type:

```python
        wheelbase_m = 0.60
        trackwidth_m = 0.60
        hw = wheelbase_m / 2.0
        ht = trackwidth_m / 2.0
```

Meaning:

* wheelbase: distance front wheels to back wheels
* trackwidth: distance left wheels to right wheels
* `hw`/`ht` are half distances from robot center

---

## Step 4.7 — Create kinematics object

Inside `setup()`, type:

```python
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(+hw, +ht),  # FL
            Translation2d(+hw, -ht),  # FR
            Translation2d(-hw, +ht),  # BL
            Translation2d(-hw, -ht),  # BR
        )
```

This tells WPILib where each wheel is on the robot.

---

## Step 4.8 — Field-oriented flag + command storage

Inside `setup()`, type:

```python
        self.field_oriented = True

        self._desired_cmd = ChassisSpeeds(0.0, 0.0, 0.0)
        self.last_cmd = ChassisSpeeds(0.0, 0.0, 0.0)
```

* `_desired_cmd` is internal
* `last_cmd` is useful for:

  * logging
  * debugging
  * possibly simulation tools

---

# 5) The `drive()` method: store movement request

The signature should be typed like:

```python
    def drive(self, vx_mps: float, vy_mps: float, omega_radps: float):
        ...
```

This method is called by `robot.py` every loop.

---

## Step 5.1 — Clamp values

Inside `drive()`, type:

```python
        vx_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vx_mps))
        vy_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vy_mps))
        omega_radps = max(-self.max_omega_radps, min(self.max_omega_radps, omega_radps))
```

This prevents unrealistic commands.

---

## Step 5.2 — Field oriented conversion

If field oriented, we need yaw from the gyro.

We want yaw as a `Rotation2d`.

Different gyros return yaw differently, so our code should rely on a **gyro interface**:

✅ The gyro object should provide a method like:

```python
def get_yaw(self):
    ...
```

and return a Rotation2d.

For now, inside `drive()` we do:

```python
        if self.field_oriented:
            yaw = self.gyro.get_yaw()
            self._desired_cmd = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx_mps, vy_mps, omega_radps, yaw
            )
        else:
            self._desired_cmd = ChassisSpeeds(vx_mps, vy_mps, omega_radps)
```

---

## Step 5.3 — Save for debug

At the end of drive():

```python
        self.last_cmd = self._desired_cmd
```

---

# 6) The `execute()` method: compute wheel commands and send them

In MagicBot, `execute()` runs repeatedly.

Type the header:

```python
    def execute(self):
        ...
```

---

## Step 6.1 — Convert chassis speeds → module states

Inside execute(), type:

```python
        states = self.kinematics.toSwerveModuleStates(self._desired_cmd)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, self.max_speed_mps)
```

Explanation:

* `toSwerveModuleStates` gives 4 states:

  * speed (m/s)
  * angle (Rotation2d)
* `desaturateWheelSpeeds` scales speeds if any exceed max

---

## Step 6.2 — Send setpoints to each module

Each state is sent to the module:

```python
        self.fl.set(states[0].speed, states[0].angle.radians())
        self.fr.set(states[1].speed, states[1].angle.radians())
        self.bl.set(states[2].speed, states[2].angle.radians())
        self.br.set(states[3].speed, states[3].angle.radians())
```

**Important:**
This depends on your real module having:

```python
def set(self, speed_mps: float, angle_rad: float):
    ...
```

Just like simulation.

---

# 7) Full reference skeleton (structure)

After students implement step-by-step, the class should include:

* `setup()`
* `drive()`
* `execute()`

Here is the structure reference:

```python
# components/swerve_drive_real.py
import math

from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics


class SwerveDriveReal:
    fl = None
    fr = None
    bl = None
    br = None
    gyro = None

    def setup(self):
        self.max_speed_mps = 4.0
        self.max_omega_radps = 2.0 * math.pi

        wheelbase_m = 0.60
        trackwidth_m = 0.60
        hw = wheelbase_m / 2.0
        ht = trackwidth_m / 2.0

        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(+hw, +ht),  # FL
            Translation2d(+hw, -ht),  # FR
            Translation2d(-hw, +ht),  # BL
            Translation2d(-hw, -ht),  # BR
        )

        self.field_oriented = True
        self._desired_cmd = ChassisSpeeds(0.0, 0.0, 0.0)
        self.last_cmd = ChassisSpeeds(0.0, 0.0, 0.0)

    def drive(self, vx_mps: float, vy_mps: float, omega_radps: float):
        vx_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vx_mps))
        vy_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vy_mps))
        omega_radps = max(-self.max_omega_radps, min(self.max_omega_radps, omega_radps))

        if self.field_oriented:
            yaw = self.gyro.get_yaw()
            self._desired_cmd = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx_mps, vy_mps, omega_radps, yaw
            )
        else:
            self._desired_cmd = ChassisSpeeds(vx_mps, vy_mps, omega_radps)

        self.last_cmd = self._desired_cmd

    def execute(self):
        states = self.kinematics.toSwerveModuleStates(self._desired_cmd)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, self.max_speed_mps)

        self.fl.set(states[0].speed, states[0].angle.radians())
        self.fr.set(states[1].speed, states[1].angle.radians())
        self.bl.set(states[2].speed, states[2].angle.radians())
        self.br.set(states[3].speed, states[3].angle.radians())
```

---

# 8) What students must understand (fundamentals)

Students should be able to answer:

1. What is `ChassisSpeeds` and what does it store?
2. What does kinematics do?
3. Why do we need wheel positions (`Translation2d`)?
4. What does field oriented mean?
5. Why do we call `.set(speed, angle)` on each module?

---

# 9) Strong hints for integrating with `robot.py` later

In `robot.py` (real robot), the driver code stays almost the same:

* read Xbox controller
* compute vx/vy/omega in real units
* call:

```python
self.swerve.drive(vx, vy, omega)
```

The only changes are:

* the component import changes from sim → real
* robot hardware objects must be created in `createObjects()`

---

# 10) Strong hints for real gyro handling

Different gyros use different method names.

To keep code clean, create a tiny wrapper class like:

```python
class GyroWrapper:
    def __init__(self, navx):
        self.navx = navx

    def get_yaw(self):
        # convert degrees to Rotation2d
        ...
```

Then the rest of the swerve drive code only uses `gyro.get_yaw()`.

This is the same pattern as your `SimpleGyro` in simulation.

---

# 11) Next improvements once real robot drives

After basic driving works, teams typically add:

* odometry (track robot pose using wheel encoders + gyro)
* pose estimation (add vision AprilTags)
* autonomous path planning
* auto-align to target while driving
* shooter alignment and distance-based RPM

But for now:
✅ just get drive working safely, predictably, and consistently

