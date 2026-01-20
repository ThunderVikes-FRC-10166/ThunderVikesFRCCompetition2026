# Learning `swerve_drive_sim.py` (Simulated Swerve Drive)

This guide teaches you how to write the **swerve drive brain** for simulation, step by step.

You already wrote `swerve_module_sim.py`, which created a *single* simulated swerve module.

Now we will write `swerve_drive_sim.py`, which creates the **SwerveDriveSim** class that controls:

- 4 swerve modules (FL, FR, BL, BR)
- a simulated gyro
- the WPILib kinematics math that turns joystick movement into wheel speed + wheel angle

> ✅ Goal: Students should be able to write this file themselves without copy/paste.

---

## IMPORTANT NOTE about `__init__` vs `setup()` (read this!)

In this simulation lesson, we are using `__init__()` because:
- it is simple
- it works great for simulation-only objects
- there is no real hardware or MagicBot injection involved yet

✅ In simulation files, `__init__()` is OK.

However, **when we build the real robot code later**, we will **avoid doing hardware setup inside `__init__()`**.

Instead, real robot components should use:

```python
def setup(self):
    ...
````

Why?

* MagicBot injects motors/sensors into components
* those objects may be `None` during `__init__()`
* calling SparkMax / navX in `__init__()` can cause bugs

✅ Real robot components = use `setup()`
✅ Simulation-only classes = `__init__()` is fine

We will teach the “real robot MagicBot style” later when hardware is ready.

---

# What you are building (high level)

A swerve robot moves using 3 motion values:

* `vx` = forward/back speed (meters per second)
* `vy` = left/right speed (meters per second)
* `omega` = spin speed (radians per second)

Your job in this file is to convert those 3 values into what the 4 wheels should do:

* Wheel speed (m/s)
* Wheel angle (radians)

---

# Step 1 — Start the file and write your imports

When starting a Python file, you type imports at the top:

```python
# swerve_drive_sim.py
import math

from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState

from .swerve_module_sim import SwerveModuleSim
```

### What these imports mean (simple):

* `math` lets us use:

  * `math.pi` (π)
* `Rotation2d` represents an angle/heading (like gyro yaw)
* `Translation2d` represents positions on the robot (where wheels are)
* `ChassisSpeeds` stores vx/vy/omega
* `SwerveDrive4Kinematics` does the swerve math
* `SwerveModuleSim` is your simulated module class from the previous file

---

# Step 2 — Create a simple gyro class (simulation only)

## 2.1 Write the class

In Python, you define a class like this:

```python
class SimpleGyro:
```

Then inside it, you write methods.

---

## 2.2 Write the constructor `__init__`

A constructor is a special method that runs when you create the object.

Type this:

```python
class SimpleGyro:
    """Sim gyro that physics.py will update."""

    def __init__(self):
        self._yaw_rad = 0.0
```

### What is `self._yaw_rad`?

This stores the robot’s current direction (yaw) in radians.

---

## 2.3 Write a method that returns yaw

A method looks like:

```python
def method_name(self):
    ...
```

So we type:

```python
    def get_yaw(self) -> Rotation2d:
        return Rotation2d(self._yaw_rad)
```

✅ This returns yaw as a WPILib `Rotation2d`.

---

## 2.4 Write a method to update yaw in simulation

In simulation, the physics system will call this.

```python
    def set_sim_yaw_rad(self, yaw_rad: float):
        self._yaw_rad = yaw_rad
```

✅ Now physics can “inject” the robot’s yaw angle into this gyro.

---

# Step 3 — Start the main `SwerveDrive` class

You define the main class like this:

```python
class SwerveDriveSim:
```

This class will store:

* speed limits
* robot geometry
* kinematics
* gyro
* 4 modules

---

# Step 4 — Write the `__init__` method for `SwerveDrive`

Inside the class, start with:

```python
class SwerveDriveSim:
    def __init__(self):
        ...
```

Full first part:

```python
class SwerveDriveSim:
    """
    Minimal swerve drive component for simulation:
    - drive(vx, vy, omega) stores desired chassis speeds
    - execute() computes 4 module targets (speed+angle) and updates the modules
    - last_cmd is used by physics.py to move the robot pose on the field
    """

    def __init__(self):
        # Top speeds (tune later)
        self.max_speed_mps = 4.0
        self.max_omega_radps = 2.0 * math.pi
```

### What are these?

* `max_speed_mps` is the fastest the robot is allowed to translate
* `max_omega_radps` is the fastest the robot is allowed to rotate

---

## Step 4.1 — Define robot geometry constants

Type:

```python
        # Robot geometry (meters)
        wheelbase_m = 0.60
        trackwidth_m = 0.60
        hw = wheelbase_m / 2.0
        ht = trackwidth_m / 2.0
```

### Meaning:

* `wheelbase_m` = front to back distance between wheels
* `trackwidth_m` = left to right distance between wheels
* `hw` and `ht` are “half distance” values so we can define wheel locations from robot center

---

## Step 4.2 — Create kinematics object

Now type:

```python
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(+hw, +ht),  # FL
            Translation2d(+hw, -ht),  # FR
            Translation2d(-hw, +ht),  # BL
            Translation2d(-hw, -ht),  # BR
        )
```

This tells WPILib:

* Front Left wheel is forward and left
* Front Right is forward and right
* Back Left is back and left
* Back Right is back and right

✅ This is required so WPILib can compute wheel speeds/angles.

---

## Step 4.3 — Add gyro + field oriented setting

Type:

```python
        self.gyro = SimpleGyro()
        self.field_oriented = True
```

### What does `field_oriented = True` mean?

It means:

> Pushing joystick forward always drives away from the driver,
> even if robot is turned sideways.

Field oriented requires gyro yaw.

---

## Step 4.4 — Create the 4 simulated modules

Type:

```python
        # Sim modules
        self.fl = SwerveModuleSim("FL")
        self.fr = SwerveModuleSim("FR")
        self.bl = SwerveModuleSim("BL")
        self.br = SwerveModuleSim("BR")
```

This creates:

* Front Left module
* Front Right module
* Back Left module
* Back Right module

Each module stores speed + angle and publishes to dashboard.

---

## Step 4.5 — Store the chassis speeds the robot should use

Type:

```python
        self._desired_cmd = ChassisSpeeds(0.0, 0.0, 0.0)
```

This means:

* vx=0
* vy=0
* omega=0

So robot is initially not moving.

---

## Step 4.6 — Add `last_cmd` for physics simulation

Type:

```python
        # physics.py reads this to move the robot
        self.last_cmd = ChassisSpeeds(0.0, 0.0, 0.0)
```

✅ This is how physics.py knows what the robot is trying to do.

---

# Step 5 — Write the `drive()` method

A method always starts with:

```python
def name(self, ...):
```

So type:

```python
    def drive(self, vx_mps: float, vy_mps: float, omega_radps: float):
```

Now we write the body.

---

## Step 5.1 — Clamp (limit) the speeds for safety

Type:

```python
        vx_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vx_mps))
        vy_mps = max(-self.max_speed_mps, min(self.max_speed_mps, vy_mps))
        omega_radps = max(-self.max_omega_radps, min(self.max_omega_radps, omega_radps))
```

### Why clamp?

So someone can’t accidentally request:

* 900 m/s or
* 100 rotations per second

Clamping keeps everything inside safe bounds.

---

## Step 5.2 — Field oriented conversion (optional)

Type:

```python
        if self.field_oriented:
            yaw = self.gyro.get_yaw()
            self._desired_cmd = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx_mps, vy_mps, omega_radps, yaw
            )
        else:
            self._desired_cmd = ChassisSpeeds(vx_mps, vy_mps, omega_radps)
```

### What does this do?

* Field oriented: convert joystick field speeds to robot-relative speeds
* Robot oriented: use the speeds as-is

WPILib needs robot-relative speeds for kinematics.

---

## Step 5.3 — Save the command for physics

Type:

```python
        self.last_cmd = self._desired_cmd
```

✅ Now physics can move the robot correctly.

---

# Step 6 — Write the `execute()` method

In MagicBot style, `execute()` runs repeatedly (fast loop).

Type:

```python
    def execute(self):
```

Now we compute module targets and send them.

---

## Step 6.1 — Convert chassis speeds into 4 module states

Type:

```python
        states = self.kinematics.toSwerveModuleStates(self._desired_cmd)
```

This produces a list of 4 wheel states:

* wheel speed
* wheel angle

---

## Step 6.2 — Desaturate wheel speeds

Type:

```python
        SwerveDrive4Kinematics.desaturateWheelSpeeds(states, self.max_speed_mps)
```

### Why?

Sometimes math asks for a wheel speed faster than max.
So WPILib scales everything down safely.

---

## Step 6.3 — Send speed + angle to each module

Type:

```python
        self.fl.set(states[0].speed, states[0].angle.radians())
        self.fr.set(states[1].speed, states[1].angle.radians())
        self.bl.set(states[2].speed, states[2].angle.radians())
        self.br.set(states[3].speed, states[3].angle.radians())
```

### What are we doing here?

We are calling the module method you wrote earlier:

```python
def set(self, speed_mps, angle_rad):
    ...
```

Each wheel gets:

* a speed in meters/second
* an angle in radians

---

## Step 6.4 — Call module execute for dashboard updates

Type:

```python
        self.fl.execute()
        self.fr.execute()
        self.bl.execute()
        self.br.execute()
```

Now dashboard values update each loop.

---

# Summary: What students should understand

* ✅ `drive()` saves what the driver *wants the robot to do*
* ✅ `execute()` converts that into **4 wheel commands**
* ✅ `SwerveDrive4Kinematics` is the WPILib math tool that makes swerve possible
* ✅ the gyro makes **field oriented driving** possible
* ✅ `last_cmd` is saved so the simulator can move the robot


