# Real Swerve Module (REV) — Student Guide (Updated Version)

This guide teaches how to build a **real** swerve module class that is:
- **Beginner friendly** (simple structure)
- **Actually usable on a real robot**
- Compatible with **RobotPy 2026** + **MagicBot injection**
- Uses **REV Spark MAX** + **NEO/NEO Vortex** + **NEO 550**
- Uses a **REV Through Bore absolute encoder** (DutyCycleEncoder)
- Provides everything the drivetrain needs for **odometry / pose estimation** later:
  - `get_position() -> SwerveModulePosition`
  - `get_state() -> SwerveModuleState`

✅ The goal is to understand each part, not copy/paste a final file.

---

# 0) What “keeping track of robot position” means (why this module matters)

Later, your drivetrain will run a pose estimator (odometry) that updates robot position every loop:

- It asks each module: **“How far did your wheel roll since the last update?”**
- It asks the gyro: **“What direction is the robot facing?”**
- It combines those to update the robot’s estimated pose (x, y, heading).

So your module MUST provide:

### ✅ `SwerveModulePosition`
This contains:
- **distance traveled by the wheel (meters)**
- **current module angle (Rotation2d)**

That’s how the robot knows where it is.

---

# 1) SIM vs REAL (quick comparison)

## SIM module (what you already did)
- Stored `speed_mps` and `angle_rad`
- Pretended the module instantly went there

## REAL module (what we must do now)
- Read **absolute angle** from the Through Bore encoder
- Use **PID** to rotate the module to the target angle
- Use **Spark MAX velocity control** to hit the target wheel speed
- Use **optimization** so the module takes the shortest turn

---

# 2) Hardware objects the module needs (MagicBot injection)

Each module controls/reads:

### Outputs (we control)
- `drive_motor` (Spark MAX + NEO/NEO Vortex)
- `turn_motor`  (Spark MAX + NEO 550)

### Inputs (we read)
- `abs_encoder` (Through Bore absolute encoder via `DutyCycleEncoder`)
- drive motor internal encoder (from `drive_motor.getEncoder()`)

⚠️ Important concept:  
We do **NOT** create motors inside the module class.  
`robot.py` creates them, and **MagicBot injects them**.

---

# 3) File + class name (keep it simple)

Create:

```
components/swervemodule.py
```

Class name must be:

```python
class SwerveModule:
```

---

# 4) Imports (use WPIMath types — this is the “correct way”)

Type:

```python
import math
import wpilib
import rev

from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
```

### Why we use these WPIMath objects
- `Rotation2d` is the official “angle object” used everywhere in WPILib
- `SwerveModuleState` is how we describe **speed + angle**
- `SwerveModulePosition` is how we describe **distance + angle** (odometry)

Using these types makes drivetrain code cleaner and less error-prone later.

---

# 5) Helper: keep angles in a normal range (-π..+π)

Type (outside the class):

```python
def wrap_to_pi(rad: float) -> float:
    # Keeps angles from growing forever
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad <= -math.pi:
        rad += 2.0 * math.pi
    return rad
```

### Why we do this
Angles wrap around in real life:
- +π and -π point the same way.

Wrapping keeps PID math and “shortest path” logic stable.

---

# 6) Class header + MagicBot injection (correct MagicBot style)

Inside your file, type:

```python
class SwerveModule:
    # MagicBot will inject these from robot.py (createObjects)
    drive_motor: rev.SparkMax
    turn_motor: rev.SparkMax
    abs_encoder: wpilib.DutyCycleEncoder
```

✅ This is the correct way: **type hints only**  
❌ Not `= None`

---

# 7) setup(): constants + controllers + encoder conversions (simple but correct)

MagicBot calls `setup()` once after injection.

Type:

```python
    def setup(self) -> None:
        # -------------------------
        # Physical constants
        # -------------------------
        self.wheel_diameter_m = 0.0762   # 3 inch wheel in meters
        self.drive_gear_ratio = 5.08     # Medium MAXSwerve (motor rotations per wheel rotation)

        # Absolute encoder offset (radians)
        # Meaning: when wheel is pointing FORWARD, we want angle = 0 rad.
        self.abs_offset_rad = 0.0  # TODO: measure per module and set in robot constants

        # -------------------------
        # Drive encoder setup (meters + m/s)
        # -------------------------
        self.drive_encoder = self.drive_motor.getEncoder()

        wheel_circumference_m = math.pi * self.wheel_diameter_m

        # 1 motor rotation -> (1/gear_ratio) wheel rotations -> meters traveled
        self.meters_per_motor_rotation = wheel_circumference_m / self.drive_gear_ratio

        # Spark encoder velocity is RPM, so convert RPM -> m/s (divide by 60)
        self.meters_per_second_per_rpm = self.meters_per_motor_rotation / 60.0

        # Tell Spark MAX encoder to report meters and m/s directly
        cfg = rev.SparkMaxConfig()
        cfg.encoder.positionConversionFactor(self.meters_per_motor_rotation)
        cfg.encoder.velocityConversionFactor(self.meters_per_second_per_rpm)

        self.drive_motor.configure(
            cfg,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters
        )

        # Closed-loop controller for drive velocity
        self.drive_ctrl = self.drive_motor.getClosedLoopController()

        # -------------------------
        # Turning PID (we control turning with voltage)
        # -------------------------
        self.turn_pid = PIDController(4.0, 0.0, 0.2)  # TODO: tune
        self.turn_pid.enableContinuousInput(-math.pi, math.pi)

        self.max_turn_volts = 6.0  # safety limit so it doesn't slam

        # -------------------------
        # Targets (what we WANT the module to do)
        # -------------------------
        self.target_speed_mps = 0.0
        self.target_angle_rad = 0.0
```

### Explain the important variables (students must understand)
- `wheel_diameter_m`: converts wheel rotations into meters traveled
- `drive_gear_ratio`: motor spins many times for 1 wheel spin
- `abs_offset_rad`: calibrates “forward = 0 rad”
- `meters_per_motor_rotation`: converts motor rotations → meters
- `meters_per_second_per_rpm`: converts RPM → m/s
- `target_speed_mps`, `target_angle_rad`: what the drivetrain requested

---

# 8) Read the absolute encoder angle (radians)

Type:

```python
    def get_abs_angle_rad(self) -> float:
        # DutyCycleEncoder gives 0.0..1.0 for one full rotation
        turns_0_to_1 = self.abs_encoder.getAbsolutePosition()
        raw_rad = turns_0_to_1 * 2.0 * math.pi

        # Apply offset so 'forward' becomes 0 rad
        adjusted = raw_rad - self.abs_offset_rad
        return wrap_to_pi(adjusted)
```

### Why absolute encoders are used for swerve
Because the module must know wheel direction *immediately* at power-on,
without needing to “home” first.

---

# 9) Shortest path optimization (use WPILib’s built-in way)

WPILib already knows the correct math for optimizing a module state.

Type:

```python
    def set(self, speed_mps: float, angle_rad: float) -> None:
        # Create a desired state using WPILib types
        desired = SwerveModuleState(speed_mps, Rotation2d(angle_rad))

        # Current module angle from the absolute encoder
        current_angle = Rotation2d(self.get_abs_angle_rad())

        # Optimize: may flip wheel 180° and reverse speed to avoid long turns
        optimized = SwerveModuleState.optimize(desired, current_angle)

        # Store targets for execute() to use
        self.target_speed_mps = optimized.speed
        self.target_angle_rad = optimized.angle.radians()
```

### What optimization does (plain English)
If the wheel would need to rotate “too far”, it instead:
- turns 180° the other way
- reverses the drive direction

This makes swerve faster and smoother.

---

# 10) Drive control: velocity closed-loop (Spark MAX)

Type:

```python
    def _apply_drive(self) -> None:
        # Because we configured conversion factors,
        # the velocity setpoint is in meters/sec (m/s).
        self.drive_ctrl.setSetpoint(
            self.target_speed_mps,
            rev.SparkLowLevel.ControlType.kVelocity
        )
```

### Why we use velocity control
It helps the module hold the same speed even if:
- battery voltage drops
- robot hits defense
- carpet friction changes

---

# 11) Turning control: WPIMath PID → motor voltage

Type:

```python
    def _apply_turn(self) -> None:
        current = self.get_abs_angle_rad()

        # PID output is a "turn effort" to reach the target angle
        output = self.turn_pid.calculate(current, self.target_angle_rad)

        # Clamp voltage so we don't slam the turn motor
        if output > self.max_turn_volts:
            output = self.max_turn_volts
        elif output < -self.max_turn_volts:
            output = -self.max_turn_volts

        self.turn_motor.setVoltage(output)
```

### Why PID exists
Motors don’t magically jump to an angle.
PID is the “brain” that says:
- “You’re not there yet, keep turning”
- “Slow down as you get close”

---

# 12) execute(): run motor outputs every loop

Type:

```python
    def execute(self) -> None:
        # Apply drive and turn outputs using the targets from set()
        self._apply_drive()
        self._apply_turn()

        # Debug info so students can see what's happening
        wpilib.SmartDashboard.putNumber(
            "swerve/abs_angle_deg",
            math.degrees(self.get_abs_angle_rad())
        )
        wpilib.SmartDashboard.putNumber(
            "swerve/target_angle_deg",
            math.degrees(self.target_angle_rad)
        )
        wpilib.SmartDashboard.putNumber(
            "swerve/target_speed_mps",
            self.target_speed_mps
        )
```

### Why we separate set() and execute()
- `set()` is called by the drivetrain to *request* behavior
- `execute()` is called every 20ms to actually *apply* behavior

This matches your simulator design and works well with MagicBot.

---

# 13) get_state() and get_position() (THIS is how we track robot position)

These two methods are the “bridge” to odometry later.

Type:

```python
    def get_state(self) -> SwerveModuleState:
        # Current wheel speed from drive encoder (m/s)
        speed_mps = self.drive_encoder.getVelocity()

        # Current wheel angle from absolute encoder
        angle = Rotation2d(self.get_abs_angle_rad())

        return SwerveModuleState(speed_mps, angle)

    def get_position(self) -> SwerveModulePosition:
        # Total distance traveled by the wheel (meters)
        distance_m = self.drive_encoder.getPosition()

        # Current wheel angle
        angle = Rotation2d(self.get_abs_angle_rad())

        return SwerveModulePosition(distance_m, angle)
```

### Why `get_position()` is required for odometry
Odometry asks:
- “How far did each wheel travel?”
- “At what angle was that wheel pointed?”

That is exactly what `SwerveModulePosition(distance, angle)` means.

---

# 14) Reset distance (needed when drivetrain resets odometry)

We want a clean way to reset how far the wheel has traveled.

Type:

```python
    def reset_drive_distance(self) -> None:
        # Reset the drive encoder to 0 meters
        self.drive_encoder.setPosition(0.0)
```

### When will the drivetrain call this?
- at robot enable
- at start of auto
- when resetting pose from vision / known starting pose

---

# 15) One more important real-robot trick: “seed” the turn PID target at startup

If you don’t do this, the first time you enable the robot,
the module may snap to 0 rad suddenly.

So we set the initial target angle to the current absolute angle.

Add this at the **end of setup()**:

```python
        # Start by holding the current angle so it doesn't jump on enable
        self.target_angle_rad = self.get_abs_angle_rad()
```

This is a simple safety and makes the robot feel calm on boot.

---

# 16) What students must do in the shop (absolute encoder offset)

Each module needs its own offset so that:
> “wheel facing forward” means `0 radians`

Simple process:
1. Put robot on blocks
2. Point the wheel forward perfectly (use a straight edge)
3. Read the absolute encoder angle (`get_abs_angle_rad()`)
4. Save that number as the module’s `abs_offset_rad`

Later, your `robot.py` or constants file will set the correct offset per module.

---

# 17) ✅ Checklist: module is “working” when…

- Wheel angle reading makes sense (0..±π, not random)
- Module turns smoothly to the commanded angle
- Wheel speed matches command
- `get_position().distance` increases as the wheel rolls
- `reset_drive_distance()` sets distance back to 0
- On enable, module does NOT violently snap (because we seeded target angle)

---

# 18) Next guide: Swerve Drive + Odometry + AprilTag Vision

Once all 4 modules work, the drivetrain will:
- convert chassis speeds → module states
- call each module `.set(speed_mps, angle_rad)`
- update pose with `SwerveDrivePoseEstimator` using:
  - gyro yaw
  - 4x `get_position()`
- fuse Limelight AprilTag poses with `addVisionMeasurement()`

This module guide is written so that it plugs into that drivetrain guide cleanly.

