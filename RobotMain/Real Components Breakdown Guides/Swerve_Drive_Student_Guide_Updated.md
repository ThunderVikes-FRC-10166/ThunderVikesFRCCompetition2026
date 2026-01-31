# Real Swerve Drive — Student Guide
## 4 modules + navX2 yaw + odometry + AprilTag vision placeholder (Limelight)

This guide teaches how to build a **swerve drivetrain** that is **FIELD-ORIENTED** from the start.

Field-oriented means:
> “Joystick forward ALWAYS means FIELD forward, even if the robot is turned.”

This guide connects:
- ✅ 4× `SwerveModule` (your module component)
- ✅ navX2 gyro yaw (robot heading)
- ✅ `SwerveDrive4Kinematics` (chassis → module commands)
- ✅ `SwerveDrive4PoseEstimator` (odometry + vision fusion)
- ✅ Limelight AprilTag pose placeholder using NetworkTables

Beginner goal: type it step-by-step, and understand the **why** for each piece.

---

# 0) File locations and absolute imports (RobotMain/components)

Create this file:

```
RobotMain/components/swervedrive.py
```

Your module file is:

```
RobotMain/components/swervemodule.py
```

✅ We will type-hint modules using an absolute import:

```python
from RobotMain.components.swervemodule import SwerveModule
```

---

# 1) Big picture (what the drivetrain does every 20ms)

Every robot loop (~20ms), the drivetrain does:

1) Read the latest drive command `(vx, vy, omega)` from your code
2) Convert it to **field-oriented** robot motion using the gyro yaw
3) Convert robot motion → 4 module states (speed + angle)
4) Tell each module what to do (`module.set(speed_mps, angle_rad)`)
5) Update pose estimate (odometry) using wheel distances + gyro
6) If Limelight sees AprilTags, correct the pose estimate

---

# 2) Imports (REAL imports used in RobotPy)

In `RobotMain/components/swervedrive.py`, type:

```python
import wpilib
from ntcore import NetworkTableInstance

import navx

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
from wpimath.estimator import SwerveDrive4PoseEstimator

from RobotMain.components.swervemodule import SwerveModule
```

Why these matter:
- `navx` → gives yaw (robot heading)
- `Rotation2d` → avoids unit mistakes with angles
- `ChassisSpeeds.fromFieldRelativeSpeeds()` → field-oriented driving
- `PoseEstimator` → tracks pose and fuses vision
- `NetworkTables` → read Limelight AprilTag pose

---

# 3) Drivetrain class + MagicBot injection (simple)

Type:

```python
class SwerveDrive:
    # MagicBot injects these from robot.py createObjects()
    fl: SwerveModule
    fr: SwerveModule
    bl: SwerveModule
    br: SwerveModule
```

No `= None`. MagicBot injection uses type hints.

---

# 4) setup(): robot geometry, kinematics, navX, estimator, Limelight

Type:

```python
    def setup(self) -> None:
        # -------------------------
        # Robot geometry (meters)
        # -------------------------
        # Measure from robot center to module locations.
        # +X forward, +Y left.
        # Example numbers — replace with your robot measurements.
        self.fl_location = Translation2d(+0.30, +0.30)
        self.fr_location = Translation2d(+0.30, -0.30)
        self.bl_location = Translation2d(-0.30, +0.30)
        self.br_location = Translation2d(-0.30, -0.30)

        # -------------------------
        # Kinematics (swerve math)
        # -------------------------
        self.kinematics = SwerveDrive4Kinematics(
            self.fl_location,
            self.fr_location,
            self.bl_location,
            self.br_location,
        )

        # -------------------------
        # navX2 gyro (robot yaw)
        # -------------------------
        # Gyro tells us robot heading so we can do field-oriented drive.
        self.ahrs = navx.AHRS.create_spi()
        self.ahrs.reset()

        # -------------------------
        # Stored drive command
        # -------------------------
        # We store what the driver wants.
        self.cmd_vx = 0.0     # m/s field-forward
        self.cmd_vy = 0.0     # m/s field-left
        self.cmd_omega = 0.0  # rad/s CCW

        # -------------------------
        # Pose estimator (odometry + vision)
        # -------------------------
        initial_pose = Pose2d(0.0, 0.0, Rotation2d())
        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_yaw(),
            self.get_module_positions(),
            initial_pose,
        )

        # -------------------------
        # Limelight NetworkTables
        # -------------------------
        self.ll_name = "limelight"
        nt = NetworkTableInstance.getDefault()
        self.ll_table = nt.getTable(self.ll_name)
```

---

# 5) Yaw from navX2 (Rotation2d)

We use Rotation2d because WPILib odometry expects it.

Type:

```python
    def get_yaw(self) -> Rotation2d:
        yaw_deg = self.ahrs.getYaw()  # degrees
        return Rotation2d.fromDegrees(yaw_deg)
```

If your robot drives “backwards field-oriented”, flip the sign here:
- `Rotation2d.fromDegrees(-yaw_deg)`

---

# 6) Get module positions (odometry needs this)

Type:

```python
    def get_module_positions(self):
        return (
            self.fl.get_position(),
            self.fr.get_position(),
            self.bl.get_position(),
            self.br.get_position(),
        )
```

This connects directly to your `SwerveModule.get_position()` from the module guide.

---

# 7) drive(vx, vy, omega) — FIELD coordinates from the start

Important: In this guide, `vx` and `vy` are **FIELD-relative** commands.

That means:
- `vx > 0` = “go toward the opponent wall” (field forward)
- `vy > 0` = “go left on the field” (field left)

Type:

```python
    def drive(self, vx: float, vy: float, omega: float) -> None:
        # Store FIELD-relative commands. execute() will convert to robot-relative.
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_omega = omega
```

---

# 8) Field-oriented conversion (the key step)

This is the “magic” of field-oriented drive:

WPILib provides a helper that converts field commands to robot commands:
- uses current yaw angle

Type:

```python
    def _get_robot_relative_speeds(self) -> ChassisSpeeds:
        # Convert FIELD commands to ROBOT commands using gyro yaw.
        # This makes joystick forward always mean field forward.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            self.cmd_vx,
            self.cmd_vy,
            self.cmd_omega,
            self.get_yaw(),
        )
```

Beginner explanation:
- If the robot is rotated 90 degrees, “field forward” is “robot left”.
- This function does that rotation math for you.

---

# 9) Robot-relative speeds → module states

Type:

```python
    def _compute_module_states(self):
        robot_speeds = self._get_robot_relative_speeds()
        states = self.kinematics.toSwerveModuleStates(robot_speeds)
        return states
```

---

# 10) Apply states to modules (connects to your module API)

Type:

```python
    def _apply_states(self, states) -> None:
        self.fl.set(states[0].speed, states[0].angle.radians())
        self.fr.set(states[1].speed, states[1].angle.radians())
        self.bl.set(states[2].speed, states[2].angle.radians())
        self.br.set(states[3].speed, states[3].angle.radians())
```

This works because your module guide provides:
- `set(speed_mps, angle_rad)`

---

# 11) Update odometry (pose estimator) every loop

Type:

```python
    def update_odometry(self) -> None:
        self.pose_estimator.update(
            self.get_yaw(),
            self.get_module_positions(),
        )
```

This is the robot’s “best guess pose” from wheel distances + gyro.

---

# 12) AprilTag vision placeholder (Limelight) — correction to pose

### Why vision is needed
Wheel odometry drifts over time (wheel slip, defense, bumps).
AprilTags give a “global” reference to correct drift.

### Simple placeholder method (NetworkTables)
Type:

```python
    def try_add_vision_measurement(self) -> None:
        # tv = 1 means Limelight currently has a valid target
        tv = self.ll_table.getNumber("tv", 0)
        if tv < 1:
            return

        # botpose_wpiblue is an array. We use x, y, yaw.
        botpose = self.ll_table.getNumberArray("botpose_wpiblue", [])
        if len(botpose) < 6:
            return

        x = botpose[0]        # meters
        y = botpose[1]        # meters
        yaw_deg = botpose[5]  # degrees

        vision_pose = Pose2d(x, y, Rotation2d.fromDegrees(yaw_deg))

        # Simple timestamp correction using Limelight pipeline latency (ms)
        latency_ms = self.ll_table.getNumber("tl", 0.0)
        timestamp = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)

        self.pose_estimator.addVisionMeasurement(vision_pose, timestamp)
```

This is intentionally simple so beginners can understand it.
Later, you can improve accuracy by using Limelight’s full timestamp/latency fields.

---

# 13) get_pose() and reset_pose()

Type:

```python
    def get_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()
```

Reset pose when you know where you are (start of auto, etc.).

```python
    def reset_pose(self, pose: Pose2d) -> None:
        # Reset wheel distances so odometry starts clean
        self.fl.reset_drive_distance()
        self.fr.reset_drive_distance()
        self.bl.reset_drive_distance()
        self.br.reset_drive_distance()

        self.pose_estimator.resetPosition(
            self.get_yaw(),
            self.get_module_positions(),
            pose,
        )
```

---

# 14) execute(): main drivetrain loop (runs every 20ms)

Type:

```python
    def execute(self) -> None:
        # 1) Convert field commands → module states
        states = self._compute_module_states()

        # 2) Tell modules what to do
        self._apply_states(states)

        # 3) Update pose (odometry)
        self.update_odometry()

        # 4) Correct pose with AprilTags when available
        self.try_add_vision_measurement()

        # 5) Debug on dashboard for students
        pose = self.get_pose()
        wpilib.SmartDashboard.putNumber("pose/x_m", pose.X())
        wpilib.SmartDashboard.putNumber("pose/y_m", pose.Y())
        wpilib.SmartDashboard.putNumber("pose/deg", pose.rotation().degrees())
```

---

# 15) Student checklist to verify field-oriented behavior

✅ **Test 1: joystick forward**
- Point robot forward, push forward → goes forward
- Rotate robot 90° to the right, push forward → still goes field forward (robot will strafe)

✅ **Test 2: odometry**
- Drive forward ~1 meter → X increases
- Strafe left ~1 meter → Y increases

✅ **Test 3: vision correction**
- Drive around (pose drifts)
- Let Limelight see a tag → pose should jump closer to truth

---

# 16) Common beginner mistakes (and what to check)

1) Robot drives “field backward” → invert yaw sign in `get_yaw()`
2) Robot spins weird → module locations (Translation2d) are wrong
3) Pose doesn’t update → module `get_position()` not returning meters
4) Vision pose is wrong → Limelight configuration / wrong botpose key

---

You now have a **field-oriented** drivetrain from the very beginning, and it plugs directly into your `SwerveModule` API.

