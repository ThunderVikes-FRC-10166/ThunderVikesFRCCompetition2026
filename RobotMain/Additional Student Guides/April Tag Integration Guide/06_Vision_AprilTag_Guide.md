# Guide 06: Vision, AprilTags, and Pose Estimation

## What You'll Learn

In this guide you will learn how to use a **Limelight camera** (or PhotonVision) to detect **AprilTags** on the FRC field, and use those detections to keep the robot's position estimate accurate. By the end, your robot will know where it is on the field — not just from wheel odometry, but from actually "seeing" landmarks.

---

## Before You Start

Make sure you have completed the earlier guides (especially the swerve drive). The robot should already be driving and tracking its position with wheel odometry.

---

# Part 1: Understanding the Problem

## What Is Odometry?

Your robot already tracks its position using **odometry** — it counts how many times the wheels spin and which direction the gyroscope says the robot is facing, then does math to figure out "I've moved 2 meters forward and 1 meter left."

This works well for a short time, but it has a big problem: **drift**.

Every small error adds up. If a wheel slips on the carpet, or the gyro is off by half a degree, those tiny errors accumulate over time. After driving around the field for 30 seconds, the robot might think it's a whole meter away from where it actually is.

## What Are AprilTags?

AprilTags are black-and-white square patterns printed on signs placed around the FRC field. Each tag has a unique ID number (like tag #1, tag #2, etc.) and is placed at a known, exact position on the field.

Think of them like street signs — if you can see the sign, you know exactly where you are on the map.

The FRC field has tags placed at specific locations every year. The WPILib library includes a JSON file with all the tag positions for the current season.

## What Is a Limelight?

A **Limelight** is a special camera designed for FRC robots. It does all the heavy image processing on its own computer, then publishes the results to **NetworkTables** (a shared data system that all robot code can read from).

When a Limelight sees an AprilTag, it calculates:
- Which tag it sees (`tid` = tag ID)
- How far off-center the tag is (`tx` = horizontal angle)
- The robot's estimated position on the field (`botpose_wpiblue`)
- How long the calculation took (latency, embedded in the `botpose_wpiblue` array)

Your robot code just reads these numbers from NetworkTables — it never talks to the camera directly.

**PhotonVision** is a free, open-source alternative that works similarly. The code patterns in this guide apply to both.

## The Solution: Pose Estimator

Instead of using basic odometry alone, we use a **Pose Estimator** — a smarter system that blends two sources:

1. **Wheel odometry** (very frequent, slightly drifty)
2. **Vision measurements** (less frequent, more accurate when available)

The Pose Estimator uses a **Kalman Filter** (a math technique) to figure out the best estimate of where the robot actually is, weighing both sources based on how much we trust each one.

It's like having a GPS in your car. The speedometer (odometry) tracks your speed continuously, but GPS (vision) corrects your position every few seconds.

---

# Part 2: The Constants

Open `RobotMain/constants.py` (and `RobotMain/components/constants.py` — they must stay identical) and add these constants:

```python
# =============================================================================
# VISION / LIMELIGHT CONSTANTS
# =============================================================================
# The Limelight camera detects AprilTags on the field to help the robot
# know exactly where it is. This works alongside the wheel odometry.
#
# The Limelight publishes data to NetworkTables automatically.
# We read from the "limelight" table (the default table name).

kLimelightTableName = "limelight"

kVisionStdDevX = 0.9
kVisionStdDevY = 0.9
kVisionStdDevTheta = 0.9

kOdometryStdDevX = 0.1
kOdometryStdDevY = 0.1
kOdometryStdDevTheta = 0.1

kVisionMaxAcceptableDistance = 2.0

kCameraForwardOffsetMeters = 0.30
kCameraLeftOffsetMeters = 0.00
kVisionMaxTagDistanceMeters = 5.0
kVisionFovHalfAngleDegrees = 30.0
```

### What Do These Mean?

| Constant | Value | What It Means |
|---|---|---|
| `kLimelightTableName` | `"limelight"` | The NetworkTables table name where the Limelight publishes data. If you renamed your Limelight (like "limelight-front"), change this. |
| `kVisionStdDevX/Y/Theta` | `0.9` | **Standard deviation** for vision measurements. Higher number = trust vision LESS. 0.9 means "vision is somewhat noisy." |
| `kOdometryStdDevX/Y/Theta` | `0.1` | Standard deviation for wheel odometry. Lower number = trust odometry MORE. 0.1 means "wheels are pretty reliable." |
| `kVisionMaxAcceptableDistance` | `2.0` | If vision says the robot is more than 2 meters from where odometry thinks, we ignore it (probably bad data). |
| `kCameraForwardOffsetMeters` | `0.30` | How far forward the camera is from the robot center (in meters). Adjust for your robot's camera mounting position. |
| `kCameraLeftOffsetMeters` | `0.00` | How far left the camera is from center. 0 means it's centered. |
| `kVisionMaxTagDistanceMeters` | `5.0` | Maximum distance (meters) the camera can reliably see a tag. |
| `kVisionFovHalfAngleDegrees` | `30.0` | Half of the camera's field of view (degrees). The Limelight has about a 60-degree total FOV. |

### Understanding Standard Deviations (Trust Levels)

Standard deviation is a statistics concept, but for our purposes it's simple:

- **Lower number = more trust**. We set odometry to 0.1 because wheel encoders are very reliable on a cycle-by-cycle basis.
- **Higher number = less trust**. We set vision to 0.9 because camera measurements can be noisy (lighting, reflections, partial tag visibility).

The Pose Estimator uses these numbers to decide how much weight to give each source. If vision says "you're at X=5" but odometry says "you're at X=4.5", the estimator will blend them — leaning more toward odometry since we trust it more per-measurement.

You can tune these values:
- If your camera gives very accurate results, lower the vision stddev (e.g., 0.5)
- If your wheels slip a lot, raise the odometry stddev (e.g., 0.3)

---

# Part 3: Upgrading from Odometry to Pose Estimator

This is the most important code change. We replace the basic `SwerveDrive4Odometry` with `SwerveDrive4PoseEstimator`.

## Step 1: Add the Import

In `RobotMain/components/swerve_drive.py`, add this import near the top:

```python
import wpimath.estimator
```

## Step 2: Replace the Odometry Object

In the `setup()` method, find where the odometry object is created and replace it with a Pose Estimator:

**Remove this** (basic odometry):
```python
self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
    self.kinematics,
    self.gyro.getRotation2d(),
    (
        self.front_left.get_position(),
        self.front_right.get_position(),
        self.rear_left.get_position(),
        self.rear_right.get_position(),
    ),
    wpimath.geometry.Pose2d(),
)
```

**Add this** (pose estimator with vision support):
```python
self.pose_estimator = wpimath.estimator.SwerveDrive4PoseEstimator(
    self.kinematics,
    self.gyro.getRotation2d(),
    (
        self.front_left.get_position(),
        self.front_right.get_position(),
        self.rear_left.get_position(),
        self.rear_right.get_position(),
    ),
    wpimath.geometry.Pose2d(),
    (constants.kOdometryStdDevX, constants.kOdometryStdDevY, constants.kOdometryStdDevTheta),
    (constants.kVisionStdDevX, constants.kVisionStdDevY, constants.kVisionStdDevTheta),
)
```

### What Changed?

The constructor looks almost identical, but with two extra arguments:

1. **Odometry standard deviations** `(0.1, 0.1, 0.1)` — how much to trust the wheels+gyro
2. **Vision standard deviations** `(0.9, 0.9, 0.9)` — how much to trust the camera

The Pose Estimator is a drop-in replacement — it has the same `update()` method for wheel data, plus a new `addVisionMeasurement()` method for camera data.

## Step 3: Add the Limelight NetworkTable

Still in `setup()`, add the table connection right after the pose estimator:

```python
self.limelight_table = ntcore.NetworkTableInstance.getDefault().getTable(
    constants.kLimelightTableName
)
self.limelight_connected = False
```

This connects to the NetworkTables table where the Limelight publishes its data. On a real robot, the Limelight does this automatically over the network. In simulation, our `physics.py` will publish fake data.

## Step 4: Update the execute() Method

In `execute()`, change the odometry update call:

**Replace this:**
```python
self.odometry.update(
    self.gyro.getRotation2d(),
    (
        self.front_left.get_position(),
        self.front_right.get_position(),
        self.rear_left.get_position(),
        self.rear_right.get_position(),
    ),
)
```

**With this:**
```python
self.pose_estimator.update(
    self.gyro.getRotation2d(),
    (
        self.front_left.get_position(),
        self.front_right.get_position(),
        self.rear_left.get_position(),
        self.rear_right.get_position(),
    ),
)
```

Same call, just on `pose_estimator` instead of `odometry`.

## Step 5: Update get_pose() and reset_odometry()

Change these methods to use the pose estimator:

```python
def get_pose(self) -> wpimath.geometry.Pose2d:
    return self.pose_estimator.getEstimatedPosition()

def reset_odometry(self, pose: wpimath.geometry.Pose2d) -> None:
    self.pose_estimator.resetPosition(
        self.gyro.getRotation2d(),
        (
            self.front_left.get_position(),
            self.front_right.get_position(),
            self.rear_left.get_position(),
            self.rear_right.get_position(),
        ),
        pose,
    )
```

Notice `getEstimatedPosition()` instead of `getPose()` — the Pose Estimator uses a different method name because it's an *estimate* that blends multiple sources, not just raw odometry.

---

# Part 4: Reading Limelight Data (The _update_limelight Method)

This is where the magic happens. Every cycle, we check if the Limelight sees a tag, and if so, feed that measurement into the Pose Estimator.

Replace the old stub `_update_limelight()` method with this:

```python
def _update_limelight(self) -> None:
    tv = self.limelight_table.getNumber("tv", 0)

    if tv < 1:
        self.limelight_connected = False
        return

    botpose = self.limelight_table.getNumberArray("botpose_wpiblue", [])
    if len(botpose) < 7:
        self.limelight_connected = False
        return

    vision_x = botpose[0]
    vision_y = botpose[1]
    vision_yaw = botpose[5]
    latency_ms = botpose[6]

    vision_pose = wpimath.geometry.Pose2d(
        vision_x,
        vision_y,
        wpimath.geometry.Rotation2d.fromDegrees(vision_yaw),
    )

    current_pose = self.pose_estimator.getEstimatedPosition()
    distance_from_estimate = current_pose.translation().distance(
        vision_pose.translation()
    )

    if distance_from_estimate > constants.kVisionMaxAcceptableDistance:
        return

    timestamp = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)

    self.pose_estimator.addVisionMeasurement(vision_pose, timestamp)
    self.limelight_connected = True
```

### Line-by-Line Explanation

**Reading from NetworkTables:**
```python
tv = self.limelight_table.getNumber("tv", 0)
```
`tv` stands for "target visible." The Limelight sets this to `1` when it sees a tag, `0` when it doesn't. If no tag is visible, we skip everything — there's nothing to update.

**Getting the robot pose:**
```python
botpose = self.limelight_table.getNumberArray("botpose_wpiblue", [])
```
`botpose_wpiblue` is an array of 7 numbers that the Limelight calculates:
- `[0]` = X position on the field (meters)
- `[1]` = Y position on the field (meters)
- `[2]` = Z position (height, usually 0)
- `[3]` = Roll (tilt left/right)
- `[4]` = Pitch (tilt forward/back)
- `[5]` = Yaw (rotation, in degrees)
- `[6]` = Latency in milliseconds (how long the calculation took)

We use the `_wpiblue` variant because it uses the WPILib "blue alliance origin" coordinate system — the origin (0,0) is always at the blue alliance wall, regardless of which alliance you're on.

**Building a Pose2d:**
```python
vision_pose = wpimath.geometry.Pose2d(
    vision_x, vision_y,
    wpimath.geometry.Rotation2d.fromDegrees(vision_yaw),
)
```
We convert the raw numbers into a WPILib `Pose2d` object that the Pose Estimator can use.

**Sanity check (reject bad data):**
```python
distance_from_estimate = current_pose.translation().distance(vision_pose.translation())
if distance_from_estimate > constants.kVisionMaxAcceptableDistance:
    return
```
This is a safety check. If the camera says the robot is more than 2 meters from where we think we are, something is probably wrong (bad lighting, seeing a reflection, etc.). We throw that measurement away.

**Accounting for latency:**
```python
timestamp = wpilib.Timer.getFPGATimestamp() - (latency_ms / 1000.0)
```
The camera takes some time to process the image. We need to tell the Pose Estimator *when* this measurement was actually taken (a few milliseconds ago), not when we read it. This lets the estimator handle the time delay correctly.

**Feeding the measurement:**
```python
self.pose_estimator.addVisionMeasurement(vision_pose, timestamp)
```
This is the key line. We give the Pose Estimator the vision-based pose and the timestamp. The estimator blends it with the wheel odometry using the standard deviations we set up, producing a better overall position estimate.

---

# Part 5: Simulating Vision in physics.py

To test vision in the simulator (without a real camera), we add fake Limelight data to `physics.py`. The simulator publishes to the same NetworkTables keys that a real Limelight would, so the robot code works exactly the same way.

### Step 1: Add imports

At the top of `physics.py`:

```python
from ntcore import NetworkTableInstance
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d
```

### Step 2: Set up the fake Limelight in __init__

```python
self.ll_table = NetworkTableInstance.getDefault().getTable(constants.kLimelightTableName)
try:
    self.tag_layout = AprilTagFieldLayout("resources/2026-rebuilt-welded.json")
    self.vision_enabled = True
except Exception:
    self.vision_enabled = False

self.cam_forward_m = constants.kCameraForwardOffsetMeters
self.cam_left_m = constants.kCameraLeftOffsetMeters
self.max_tag_distance_m = constants.kVisionMaxTagDistanceMeters
self.fov_half_angle_deg = constants.kVisionFovHalfAngleDegrees
```

The `AprilTagFieldLayout` loads the official field JSON file with all the tag positions. If the file isn't found, vision simulation is disabled but the robot still runs fine.

### Step 3: Add the tag detection helper

This function checks if any AprilTag is visible from the robot's current position:

```python
def _pick_visible_tag(self, robot_pose_2d):
    if not self.vision_enabled:
        return None

    heading = robot_pose_2d.rotation()
    cosA = heading.cos()
    sinA = heading.sin()

    cam_dx = self.cam_forward_m * cosA - self.cam_left_m * sinA
    cam_dy = self.cam_forward_m * sinA + self.cam_left_m * cosA

    cam_x = robot_pose_2d.X() + cam_dx
    cam_y = robot_pose_2d.Y() + cam_dy

    best = None

    for tag in self.tag_layout.getTags():
        tag_id = tag.ID
        tag_pose: Pose3d = tag.pose

        dx = tag_pose.X() - cam_x
        dy = tag_pose.Y() - cam_y
        dist = math.hypot(dx, dy)

        if dist > self.max_tag_distance_m:
            continue

        rel_x = dx * cosA + dy * sinA
        rel_y = -dx * sinA + dy * cosA

        if rel_x <= 0.01:
            continue

        tx_deg = math.degrees(math.atan2(rel_y, rel_x))

        if abs(tx_deg) > self.fov_half_angle_deg:
            continue

        if best is None or dist < best[0]:
            best = (dist, tag_id, tx_deg)

    if best is None:
        return None

    dist, tag_id, tx_deg = best
    return (tag_id, tx_deg, dist)
```

This simulates the camera's field of view:
- Calculate where the camera is on the robot
- For each tag on the field, check if it's close enough and in front of the camera
- Return the closest visible tag

### Step 4: Publish fake Limelight data

```python
def _update_vision_sim(self, now, tm_diff):
    pose = self.physics_controller.get_pose()

    seen = self._pick_visible_tag(pose)

    if seen is None:
        self.ll_table.putNumber("tv", 0)
        self.ll_table.putNumber("tid", -1)
        self.ll_table.putNumber("tx", 0.0)
        self.ll_table.putNumber("ty", 0.0)
    else:
        tag_id, tx_deg, dist_m = seen
        self.ll_table.putNumber("tv", 1)
        self.ll_table.putNumber("tid", tag_id)
        self.ll_table.putNumber("tx", tx_deg)
        self.ll_table.putNumber("ty", 0.0)

        yaw_deg = math.degrees(self.sim_heading)
        botpose = [pose.X(), pose.Y(), 0.0, 0.0, 0.0, yaw_deg, 11.0]
        self.ll_table.putNumberArray("botpose_wpiblue", botpose)
```

Then call `self._update_vision_sim(now, tm_diff)` at the end of `update_sim()`.

---

# Part 6: How to Use Vision for Autonomous Navigation

Once vision is working, you can use the corrected pose to drive to specific field locations during autonomous mode.

### Simple "Drive to Pose" Approach

Here's a snippet showing how an autonomous routine could use the corrected pose to drive to a specific location:

```python
def drive_to_pose(self, target_pose, tolerance=0.1):
    """Drive toward a target pose using the corrected position estimate."""
    current_pose = self.swerve_drive.get_pose()
    
    dx = target_pose.X() - current_pose.X()
    dy = target_pose.Y() - current_pose.Y()
    distance = math.hypot(dx, dy)
    
    if distance < tolerance:
        return True  # We've arrived!
    
    # Normalize and scale the speed
    speed = min(distance * 0.5, constants.kAutoMaxSpeed)
    x_speed = (dx / distance) * speed / constants.kMaxSpeed
    y_speed = (dy / distance) * speed / constants.kMaxSpeed
    
    self.swerve_drive.set_drive_command(x_speed, y_speed, 0.0, True, False)
    return False  # Still driving
```

Because `get_pose()` now returns the vision-corrected position, this navigation will be much more accurate than using odometry alone.

### AprilTag-Based Alignment

You can also align to a specific tag (useful for scoring):

```python
# Read the tag offset from NetworkTables
tx = limelight_table.getNumber("tx", 0.0)

# If the tag is off-center, rotate to center it
if abs(tx) > 1.0:  # More than 1 degree off
    rot_speed = -tx * 0.02  # Proportional control
    swerve_drive.set_drive_command(0, 0, rot_speed, False, False)
```

This continuously adjusts the robot's rotation until the AprilTag is centered in the camera view — perfect for lining up a shot.

---

# Part 7: Setting Up a Real Limelight

When you move from simulation to a real robot:

1. **Mount the Limelight** on the front of the robot, pointing forward
2. **Connect it** to the robot's network (Ethernet or radio)
3. **Configure it** via the Limelight web interface (`http://10.TE.AM.11:5801`)
4. **Set the pipeline** to "AprilTag" mode
5. **Update `kCameraForwardOffsetMeters`** and `kCameraLeftOffsetMeters` to match the actual mounting position

The Limelight automatically publishes to the same NetworkTables keys (`tv`, `tid`, `tx`, `botpose_wpiblue`) that we read in the code. Your robot code does not change.

**Remove the simulation vision code** from `physics.py` when testing on the real robot — otherwise the fake data could conflict with real camera data. (The simulation code only runs when you use `--sim` anyway, so this typically isn't a problem.)

---

# Part 8: PhotonVision Alternative

PhotonVision is a free, open-source alternative to Limelight. If your team uses a Raspberry Pi or Orange Pi with a camera instead of a Limelight, you would:

1. Install PhotonVision on the coprocessor
2. Install the Python library: add `photonlibpy` to your `pyproject.toml`
3. Use the PhotonVision API instead of NetworkTables directly

The concept is identical — the camera detects tags and gives you a pose estimate. The main code difference is using PhotonVision's library instead of reading raw NetworkTables:

```python
# PhotonVision approach (for reference — not needed if using Limelight)
from photonlibpy.photonCamera import PhotonCamera

camera = PhotonCamera("your_camera_name")
result = camera.getLatestResult()
if result.hasTargets():
    best_target = result.getBestTarget()
    # Use the target data to estimate pose...
```

For most FRC teams, the Limelight approach (reading NetworkTables directly) is simpler and what we use in this codebase.

---

# Summary

| Concept | What It Does |
|---|---|
| **Odometry** | Tracks position using wheel encoders + gyro. Accurate short-term but drifts over time. |
| **AprilTags** | Known landmarks on the field that the camera can detect. |
| **Limelight** | Camera that detects AprilTags and publishes pose estimates to NetworkTables. |
| **Pose Estimator** | Blends odometry + vision measurements using a Kalman Filter for the best position estimate. |
| **Standard Deviations** | Numbers that control how much to trust each data source (lower = more trust). |
| **botpose_wpiblue** | The Limelight's estimate of the robot's position using the blue-alliance coordinate system. |
| **Sanity Check** | We reject vision measurements that are too far from the current estimate (probably bad data). |

---

# Checklist Before Moving On

- [ ] Vision constants added to both `RobotMain/constants.py` and `RobotMain/components/constants.py`
- [ ] `SwerveDrive4Odometry` replaced with `SwerveDrive4PoseEstimator` in `swerve_drive.py`
- [ ] `_update_limelight()` method reads from NetworkTables and calls `addVisionMeasurement()`
- [ ] `get_pose()` uses `getEstimatedPosition()` instead of `getPose()`
- [ ] `reset_odometry()` uses `pose_estimator.resetPosition()` instead of `odometry.resetPosition()`
- [ ] `physics.py` loads the AprilTag field layout and publishes fake Limelight data
- [ ] Simulation runs without errors (`python -m robotpy sim --nogui`)
- [ ] Camera mount position (`kCameraForwardOffsetMeters`, `kCameraLeftOffsetMeters`) matches your actual robot

---

# What's Next?

Now that your robot can see where it is on the field, you're ready to build smarter autonomous routines! See **Guide 07: Autonomous Strategies** to learn how to plan autonomous modes that use (or don't use) vision to accomplish match objectives.
